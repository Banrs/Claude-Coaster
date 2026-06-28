// Simple procedural ride audio (no asset files): a wind/rumble bed whose
// loudness + brightness scale with speed, plus a launch "whoosh" swell on boost.
// Synthesised live in an AVAudioSourceNode render callback. Headless --shot/--bench
// never start it.
#pragma once
#import <AVFoundation/AVFoundation.h>
#include <atomic>
#include <cmath>

struct RideAudio {
    AVAudioEngine*     engine = nil;
    AVAudioSourceNode* src    = nil;
    // shared, lock-free-ish: written from the main thread, read in the audio thread
    std::atomic<float> speed{40.0f};     // m/s
    std::atomic<float> whoosh{0.0f};     // 0..1 launch swell, decays over ~1s

    void start() {
        engine = [[AVAudioEngine alloc] init];
        AVAudioFormat* fmt = [[AVAudioFormat alloc] initStandardFormatWithSampleRate:44100.0 channels:2];

        // capture the atomics by pointer (the block outlives this scope)
        std::atomic<float>* spd = &speed;
        std::atomic<float>* wh  = &whoosh;
        __block double phase = 0.0;           // low rumble oscillator
        __block float  rng   = 0.0f;          // simple noise state
        __block float  lp    = 0.0f;          // low-pass state for wind
        __block float  hp    = 0.0f;          // residual-air state (brightness)
        __block float  air   = 0.0f;          // band-limited "air" (the hiss, gently rolled off)

        src = [[AVAudioSourceNode alloc] initWithFormat:fmt
                renderBlock:^OSStatus(BOOL* isSilence, const AudioTimeStamp* ts,
                                      AVAudioFrameCount n, AudioBufferList* abl) {
            (void)isSilence; (void)ts;
            float v  = spd->load();
            float ws = wh->load();
            // map speed (8..120 m/s) to wind loudness + brightness
            float t  = fminf(fmaxf((v - 8.0f) / 100.0f, 0.0f), 1.0f);
            float windAmp = 0.05f + 0.26f * t * t;
            float bright  = 0.03f + 0.13f * t;             // less high-freq content -> a smooth rush, not sandy hiss
            float rumbleAmp = (0.05f + 0.10f * t);
            float rumbleHz  = 38.0f + 26.0f * t;
            for (UInt32 b = 0; b < abl->mNumberBuffers; b++) {
                float* out = (float*)abl->mBuffers[b].mData;
                for (AVAudioFrameCount i = 0; i < n; i++) {
                    // white noise -> wind: TWO-POLE low-pass for a smooth airy body (the old
                    // single light LP + heavy high-pass blend was the "sandy" graininess).
                    float white = ((float)((rand() & 0xffff) / 32768.0f) - 1.0f);
                    lp += (white - lp) * 0.018f;           // pole 1 (heavier smoothing)
                    rng += (lp - rng) * 0.10f;             // pole 2 (rng = smoothed wind body)
                    hp = white - lp;                       // residual top-end (full-band: this was the grit)
                    air += (hp - air) * 0.45f;             // band-limit it -> soft "air", not gritty Nyquist hiss
                    float wind = (rng * (1.0f - bright) + air * bright) * windAmp;
                    // low engine rumble (sine + its octave)
                    phase += (rumbleHz / 44100.0) * 2.0 * M_PI;
                    if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
                    float rumble = (sinf((float)phase) * 0.7f + sinf((float)phase * 2.0f) * 0.3f) * rumbleAmp;
                    // launch whoosh: a swell of the SMOOTH wind body (not raw hiss) with ws
                    float whooshSig = rng * ws * 0.6f;
                    float s = wind + rumble + whooshSig;
                    out[i] = tanhf(s);                     // soft saturation (no harsh hard-clip on whoosh peaks)
                }
            }
            return noErr;
        }];

        [engine attachNode:src];
        [engine connect:src to:engine.mainMixerNode format:fmt];
        NSError* err = nil;
        if (![engine startAndReturnError:&err])
            fprintf(stderr, "audio: engine start failed: %s\n",
                    [[err localizedDescription] UTF8String]);
    }

    void setSpeed(float v) { speed.store(v); }
    // call when SPACE fires a boost/launch -> a quick swell
    void triggerWhoosh()   { whoosh.store(1.0f); }
    // decay the whoosh a little each frame
    void tick(float dt)    { float w = whoosh.load(); whoosh.store(fmaxf(w - dt * 1.4f, 0.0f)); }
};
