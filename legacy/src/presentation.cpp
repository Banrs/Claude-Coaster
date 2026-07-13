static Sound makeClackSound() {
    const int sr = 44100; const float dur = 0.05f;
    int n = (int)(sr * dur);
    short *d = (short *)RL_MALLOC(n * sizeof(short));
    float y = 0;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sr;
        float x = frnd(-1, 1);
        y += 0.45f * (x - y);
        d[i] = (short)(y * expf(-t * 110.0f) * 9000);
    }
    Wave w = { (unsigned)n, 44100, 16, 1, d };
    Sound s = LoadSoundFromWave(w);
    RL_FREE(d);
    return s;
}
static Sound makeWhooshSound() {
    const int sr = 44100; const float dur = 0.8f;
    int n = (int)(sr * dur);
    short *d = (short *)RL_MALLOC(n * sizeof(short));
    float y = 0;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sr;
        float p = t / dur;
        float x = frnd(-1, 1);

        float cut = 0.05f + 0.30f * sinf(PI * p);
        y += cut * (x - y);
        float env = powf(sinf(PI * p), 1.3f);
        d[i] = (short)(y * env * 17000);
    }
    Wave w = { (unsigned)n, 44100, 16, 1, d };
    Sound s = LoadSoundFromWave(w);
    RL_FREE(d);
    return s;
}

static volatile float g_windVol   = 0.0f;
static volatile float g_rumbleVol = 0.0f;
static void windCallback(void *buffer, unsigned int frames) {
    short *d = (short *)buffer;
    static uint32_t ar = 0x9e3779b9u;
    static float lp = 0, hp = 0, rmb = 0, sm = 0, smR = 0;
    float target = g_windVol, targetR = g_rumbleVol;
    for (unsigned int i = 0; i < frames; i++) {
        sm  += (target  - sm)  * 0.0006f;
        smR += (targetR - smR) * 0.0006f;
        ar ^= ar << 13; ar ^= ar >> 17; ar ^= ar << 5;
        float white = ((int)(ar & 0xffff) - 32768) / 32768.0f;
        lp  += 0.06f * (white - lp);
        hp  += 0.40f * (white - hp);
        rmb += 0.012f * (white - rmb);
        float wind   = (lp * 0.65f + hp * 0.35f) * sm * sm;
        float rumble = rmb * 4.0f * smR;
        int v = (int)((wind * 27000.0f) + (rumble * 30000.0f));
        d[i] = (short)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
    }
}
static void textSh(const char *s, int x, int y, int size, Color c) {
    DrawText(s, x + 2, y + 2, size, Color{ 20, 20, 30, 200 });
    DrawText(s, x, y, size, c);
}

static void hudPanel(float x, float y, float w, float h, Color fill = Color{ 18, 22, 34, 168 }) {
    Rectangle r = { x, y, w, h };
    DrawRectangleRounded(r, 0.32f, 6, fill);
    DrawRectangleRoundedLines(r, 0.32f, 6, Color{ 150, 168, 200, 70 });
    DrawRectangleRounded(Rectangle{ x + 5, y + 3, w - 10, 2 }, 1.0f, 3, Color{ 220, 232, 255, 36 });
}
