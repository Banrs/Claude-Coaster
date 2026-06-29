#pragma once
//
// Hud.h - minimal self-contained ASCII text overlay data generator.
//
// Header-only. Produces:
//   (a) a single-channel R8 font atlas as CPU pixel data (hud::fontAtlas)
//   (b) textured quad vertex data for arbitrary strings (hud::addText)
//
// It has NO dependency on Vulkan or any other project header. The integrator
// is expected to upload the atlas to a VkImage and the vertices to a vertex
// buffer, then draw with the accompanying hud.vert / hud.frag shaders.
//
// Coordinate conventions:
//   * Vertex positions are in PIXELS, top-left origin (x right, y down).
//   * UVs are in [0,1].
//
#include <vector>
#include <cstdint>
#include <string>

namespace hud {

// ---------------------------------------------------------------------------
// Public data structures
// ---------------------------------------------------------------------------

struct Vtx {
    float x, y;        // position in pixels, top-left origin
    float u, v;        // texture coordinate, [0,1]
    float r, g, b, a;  // colour + alpha (final alpha = atlas.r * a)
};

struct Atlas {
    std::vector<uint8_t> pixels; // single-channel R8; 255 = ink, 0 = empty
    int w = 0, h = 0;
};

// ---------------------------------------------------------------------------
// Font definition
// ---------------------------------------------------------------------------
//
// 8x8 bitmap font covering ASCII 32..126 (95 glyphs). Each glyph is 8 bytes,
// one byte per row (top row first). Within a byte, bit 0 (LSB, value 1) is the
// LEFTMOST pixel and bit 7 is the rightmost. Missing glyphs are all-zero and
// render blank.
//
// This is a compact rendition of a classic public-domain 8x8 console font.
//
namespace detail {

static constexpr int kFirstChar = 32;   // ' '
static constexpr int kLastChar  = 126;  // '~'
static constexpr int kNumChars  = kLastChar - kFirstChar + 1; // 95
static constexpr int kGlyphW    = 8;
static constexpr int kGlyphH    = 8;
static constexpr int kCols      = 16;   // atlas grid columns
static constexpr int kPad       = 0;    // padding between cells (px)

// Bitmaps for ASCII 32..126, 8 bytes each. LSB = leftmost pixel.
static constexpr uint8_t kFont[kNumChars][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // 33 '!'
    {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00}, // 34 '"'
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // 35 '#'
    {0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00}, // 36 '$'
    {0x00,0xC6,0xCC,0x18,0x30,0x66,0xC6,0x00}, // 37 '%'
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // 38 '&'
    {0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00}, // 39 '\''
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // 40 '('
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // 41 ')'
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // 42 '*'
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // 43 '+'
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // 44 ','
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // 45 '-'
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // 46 '.'
    {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, // 47 '/'
    {0x7C,0xC6,0xCE,0xDE,0xF6,0xE6,0x7C,0x00}, // 48 '0'
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 49 '1'
    {0x7C,0xC6,0x06,0x1C,0x70,0xC0,0xFE,0x00}, // 50 '2'
    {0x7C,0xC6,0x06,0x3C,0x06,0xC6,0x7C,0x00}, // 51 '3'
    {0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x1E,0x00}, // 52 '4'
    {0xFE,0xC0,0xC0,0xFC,0x06,0xC6,0x7C,0x00}, // 53 '5'
    {0x38,0x60,0xC0,0xFC,0xC6,0xC6,0x7C,0x00}, // 54 '6'
    {0xFE,0xC6,0x0C,0x18,0x30,0x30,0x30,0x00}, // 55 '7'
    {0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0x7C,0x00}, // 56 '8'
    {0x7C,0xC6,0xC6,0x7E,0x06,0x0C,0x78,0x00}, // 57 '9'
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // 58 ':'
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30}, // 59 ';'
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // 60 '<'
    {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // 61 '='
    {0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00}, // 62 '>'
    {0x7C,0xC6,0x0C,0x18,0x18,0x00,0x18,0x00}, // 63 '?'
    {0x7C,0xC6,0xDE,0xDE,0xDE,0xC0,0x78,0x00}, // 64 '@'
    {0x38,0x6C,0xC6,0xC6,0xFE,0xC6,0xC6,0x00}, // 65 'A'
    {0xFC,0x66,0x66,0x7C,0x66,0x66,0xFC,0x00}, // 66 'B'
    {0x3C,0x66,0xC0,0xC0,0xC0,0x66,0x3C,0x00}, // 67 'C'
    {0xF8,0x6C,0x66,0x66,0x66,0x6C,0xF8,0x00}, // 68 'D'
    {0xFE,0x62,0x68,0x78,0x68,0x62,0xFE,0x00}, // 69 'E'
    {0xFE,0x62,0x68,0x78,0x68,0x60,0xF0,0x00}, // 70 'F'
    {0x3C,0x66,0xC0,0xC0,0xCE,0x66,0x3E,0x00}, // 71 'G'
    {0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0x00}, // 72 'H'
    {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 73 'I'
    {0x1E,0x0C,0x0C,0x0C,0xCC,0xCC,0x78,0x00}, // 74 'J'
    {0xE6,0x66,0x6C,0x78,0x6C,0x66,0xE6,0x00}, // 75 'K'
    {0xF0,0x60,0x60,0x60,0x62,0x66,0xFE,0x00}, // 76 'L'
    {0xC6,0xEE,0xFE,0xFE,0xD6,0xC6,0xC6,0x00}, // 77 'M'
    {0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0x00}, // 78 'N'
    {0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 79 'O'
    {0xFC,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00}, // 80 'P'
    {0x7C,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x06}, // 81 'Q'
    {0xFC,0x66,0x66,0x7C,0x6C,0x66,0xE6,0x00}, // 82 'R'
    {0x7C,0xC6,0xE0,0x78,0x0E,0xC6,0x7C,0x00}, // 83 'S'
    {0x7E,0x7E,0x5A,0x18,0x18,0x18,0x3C,0x00}, // 84 'T'
    {0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 85 'U'
    {0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x00}, // 86 'V'
    {0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00}, // 87 'W'
    {0xC6,0xC6,0x6C,0x38,0x6C,0xC6,0xC6,0x00}, // 88 'X'
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x3C,0x00}, // 89 'Y'
    {0xFE,0xC6,0x8C,0x18,0x32,0x66,0xFE,0x00}, // 90 'Z'
    {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, // 91 '['
    {0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, // 92 '\\'
    {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, // 93 ']'
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00}, // 94 '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // 95 '_'
    {0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, // 96 '`'
    {0x00,0x00,0x78,0x0C,0x7C,0xCC,0x76,0x00}, // 97 'a'
    {0xE0,0x60,0x7C,0x66,0x66,0x66,0xDC,0x00}, // 98 'b'
    {0x00,0x00,0x7C,0xC6,0xC0,0xC6,0x7C,0x00}, // 99 'c'
    {0x1C,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00}, // 100 'd'
    {0x00,0x00,0x7C,0xC6,0xFE,0xC0,0x7C,0x00}, // 101 'e'
    {0x1C,0x36,0x30,0x78,0x30,0x30,0x78,0x00}, // 102 'f'
    {0x00,0x00,0x76,0xCC,0xCC,0x7C,0x0C,0xF8}, // 103 'g'
    {0xE0,0x60,0x6C,0x76,0x66,0x66,0xE6,0x00}, // 104 'h'
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // 105 'i'
    {0x06,0x00,0x0E,0x06,0x06,0x66,0x66,0x3C}, // 106 'j'
    {0xE0,0x60,0x66,0x6C,0x78,0x6C,0xE6,0x00}, // 107 'k'
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 108 'l'
    {0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xD6,0x00}, // 109 'm'
    {0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x00}, // 110 'n'
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7C,0x00}, // 111 'o'
    {0x00,0x00,0xDC,0x66,0x66,0x7C,0x60,0xF0}, // 112 'p'
    {0x00,0x00,0x76,0xCC,0xCC,0x7C,0x0C,0x1E}, // 113 'q'
    {0x00,0x00,0xDC,0x76,0x66,0x60,0xF0,0x00}, // 114 'r'
    {0x00,0x00,0x7E,0xC0,0x7C,0x06,0xFC,0x00}, // 115 's'
    {0x30,0x30,0xFC,0x30,0x30,0x36,0x1C,0x00}, // 116 't'
    {0x00,0x00,0xCC,0xCC,0xCC,0xCC,0x76,0x00}, // 117 'u'
    {0x00,0x00,0xC6,0xC6,0xC6,0x6C,0x38,0x00}, // 118 'v'
    {0x00,0x00,0xC6,0xD6,0xD6,0xFE,0x6C,0x00}, // 119 'w'
    {0x00,0x00,0xC6,0x6C,0x38,0x6C,0xC6,0x00}, // 120 'x'
    {0x00,0x00,0xC6,0xC6,0xC6,0x7E,0x06,0xFC}, // 121 'y'
    {0x00,0x00,0xFE,0x4C,0x18,0x32,0xFE,0x00}, // 122 'z'
    {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, // 123 '{'
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // 124 '|'
    {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, // 125 '}'
    {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00}, // 126 '~'
};

static constexpr int kRows = (kNumChars + kCols - 1) / kCols; // 6 rows for 95 glyphs
static constexpr int kAtlasW = kCols * (kGlyphW + kPad);
static constexpr int kAtlasH = kRows * (kGlyphH + kPad);
// the last grid cell (index 95) is unused by glyphs; we fill it solid white so
// untextured panels can sample a fully-opaque texel.
static constexpr float kSolidU = (kCols*(kGlyphW+kPad) - 4.0f) / float(kAtlasW);
static constexpr float kSolidV = (kRows*(kGlyphH+kPad) - 4.0f) / float(kAtlasH);

} // namespace detail

// ---------------------------------------------------------------------------
// Atlas builder
// ---------------------------------------------------------------------------

inline Atlas fontAtlas() {
    using namespace detail;
    Atlas a;
    a.w = kAtlasW;
    a.h = kAtlasH;
    a.pixels.assign(static_cast<size_t>(a.w) * a.h, 0);

    for (int g = 0; g < kNumChars; ++g) {
        int cellCol = g % kCols;
        int cellRow = g / kCols;
        int ox = cellCol * (kGlyphW + kPad);
        int oy = cellRow * (kGlyphH + kPad);
        for (int row = 0; row < kGlyphH; ++row) {
            uint8_t bits = kFont[g][row];
            for (int col = 0; col < kGlyphW; ++col) {
                // bit 7 (MSB) is the leftmost pixel (standard 8x8 font convention).
                if (bits & (0x80u >> col)) {
                    int px = ox + col;
                    int py = oy + row;
                    a.pixels[static_cast<size_t>(py) * a.w + px] = 255;
                }
            }
        }
    }
    // solid white block in the unused last cell (for untextured panels)
    for (int py = kAtlasH - 8; py < kAtlasH; ++py)
        for (int px = kAtlasW - 8; px < kAtlasW; ++px)
            a.pixels[static_cast<size_t>(py) * a.w + px] = 255;
    return a;
}

// ---------------------------------------------------------------------------
// Geometry generation
// ---------------------------------------------------------------------------

// Advance per character cell, in pixels, given a scale (scale = px per glyph cell).
inline float textWidth(const std::string& text, float scale) {
    float maxW = 0.0f, w = 0.0f;
    for (char c : text) {
        if (c == '\n') { if (w > maxW) maxW = w; w = 0.0f; continue; }
        w += scale;
    }
    if (w > maxW) maxW = w;
    return maxW;
}

// Append textured quads for `text` at pixel (px,py) (top-left of first glyph
// cell). `scale` is the size in pixels of one glyph cell. Six vertices per glyph
// (two triangles), suitable for a non-indexed triangle-list draw.
// Append a solid (untextured) filled rectangle — used for HUD panels/bars.
inline void addRect(std::vector<Vtx>& out, float x, float y, float w, float h,
                    float r, float g, float b, float a) {
    using namespace detail;
    float u = kSolidU, v = kSolidV;
    Vtx tl{ x,   y,   u, v, r, g, b, a };
    Vtx bl{ x,   y+h, u, v, r, g, b, a };
    Vtx br{ x+w, y+h, u, v, r, g, b, a };
    Vtx tr{ x+w, y,   u, v, r, g, b, a };
    out.push_back(tl); out.push_back(bl); out.push_back(br);
    out.push_back(tl); out.push_back(br); out.push_back(tr);
}

inline void addText(std::vector<Vtx>& out, const std::string& text,
                    float px, float py, float scale,
                    float r, float g, float b, float a = 1.0f) {
    using namespace detail;

    const float du = static_cast<float>(kGlyphW) / static_cast<float>(kAtlasW);
    const float dv = static_cast<float>(kGlyphH) / static_cast<float>(kAtlasH);

    float x = px;
    float y = py;

    for (char ch : text) {
        unsigned char c = static_cast<unsigned char>(ch);
        if (c == '\n') {
            x = px;
            y += scale;
            continue;
        }
        if (c >= kFirstChar && c <= kLastChar && c != ' ') {
            int g_ = static_cast<int>(c) - kFirstChar;
            int cellCol = g_ % kCols;
            int cellRow = g_ / kCols;

            float u0 = static_cast<float>(cellCol * (kGlyphW + kPad)) / static_cast<float>(kAtlasW);
            float v0 = static_cast<float>(cellRow * (kGlyphH + kPad)) / static_cast<float>(kAtlasH);
            float u1 = u0 + du;
            float v1 = v0 + dv;

            float x0 = x;
            float y0 = y;
            float x1 = x + scale;
            float y1 = y + scale;

            // Two triangles: (TL,BL,BR) and (TL,BR,TR)
            Vtx tl{ x0, y0, u0, v0, r, g, b, a };
            Vtx bl{ x0, y1, u0, v1, r, g, b, a };
            Vtx br{ x1, y1, u1, v1, r, g, b, a };
            Vtx tr{ x1, y0, u1, v0, r, g, b, a };

            out.push_back(tl);
            out.push_back(bl);
            out.push_back(br);
            out.push_back(tl);
            out.push_back(br);
            out.push_back(tr);
        }
        x += scale; // advance one cell (spaces and unknowns advance, render blank)
    }
}

// ---------------------------------------------------------------------------
// Game HUD — feature parity with the raylib base game (src/main.cpp ~2873-2960)
// ---------------------------------------------------------------------------
//
// Self-contained replica of the base game's on-screen HUD. Emits panels/bars
// (via addRect) and shadowed text (two passes via addText) into `out`, in
// pixel coordinates with a top-left origin. Uses ONLY the existing primitives
// (addRect, addText, textWidth) defined above.
//
// Colour mapping: base game colours are 0..255; divide by 255 here. Panel fill
// uses the base default hudPanel fill {18,22,34,168} -> alpha 168/255 ~= 0.659.
// Shadowed text mirrors textSh(): a dark pass {20,20,30} offset by +2,+2, then
// the main-colour pass on top. The 8x8 cell `scale` is taken 1:1 from the base
// raylib font `size` (16/26/44...), matching the spec.
//
struct GameHud {
    float screenW, screenH;
    bool  riding;          // ride camera vs free-fly
    float speedKmh;
    int   altitude;        // metres above ground
    float gForce;
    const char* element;   // friendly name, "" if none
    bool  elementSpecial;  // inversion -> accent colour
    int   score;
    float boost;           // 0..100
};

namespace detail {

// Draw a base-game-style panel: dark rounded fill (approximated as a sharp
// rect, since no rounding primitive exists) plus a faint top highlight strip,
// mirroring hudPanel(). Colours are 0..255 with explicit alpha.
inline void gameHudPanel(std::vector<Vtx>& out, float x, float y, float w, float h,
                         float fr = 18, float fg = 22, float fb = 34, float fa = 168) {
    addRect(out, x, y, w, h, fr/255.f, fg/255.f, fb/255.f, fa/255.f);
    // top highlight strip (hudPanel: Color{220,232,255,36} at x+5,y+3,w-10,h=2)
    addRect(out, x + 5, y + 3, w - 10, 2, 220/255.f, 232/255.f, 255/255.f, 36/255.f);
}

// Shadowed text, mirroring textSh(): dark drop shadow then main colour.
// Colours are 0..255; `a` is 0..255 alpha for the main pass.
inline void gameHudTextSh(std::vector<Vtx>& out, const std::string& text,
                          float x, float y, float size,
                          float cr, float cg, float cb, float ca = 255) {
    addText(out, text, x + 2, y + 2, size, 20/255.f, 20/255.f, 30/255.f, 200/255.f);
    addText(out, text, x, y, size, cr/255.f, cg/255.f, cb/255.f, ca/255.f);
}

// Format an integer with leading zeros to `digits` width (mirrors "%06d").
inline std::string zeroPad(int value, int digits) {
    if (value < 0) value = 0;
    std::string s = std::to_string(value);
    if ((int)s.size() < digits) s = std::string(digits - s.size(), '0') + s;
    return s;
}

} // namespace detail

// Build the full game HUD into `out`. Replicates the base game's elements:
//   * top-left SCORE panel (always)
//   * top-right SPEED card (riding only; FREE-FLY hint when not riding)
//   * element panel below the speed card (riding only, when an element is named)
//   * boost meter bottom-left (riding only)
inline void buildGameHud(std::vector<Vtx>& out, const GameHud& s) {
    using namespace detail;
    const float sw = s.screenW;
    const float shh = s.screenH;

    // --- Top-left SCORE panel (always shown) ---------------------------------
    {
        std::string sc = zeroPad(s.score, 6);
        float vw = textWidth(sc, 26.0f);                  // MeasureText(sc, 26)
        gameHudPanel(out, 18, 14, 78 + vw, 40);           // hudPanel(18,14,78+vw,40)
        gameHudTextSh(out, "SCORE", 32, 22, 16, 150, 168, 200, 235);
        gameHudTextSh(out, sc, 92, 19, 26, 255, 255, 255, 255); // RAYWHITE
    }

    if (s.riding) {
        // --- Top-right SPEED card --------------------------------------------
        {
            int kmh = (int)s.speedKmh;
            std::string num = std::to_string(kmh);
            float nw = textWidth(num, 44.0f);             // MeasureText(num, 44)
            float cardW = nw + 92.0f;
            float cardX = sw - cardW - 18.0f;
            gameHudPanel(out, cardX, 14, cardW, 62);
            // speed colour: >250 red, >150 green, else white
            float sr = 255, sg = 255, sb = 255;
            if (kmh > 250)      { sr = 255; sg = 120; sb = 90;  }
            else if (kmh > 150) { sr = 120; sg = 230; sb = 170; }
            gameHudTextSh(out, num, cardX + 18, 18, 44, sr, sg, sb, 255);
            gameHudTextSh(out, "KM/H", cardX + 26 + nw, 26, 18, 168, 184, 214, 235);
            std::string alt = "ALT " + std::to_string(s.altitude) + "m";
            float aw = textWidth(alt, 16.0f);             // MeasureText(alt, 16)
            gameHudTextSh(out, alt, (cardX + cardW) - aw - 16, 53, 16, 150, 168, 200, 220);
        }

        // --- Element panel (below speed card, when an element is named) -------
        if (s.element && s.element[0] != '\0') {
            std::string en = s.element;
            float fs = 18.0f;
            float tw = textWidth(en, fs);                 // MeasureText(en, 18)
            float pw = tw + 28.0f;
            float px = sw - pw - 18.0f;
            float py = 84.0f;
            // accent: special -> {255,200,110}, else {150,184,230}
            float ar, ag, ab;
            if (s.elementSpecial) { ar = 255; ag = 200; ab = 110; }
            else                  { ar = 150; ag = 184; ab = 230; }
            gameHudPanel(out, px, py, pw, 30, 18, 22, 34, 168);
            // small accent bar: DrawRectangleRounded({px+8,py+9,4,12}) -> sharp rect
            addRect(out, px + 8, py + 9, 4, 12, ar/255.f, ag/255.f, ab/255.f, 1.0f);
            // text colour: special -> accent, else {214,224,240}
            if (s.elementSpecial)
                gameHudTextSh(out, en, px + 18, py + 7, fs, ar, ag, ab, 255);
            else
                gameHudTextSh(out, en, px + 18, py + 7, fs, 214, 224, 240, 235);
        }

        // --- Boost meter (bottom-left) ---------------------------------------
        {
            float bx = 20, by = shh - 44, bw = 228, bh = 22;
            gameHudTextSh(out, "BOOST", bx, by - 22, 16, 150, 168, 200, 235);
            // bar background {14,18,28,190}
            addRect(out, bx, by, bw, bh, 14/255.f, 18/255.f, 28/255.f, 190/255.f);
            float fillW = (bw - 6) * s.boost / 100.0f;
            if (fillW > 4) {
                float fr, fg, fb;
                if (s.boost > 60)      { fr = 120; fg = 230; fb = 170; }
                else if (s.boost > 30) { fr = 255; fg = 180; fb = 70;  }
                else                   { fr = 235; fg = 90;  fb = 70;  }
                addRect(out, bx + 3, by + 3, fillW, bh - 6, fr/255.f, fg/255.f, fb/255.f, 1.0f);
            }
        }
    } else {
        // --- Free-fly mode: score panel above, plus a hint line --------------
        gameHudTextSh(out, "FREE-FLY  PRESS C TO RIDE", 18, 64, 16, 235, 235, 235, 200);
    }
}

} // namespace hud
