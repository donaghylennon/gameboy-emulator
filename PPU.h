#pragma once

#include "Memory.h"

#include <SDL2/SDL.h>

#include <queue>

#define HBLANK_SRC 0x8
#define VBLANK_SRC 0x10
#define OAM_SRC 0x20

enum PpuState {
    OAM_SCAN, DRAWING, HBLANK, VBLANK
};

class PPU {
public:
    PPU(Memory& memory);
    ~PPU();

    void run_cycle();

    void run_line();
    void fetch_tile_row();
    void fetch_scanline();

    void draw_line();
private:
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    uint32_t draw_buffer[160*144] = {};
    Memory& memory;
    unsigned display_scale = 3;
    std::queue<uint32_t> background_fifo;
    std::queue<int> object_fifo;
    uint32_t colours[4] = { 0xFFFFFFFF, 0xBBBBBBFF, 0x555555FF, 0x000000FF };

    PpuState current_state = OAM_SCAN;
    unsigned oam_scan_counter = 0;
    unsigned drawing_counter = 0;
    unsigned hblank_counter = 0;
    unsigned vblank_counter = 0;
    unsigned current_line = 0;
    unsigned fetcher_x = 0;

    void set_mode_flag();

    unsigned bg_palette(unsigned index);
};
