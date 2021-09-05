#pragma once

#include "Memory.h"

#include <SDL2/SDL.h>

#include <queue>

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
private:
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    Memory& memory;
    unsigned display_scale = 3;
    std::queue<uint32_t> background_fifo;
    std::queue<int> object_fifo;
    uint32_t colours[4] = { 0x0, 0x55555555, 0xBBBBBBBB, 0xFFFFFFFF };

    PpuState current_state = OAM_SCAN;
    unsigned oam_scan_counter = 80;
    unsigned drawing_counter = 43;
    unsigned hblank_counter = 51;
    unsigned vblank_counter = 4560;
    unsigned current_line = 0;
    unsigned fetcher_x = 0;
};
