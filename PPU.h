#pragma once

#include "Memory.h"

#include <SDL2/SDL.h>

#include <queue>

class PPU {
public:
    PPU(Memory& memory);
    ~PPU();

    void run_line();
private:
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    unsigned display_scale = 3;
    std::queue<int> background_fifo;
    std::queue<int> object_fifo;
    uint32_t colours[4] = { 0x0, 0x55555555, 0xBBBBBBBB, 0xFFFFFFFF };
};
