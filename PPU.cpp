#include "PPU.h"

PPU::PPU(Memory& memory) {
    window = SDL_CreateWindow("GBEMU", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            160*display_scale, 144*display_scale, SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
            SDL_TEXTUREACCESS_STREAMING, 160*display_scale, 144*display_scale);
}

PPU::~PPU() {
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}

void PPU::run_line() {
}
