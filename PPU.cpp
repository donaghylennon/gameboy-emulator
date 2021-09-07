#include "PPU.h"

PPU::PPU(Memory& memory) : memory(memory) {
    window = SDL_CreateWindow("GBEMU", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            160*display_scale, 144*display_scale, SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
            SDL_TEXTUREACCESS_STREAMING, 160, 144);
}

PPU::~PPU() {
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}

void PPU::run_cycle() {
    if (!(memory.read(0xFF40) & 0x80))
        return;
    for (int i = 0; i < 4; i++) {
        if (current_state == OAM_SCAN) {
            oam_scan_counter++;
            if (oam_scan_counter == 80) {
                oam_scan_counter = 0;
                current_state = DRAWING;
                set_mode_flag();
            }
        } else if (current_state == DRAWING) {
            if (drawing_counter == 0)
                run_line();
            drawing_counter++;
            if (drawing_counter == 43) {
                drawing_counter = 0;
                current_state = HBLANK;
                memory.set_interrupt(INT_LCDSTAT, true);
                memory.write(0xFF41, memory.read(0xFF41) | HBLANK_SRC);
                set_mode_flag();
            }
        } else if (current_state == HBLANK) {
            hblank_counter++;
            if (hblank_counter == 51) {
                hblank_counter = 0;
                current_line++;
                if (current_line == 143) {
                    current_state = VBLANK;
                    memory.set_interrupt(INT_VBLANK, true);
                    memory.set_interrupt(INT_LCDSTAT, true);
                    memory.write(0xFF41, memory.read(0xFF41) | VBLANK_SRC);
                    set_mode_flag();
                    current_line = 0;
                } else {
                    current_state = OAM_SCAN;
                    memory.set_interrupt(INT_LCDSTAT, true);
                    memory.write(0xFF41, memory.read(0xFF41) | OAM_SRC);
                    set_mode_flag();
                }
            }
        } else if (current_state == VBLANK) {
            vblank_counter++;
            if ((vblank_counter % 456) == 0) {
                memory.write(0xFF44, memory.read(0xFF44)+1);
            }
            if (vblank_counter == 4560) {
                memory.write(0xFF44, 0);
                vblank_counter = 0;
                current_state = OAM_SCAN;
                memory.set_interrupt(INT_LCDSTAT, true);
                memory.write(0xFF41, memory.read(0xFF41) | OAM_SRC);
                set_mode_flag();
            }
        }
    }
}

void PPU::run_line() {
    for (int i = 0; i < 20; i++) {
        fetch_tile_row();
    }
    draw_line();
    memory.write(0xFF44, memory.read(0xFF44)+1);
}

void PPU::fetch_tile_row() {
    unsigned tilemap_start;
    if (memory.read(0xFF40) & 0x8) {
        tilemap_start = 0x9C00;
    } else {
        tilemap_start = 0x9800;
    }
    unsigned tilemap_x = ((memory.read(0xFF43) / 8) + fetcher_x) & 0x1F;
    unsigned tilemap_y = (current_line + memory.read(0xFF42)) & 0xFF;

    unsigned tilemap_index = (tilemap_y/8) * 32 + tilemap_x;
    uint8_t tile_index = memory.read(tilemap_start + tilemap_index);

    uint8_t tiledata_low, tiledata_high;
    if (memory.read(0xFF40) & 0x10) {
        tiledata_low = memory.read(0x8000 + tile_index);
        tiledata_high = memory.read(0x8000 + tile_index + 1);
    } else {
        tiledata_low = memory.read(0x9000 + (int8_t)tile_index);
        tiledata_high = memory.read(0x9000 + (int8_t)(tile_index + 1));
    }

    for (int i = 0; i < 8; i++) {
        uint8_t mask = 1 << i;
        unsigned colour_index = ((tiledata_high & mask) >> (i-1)) & ((tiledata_low & mask) >> i);
        background_fifo.push(colours[colour_index]);
    }

    fetcher_x++;
}

void PPU::draw_line() {
    for (int i = 0; i < 160; i++) {
        draw_buffer[current_line*160 + i] = background_fifo.front();
        background_fifo.pop();
    }

    SDL_UpdateTexture(texture, nullptr, draw_buffer, 160*4);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
}

void PPU::set_mode_flag() {
    switch (current_state) {
        case OAM_SCAN:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xF8) | 2);
            break;
        case DRAWING:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xF8) | 3);
            break;
        case HBLANK:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xF8) | 0);
            break;
        case VBLANK:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xF8) | 1);
            break;
    }
}
