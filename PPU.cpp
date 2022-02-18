#include "PPU.h"

PPU::PPU(Memory& memory) : memory(memory) {
    window = SDL_CreateWindow("GBEMU", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            160*display_scale, 144*display_scale, SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
            SDL_TEXTUREACCESS_STREAMING, 160, 144);
    render_screen();
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
                if (current_line == 144) {
                    memory.write(0xFF44, current_line);
                    current_state = VBLANK;
                    memory.set_interrupt(INT_VBLANK, true);
                    memory.set_interrupt(INT_LCDSTAT, true);
                    memory.write(0xFF41, memory.read(0xFF41) | VBLANK_SRC);
                    set_mode_flag();
                    render_screen();
                } else {
                    memory.write(0xFF44, current_line);
                    current_state = OAM_SCAN;
                    memory.set_interrupt(INT_LCDSTAT, true);
                    memory.write(0xFF41, memory.read(0xFF41) | OAM_SRC);
                    set_mode_flag();
                }
            }
        } else if (current_state == VBLANK) {
            vblank_counter++;
            if ((vblank_counter % 456) == 0) {
                current_line++;
                memory.write(0xFF44, current_line);
            }
            if (vblank_counter == 4560) {
                current_line = 0;
                memory.write(0xFF44, 0);
                vblank_counter = 0;
                current_state = OAM_SCAN;
                memory.set_interrupt(INT_LCDSTAT, true);
                memory.write(0xFF41, memory.read(0xFF41) | OAM_SRC);
                set_mode_flag();
            }
        }
        if (memory.read(0xFF45) == memory.read(0xFF44)) {
            memory.write(0xFF41, memory.read(0xFF41) | 0x40);
            memory.set_interrupt(INT_LCDSTAT, true);
        }
    }
}

void PPU::run_line() {
    fetch_scanline();
    fetch_scanline_sprites();
}

void PPU::fetch_scanline() {
    unsigned tilemap_start;
    if (memory.read(0xFF40) & 0x8) {
        tilemap_start = 0x9C00;
    } else {
        tilemap_start = 0x9800;
    }

    unsigned current_scanline = memory.read(0xFF44) + memory.read(0xFF42);

    uint16_t tilemap_line_start = ((current_scanline / 8) * 32) % 1024;
    uint16_t tilemap_line_end = tilemap_line_start + 32;

    unsigned tile_line = current_scanline % 8;

    unsigned skip_pixels = memory.read(0xFF43);
    uint8_t palette = memory.read(0xFF47);
    unsigned drawn_pixels = 0;
    // Loop though each tile index in the line in the tilemap corresponding to the
    // tiles the current scanline falls in
    for(int i = 0; i < 32 && drawn_pixels < 160; i++) {
        uint8_t tile_index = memory.read(tilemap_start + tilemap_line_start + i);

        unsigned tile_line_address;
        if (memory.read(0xFF40) & 0x10) {
            tile_line_address = 0x8000 + (tile_index * 16) + (tile_line * 2);
        } else {
            tile_line_address = 0x9000 + ((int8_t)tile_index * 16) + (tile_line * 2);
        }

        uint8_t tiledata_low = memory.read(tile_line_address);
        uint8_t tiledata_high = memory.read(tile_line_address + 1);

        for (int i = 7; i >= 0 && drawn_pixels < 160; i--) {
            if (skip_pixels) {
                skip_pixels--;
            } else {
                uint8_t mask = 1 << i;
                unsigned palette_index = (i != 0 ? ((tiledata_high & mask) >> (i-1)) 
                    : (tiledata_high & mask) << 1) 
                    | ((tiledata_low & mask) >> i);
                unsigned colour_index = bg_palette(palette_index, palette);
                draw_buffer[current_line*160 + drawn_pixels] = colours[colour_index];
                drawn_pixels++;
            }
        }
    }
}

void PPU::fetch_scanline_sprites() {
    if (!(memory.read(0xFF40) & 0x2))
        return;
    unsigned num_sprites = 0;
    unsigned sprites_indexes[10];
    for (unsigned i = 0; i < 0xA0 && num_sprites < 10; i += 4) {
        uint8_t y_pos = memory.read(0xFE00 + i);
        if (y_pos > 8 && (y_pos) / 8 == (current_line + 16) / 8)
            sprites_indexes[num_sprites++] = i;
    }
    if (num_sprites == 0)
        return;
    for (int i = num_sprites - 1; i >= 0; i--) {
        unsigned tile_line = (current_line - (memory.read(0xFE00 + sprites_indexes[i]))) / 8;
        unsigned tile_index = memory.read(0xFE00 + sprites_indexes[i] + 2);

        unsigned tile_line_address = 0x8000 + (tile_index * 16) + (tile_line * 2);
        
        uint8_t tiledata_low = memory.read(tile_line_address);
        uint8_t tiledata_high = memory.read(tile_line_address + 1);

        unsigned x_pos = memory.read(0xFE00 + sprites_indexes[i] + 1);
        uint8_t palette;
        if (memory.read(0xFE00 + sprites_indexes[i] + 3) & 0x10)
            palette = memory.read(0xFF48);
        else
            palette = memory.read(0xFF49);

        for (int j = 7; j >= 0 && x_pos < 168; j--) {
            if (x_pos >= 8) {
                uint8_t mask = 1 << j;
                unsigned palette_index = (j != 0 ? ((tiledata_high & mask) >> (j-1)) 
                    : (tiledata_high & mask) << 1) 
                    | ((tiledata_low & mask) >> j);
                unsigned colour_index = bg_palette(palette_index, palette);
                if (colour_index != 0)
                    draw_buffer[current_line*160 + (x_pos - 8)] = colours[colour_index];
            }
            x_pos++;
        }
    }
}

void PPU::render_screen() {
    SDL_UpdateTexture(texture, nullptr, draw_buffer, 160*4);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
}

void PPU::set_mode_flag() {
    switch (current_state) {
        case OAM_SCAN:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xFC) | 2);
            break;
        case DRAWING:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xFC) | 3);
            break;
        case HBLANK:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xFC) | 0);
            break;
        case VBLANK:
            memory.write(0xFF41, (memory.read(0xFF41) & 0xFC) | 1);
            break;
    }
}

unsigned PPU::bg_palette(unsigned index, uint8_t palette) {
    uint8_t mask = 0x3 << (index * 2);
    return (palette & mask) >> (index * 2);
}
