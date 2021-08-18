#pragma once

#include <cstdint>
#include <string>
#include <fstream>

class Memory {
public:
    void write(unsigned address, uint8_t data);
    uint8_t read(unsigned address);
    uint8_t& operator[](unsigned index);
    void increment_timer();
    void increment_divider();
    void load_rom(std::string rom_path);
private:
    uint8_t rom[0x8000];
    uint8_t vram[0x2000];
    uint8_t external_ram[0x2000];
    uint8_t wram[0x2000];
    uint8_t oam[0xA0];
    uint8_t joypad_reg;
    uint8_t serial_regs[2];
    uint8_t timer_regs[4];

    // Use with operator[] to disallow
    // writing but still give reference?
    uint8_t junk;
};
