#pragma once

#include <cstdint>
#include <string>
#include <fstream>
#include <iostream>

#define F_TRANSFER_ST_MASK 0x80

#define INT_VBLANK 0x1
#define INT_LCDSTAT 0x2
#define INT_TIMER 0x4
#define INT_SERIAL 0x8
#define INT_JOYPAD 0x10

class Memory {
public:
    void write(unsigned address, uint8_t data);
    uint8_t read(unsigned address);
    uint8_t& operator[](unsigned index);
    void increment_timer();
    void increment_divider();
    void load_rom(std::string rom_path);
    void load_boot_rom(std::string boot_rom_path);
    void set_interrupt(unsigned type, bool value);
private:
    uint8_t boot_rom[0x100];
    uint8_t rom[0x8000];
    uint8_t vram[0x2000];
    uint8_t external_ram[0x2000];
    uint8_t wram[0x2000];
    uint8_t oam[0xA0];
    uint8_t joypad_reg;
    uint8_t serial_regs[2];
    uint8_t timer_regs[4];
    uint8_t sound_regs[0x17];
    uint8_t waveform_ram[0x10];
    uint8_t lcd_regs[0xC];
    uint8_t boot_rom_reg;
    uint8_t interrupt_flag;
    uint8_t interrupt_enable;
    uint8_t high_ram[0x7F];

    uint8_t junk;
    uint8_t timer_copy;
    
    bool boot_rom_enabled = true;
};
