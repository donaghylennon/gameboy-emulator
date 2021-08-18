#include "Memory.h"

void Memory::write(unsigned address, uint8_t data) {
    if (address == 0xFF00)
        joypad_reg = (joypad_reg & 0x0F) & (data & 0xF0);
    if (address >= 0xFF01 && address < 0xFF03)
        serial_regs[address - 0xFF01] = data;
    if (address == 0xFF04)
        timer_regs[0] = 0;
    if (address >= 0xFF05 && address < 0xFF08)
        timer_regs[address - 0xFF04] = data;
}

uint8_t Memory::read(unsigned address) {
    if (address < 0x8000)
        return rom[address];
    if (address == 0xFF00)
        return joypad_reg;
    if (address >= 0xFF01 && address < 0xFF03)
        return serial_regs[address - 0xFF01];
    if (address >= 0xFF04 && address < 0xFF08)
        ;
}

uint8_t& Memory::operator[](unsigned index) {
    if (index < 0x7fff)
        return rom[index];
    if (index == 0xFF00)
        return joypad_reg;
    if (index >= 0xFF01 && index < 0xFF03)
        return serial_regs[index - 0xFF01];
    if (index >= 0xFF04 && index < 0xFF08)
        return timer_regs[index - 0xFF04];
}

void Memory::increment_timer() {
    // reset to value in mod reg on overflow
    //if ((timer_regs[3] & 0x80) && timer_regs[1]++ == 0) 
    //    timer_regs[1] = timer_regs[2];
}

void Memory::increment_divider() {
    timer_regs[0]++;
}

void Memory::load_rom(std::string rom_path) {
    auto rom_file = std::ifstream(rom_path);
    rom_file.read(reinterpret_cast<char*>(rom), 0x8000);
}
