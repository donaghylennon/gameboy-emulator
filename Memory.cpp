#include "Memory.h"

void Memory::write(unsigned address, uint8_t data) {
    if (address < 0x8000) {
        //rom[address] = data;
        std::cout << "Error: Writing to rom" << std::endl;
        std::cout << "Address: ";
        printf("%x\n", address);
        std::cout << "Data: ";
        printf("%x\n", data);
    } else if (address < 0xA000)
        vram[address - 0x8000] = data;
    else if (address < 0xC000)
        external_ram[address - 0xA000] = data;
    else if (address < 0xE000)
        wram[address - 0xC000] = data;
    else if (address < 0xFE00)
        wram[address - 0xE000] = data;
    else if (address < 0xFEA0)
        oam[address - 0xFE00] = data;
    else if (address < 0xFF00)
        ;
    else if (address == 0xFF00)
        joypad_reg = (joypad_reg & 0x0F) | (data & 0xF0);
    else if (address >= 0xFF01 && address < 0xFF03) {
        serial_regs[address - 0xFF01] = data;
        if ((serial_regs[1] & F_TRANSFER_ST_MASK) && address == 0xFF02)
            std::cout << (char)serial_regs[0];
    }
    else if (address == 0xFF04)
        timer_regs[0] = 0;
    else if (address >= 0xFF05 && address < 0xFF08)
        timer_regs[address - 0xFF04] = data;
    else if (address == 0xFF0F)
        interrupt_flag = data;
    else if (address == 0xFFFF)
        interrupt_enable = data;
}

uint8_t Memory::read(unsigned address) {
    if (address < 0x8000)
        return rom[address];
    else if (address < 0xA000)
        return vram[address - 0x8000];
    else if (address < 0xC000)
        return external_ram[address - 0xA000];
    else if (address < 0xE000)
        return wram[address - 0xC000];
    else if (address < 0xFE00)
        return wram[address - 0xE000];
    else if (address < 0xFEA0)
        return oam[address - 0xFE00];
    else if (address < 0xFF00)
        return 0xFF;
    else if (address == 0xFF00)
        return joypad_reg;
    else if (address >= 0xFF01 && address < 0xFF03)
        return serial_regs[address - 0xFF01];
    else if (address >= 0xFF04 && address < 0xFF08)
        ;
    else if (address == 0xFF0F)
        return interrupt_flag;
    else if (address < 0xFF27)
        return sound_regs[address - 0xFF10];
    else if (address < 0xFF40)
        return waveform_ram[address - 0xFF30];
    else if (address == 0xFF44)
        return 0x90;
    else if (address < 0xFF4C)
        return lcd_regs[address - 0xFF40];
    else if (address == 0xFFFF)
        return interrupt_enable;
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

void Memory::set_interrupt(unsigned type, bool value) {
    if (value)
        write(0xFF0F, read(0xFF0F) | type);
    else
        write(0xFF0F, read(0xFF0F) & ~type);
}
