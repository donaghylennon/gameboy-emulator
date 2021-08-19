#include "CPU.h"

#include <chrono>

CPU::CPU(std::string rom_path) {
    memory.load_rom(rom_path);
}

void CPU::run() {
    int i = 0;
    for (; i < (0xbf-0x80+1); i++) {
        rom[0x100+i] = 0x80+i;
    }
    //for (int j = 0; j < 4; j++) {
    //        rom[0x100 + i++] = 0x10*j + 0x6;
    //        rom[0x100 + i++] = 0x0;
    //}

    pc = 0x100;

    auto prev_cycle = std::chrono::high_resolution_clock::now();
    unsigned timer_counter = 0;
    bool running = true;

    while (running) {
        auto current_time = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float, std::chrono::milliseconds::period>(
            current_time - prev_cycle
        ).count();

        if (dt > CYCLE_TIME) {
            debug_regs_default();
            
            prev_cycle = current_time;

            if (pc >= 0x17f -0x40 + 10)
                break;
            if (wait_cycles) {
                wait_cycles--;
            } else {
                fetch_next_instr();
            }

            if (timer_counter++ == 256) {
                timer_counter = 0;
                memory.increment_divider();
            }

            print_registers();
        }

        if (serial_regs[1] & 0x80) {
            std::cout << serial_regs[0];
            serial_regs[0] = 0;
            serial_regs[1] = serial_regs[1] & 0x7f;
        }
    }
}

uint16_t& CPU::registers16(unsigned index) {
    return reinterpret_cast<uint16_t*>(registers)[index];
}

uint8_t& CPU::get_mem(uint16_t address) {
    if (address >= 0 && address < 0x8) {
        return rom[address];
    } else if (address == 0xFF00) {
        return joypad_reg;
    } else if (address > 0xFF01 && address < 0xFF03) {
        return serial_regs[address - 0xFF01];
    }
    std::cout << "Error: accessing unimplemented address" << std::endl;
    return temp_memory[address];
}

void CPU::debug_regs_default() {
    registers[B] = 0x42;
    registers[C] = 0x43;
    registers[D] = 0x44;
    registers[E] = 0x45;
    registers[H] = 0x48;
    registers[L] = 0x4c;
    registers[A] = 0x41;
    registers[F] = 0x46;
}

void CPU::print_registers() {
    std::cout << "Reg B: ";
    printf("%x\n", registers[B]);
    std::cout << "Reg C: ";
    printf("%x\n", registers[C]);
    std::cout << "Reg D: ";
    printf("%x\n", registers[D]);
    std::cout << "Reg E: ";
    printf("%x\n", registers[E]);
    std::cout << "Reg H: ";
    printf("%x\n", registers[H]);
    std::cout << "Reg L: ";
    printf("%x\n", registers[L]);
    std::cout << "Reg A: ";
    printf("%x\n", registers[A]);
    std::cout << "Reg F: ";
    printf("%x\n", registers[F]);
    std::cout << "==========================" << std::endl;
}

void CPU::fetch_next_instr() {
    uint8_t instr = memory.read(pc);
        std::cout << "Instruction: ";
        printf("%x\n", instr);
    switch (instr & 0xF0) {
        case 0x00:
        case 0x10:
        case 0x20:
        case 0x30:
            if ((instr & 0x7) == 0x6)
                load_r_imm();
            if ((instr & 0x0F) == 0x3)
                inc_rr();
            break;
        case 0x40:
        case 0x50:
        case 0x60:
        case 0x70:
            if (instr != 0x76)
                load_r_r();
            else
                ;//halt
            break;
        case 0x80:
        case 0x90:
        case 0xA0:
        case 0xB0:
            alu_r();
        case 0xC0:
        case 0xD0:
        case 0xE0:
        case 0xF0:
            if ((instr & 0x7) == 0x6)
                ;
            break;
    }
    pc++;
}

void CPU::load_r_r() {
    std::cout << "Entering load_r_r" << std::endl;
    uint8_t instr = memory.read(pc);
    if ((instr & 0x7) == 0x6) {
        get_reg_by_index(left_reg_index(instr)) = memory.read(registers16(HL));
    } else if ((instr & 0x38) == 0x30) {
        memory.write(registers16(HL), get_reg_by_index(right_reg_index(instr)));
    } else {
        std::cout << "Reg to reg" << std::endl;
        std::cout << "Instruction: ";
        printf("%x\n", instr);
        std::cout << "Left: " << left_reg_index(instr) << std::endl;
        std::cout << "Right: " << right_reg_index(instr) << std::endl;
        get_reg_by_index(left_reg_index(instr)) = get_reg_by_index(right_reg_index(instr));
    }
}

void CPU::load_r_imm() {
    std::cout << "Entering load_r_imm" << std::endl;
    uint8_t instr = memory.read(pc++);
    uint8_t data = memory.read(pc);
        std::cout << "Instruction: ";
        printf("%x\n", instr);
        std::cout << "Data: ";
        printf("%x\n", data);
    if (instr == 0x36)
        memory.write(registers16(HL), data);
    else
        get_reg_by_index(left_reg_index(instr)) = data;

        std::cout << "Left: " << left_reg_index(instr) << std::endl;
}

void CPU::load_mem_from_reg_r() {
    uint8_t instr = memory.read(pc);
    
    // Use another array/method to map to real reg indexes?
    switch (instr) {
        case 0x02:
            load_mem_r(registers16(BC));
            break;
        case 0x12:
            load_mem_r(registers16(DE));
            break;
        case 0x22:
            load_mem_r(registers16(HL)++);
            break;
        case 0x32:
            load_mem_r(registers16(HL)--);
            break;
        case 0xE0:
            load_mem_r(memory.read(++pc) + 0xFF00);
        case 0xE2:
            load_mem_r(registers[C] + 0xFF00);
            break;
    }
}

void CPU::load_r_mem_from_reg() {
    uint8_t instr = memory.read(pc);

    switch (instr) {
        case 0x0A:
            load_r_mem(registers16(BC));
            break;
        case 0x1A:
            load_r_mem(registers16(DE));
            break;
        case 0x2A:
            load_r_mem(registers16(HL)++);
            break;
        case 0x3A:
            load_r_mem(registers16(HL)--);
            break;
        case 0xF0:
            load_r_mem(memory.read(++pc) + 0xFF00);
            break;
        case 0xF2:
            load_r_mem(registers[C] + 0xFF00);
            break;
    }
}

// ********* change to inline function to get 16 bit and reduce number of functions
void CPU::load_mem_from_imm_r() {
    //uint8_t instr = memory.read(pc);
    uint8_t lsb = memory.read(++pc);
    uint8_t msb = memory.read(++pc);

    load_mem_r((msb << 4) & lsb);
}

inline uint16_t CPU::concat_bytes(uint8_t lsb, uint8_t msb) {
    return (msb << 4) & lsb;
}

void CPU::load_r_mem_from_imm() {
    uint8_t lsb = memory.read(++pc);
    uint8_t msb = memory.read(++pc);

    load_r_mem((msb << 4) & lsb);
}
// ********************************************************************************

void CPU::load_mem_r(uint16_t address) {
    memory.write(address, registers[A]);
}

void CPU::load_r_mem(uint16_t address) {
    registers[A] = memory.read(address);
}

void CPU::load_rr_imm() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0x01:
            registers16(BC) = concat_bytes(memory.read(++pc), memory.read(++pc));
            break;
        case 0x21:
            registers16(DE) = concat_bytes(memory.read(++pc), memory.read(++pc));
            break;
        case 0x31:
            registers16(HL) = concat_bytes(memory.read(++pc), memory.read(++pc));
            break;
        case 0x41:
            sp = concat_bytes(memory.read(++pc), memory.read(++pc));
            break;
    }
}

void CPU::load_mem_from_imm_sp() {
    uint8_t lsb = memory.read(++pc);
    uint8_t msb = memory.read(++pc);
    memory.write(concat_bytes(lsb, msb), sp & 0xFF);
    memory.write(concat_bytes(lsb, msb), sp << 8);
}

void CPU::load_sp_hl() {
    sp = registers16(HL);
}

void CPU::load_hl_sp_plus_simm() {
    uint8_t carry;
    uint8_t half_carry = 0;
    int8_t operand = memory.read(++pc);
    if (((sp & 0xFFF) - (operand & 0xFFF)) & 0x1000)
        half_carry = 1;
    // This might be off by one when op negative, since when operand negative 0xFFFF - 1 = 0
    carry = sp > ((0xFFFF - operand) & 0xFFFF);
    registers16(HL) = sp + (int8_t) memory.read(++pc);
    registers[F] = (half_carry << F_HALF_CARRY_SHIFT) & (carry << F_CARRY_SHIFT);
}

void CPU::alu_r() {
    std::cout << "Entering alu_r" << std::endl;
    uint8_t instr = memory.read(pc);
    switch (instr & 0xF8) {
        case 0x80:
            alu_add(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0x88:
            alu_adc(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0x90:
            alu_sub(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0x98:
            alu_sbc(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0xA0:
            alu_and(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0xA8:
            alu_xor(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0xB0:
            alu_or(get_reg_by_index(right_reg_index(instr)));
            break;
        case 0xB8:
            alu_cp(get_reg_by_index(right_reg_index(instr)));
            break;
    }
}

void CPU::alu_imm() {
    uint8_t instr = memory.read(pc++);
    uint8_t data = memory.read(pc);
    switch (instr) {
        case 0xC6:
            alu_add(data);
        case 0xCE:
            alu_adc(data);
        case 0xD6:
            alu_sub(data);
        case 0xDE:
            alu_sbc(data);
        case 0xE6:
            alu_and(data);
        case 0xEE:
            alu_xor(data);
        case 0xF6:
            alu_or(data);
        case 0xFE:
            alu_cp(data);
    }
}

void CPU::alu_add(uint8_t operand) {
    std::cout << "Entering alu_add" << std::endl;
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;

    carry = registers[A] > 0xFF - operand;
    half_carry = (registers[A] & 0xF) > 0xF - (operand & 0xF);
    registers[A] += operand;
    
    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
    //uint16_t result = registers[A] + operand;
    //if (result == 0)
    //    registers[F] = registers[F] | 0x80;
    //else 
    //    registers[F] = registers[F] & 0x7F;

    //registers[F] = registers[F] & ~0x40;

    //if ((registers[A] & 0x8) && (operand & 0x8))
    //    registers[F] = registers[F] | 0x20;

    //if (result & 0x100)
    //    registers[F] = registers[F] | 0x10;

    //registers[A] = (uint8_t) result;
}

void CPU::alu_adc(uint8_t operand) {
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;
    if (registers[F] & F_CARRY_MASK) {
        carry = registers[A] >= 0xFF - operand;
        half_carry = (registers[A] & 0xF) >= 0xF - (operand & 0xF);
        registers[A] += operand + 1;
    } else {
        carry = registers[A] > 0xFF - operand;
        half_carry = (registers[A] & 0xF) > 0xF - (operand & 0xF);
        registers[A] += operand;
    }
    
    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_sub(uint8_t operand) {
    uint8_t zero;
    uint8_t carry;
    uint8_t half_carry = 0;

    carry = registers[A] < operand;
    //half_carry = (registers[A] & 0xF) > 0xF - (operand & 0xF);
    if (((registers[A] & 0xF) - (operand & 0xF)) & 0x10)
        half_carry = 1;
    registers[A] -= operand;
    
    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_sbc(uint8_t operand) {
    uint8_t zero;
    uint8_t carry;
    uint8_t half_carry = 0;
    uint8_t carry_in = registers[F] & F_CARRY_MASK ? 1 : 0;

    carry = registers[A] < operand + carry_in;
    if (((registers[A] & 0xF) - (operand & 0xF) - carry_in) & 0x10)
        half_carry = 1;
    registers[A] -= (operand + carry_in);
    
    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_and(uint8_t operand) {
    uint8_t zero = 0;
    registers[A] &= operand;

    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT) | F_HALF_CARRY_MASK;
}

void CPU::alu_xor(uint8_t operand) {
    uint8_t zero = 0;
    registers[A] ^= operand;

    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT);
}

void CPU::alu_or(uint8_t operand) {
    uint8_t zero = 0;
    registers[A] |= operand;

    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT);
}

void CPU::alu_cp(uint8_t operand) {
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;

    uint8_t result = registers[A] - operand;
    carry = registers[A] < operand ? 1 : 0;
    half_carry = (registers[A] & 0xF) < (operand & 0xF) ? 1 : 0;
    zero = registers[A] == 0 ? 1 : 0;
    registers[F] = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::inc_rr() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0x03:
            registers16(BC)++;
            break;
        case 0x13:
            registers16(DE)++;
            break;
        case 0x23:
            registers16(HL)++;
            break;
        case 0x33:
            sp++;
            break;
    }
}

void CPU::inc_r() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0x04:
            registers[B]++;
            break;
        case 0x0C:
            registers[C]++;
            break;
        case 0x14:
            registers[D]++;
            break;
        case 0x1C:
            registers[E]++;
            break;
        case 0x24:
            registers[H]++;
            break;
        case 0x2C:
            registers[L]++;
            break;
        case 0x34:
            memory.write(registers16(HL), memory.read(registers16(HL)));
            break;
        case 0x3C:
            registers[A]++;
            break;
    }
}

void CPU::dec_rr() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0x0B:
            registers16(BC)--;
            break;
        case 0x1B:
            registers16(DE)--;
            break;
        case 0x2B:
            registers16(HL)--;
            break;
        case 0x3B:
            sp--;
            break;
    }
}

void CPU::dec_r() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0x05:
            registers[B]--;
            break;
        case 0x0D:
            registers[C]--;
            break;
        case 0x15:
            registers[D]--;
            break;
        case 0x1D:
            registers[E]--;
            break;
        case 0x25:
            registers[H]--;
            break;
        case 0x2D:
            registers[L]--;
            break;
        case 0x35:
            memory.write(registers16(HL), memory.read(registers16(HL)-1));
            break;
        case 0x3D:
            registers[A]--;
            break;
    }
}

void CPU::push() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0xC5:
            memory.write(sp--, registers16(BC));
            break;
        case 0xD5:
            memory.write(sp--, registers16(DE));
            break;
        case 0xE5:
            memory.write(sp--, registers16(HL));
            break;
        case 0xF5:
            memory.write(sp--, registers16(AF));
            break;
    }
}

void CPU::pop() {
    uint8_t instr = memory.read(pc);
    switch (instr) {
        case 0xC1:
            memory.write(++sp, registers16(BC));
            break;
        case 0xD1:
            memory.write(++sp, registers16(DE));
            break;
        case 0xE1:
            memory.write(++sp, registers16(HL));
            break;
        case 0xF1:
            memory.write(++sp, registers16(AF));
            break;
    }
}

uint8_t& CPU::get_reg_by_index(unsigned index) {
    if (index == 7)
        return registers[A];
    else
        return registers[index];
}

uint8_t CPU::read_reg(unsigned index) {
    // Do this using an array matching index to actual reg pos?
    if (index == 7)
        return registers[A];
    else if (index == 6)
        return memory.read(registers16(HL));
    else
        return registers[index];
}

void CPU::write_reg(unsigned index, uint8_t data) {
    if (index == 7)
        registers[A] = data;
    else if (index == 6)
        memory.write(registers16(HL), data);
    else
        registers[index] = data;
}

inline unsigned CPU::left_reg_index(uint8_t instr) {
    return (instr & 0x38) >> 3;
}

inline unsigned CPU::right_reg_index(uint8_t instr) {
    return instr & 0x07;
}

void CPU::increment_timer() {
    if (timer_regs[1]++ == 0) { // overflow
        timer_regs[1] = timer_regs[2];
        // interrupt requested
    }
}
