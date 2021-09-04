#include "CPU.h"

#include <chrono>

CPU::CPU(std::string rom_path) {
    memory.load_rom(rom_path);
}

void CPU::run() {
    int i = 0;
    //for (; i < (0xbf-0x80+1); i++) {
    //    rom[0x100+i] = 0x80+i;
    //}
    //for (int j = 0; j < 4; j++) {
    //        rom[0x100 + i++] = 0x10*j + 0x6;
    //        rom[0x100 + i++] = 0x0;
    //}

    pc = 0x100;

    auto prev_cycle = std::chrono::high_resolution_clock::now();
    unsigned timer_counter = 0;
    bool running = true;

    registers8(A) = 0x01;

    while (running) {
        auto current_time = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float, std::chrono::microseconds::period>(
            current_time - prev_cycle
        ).count();

        if (dt > CYCLE_TIME) {
            prev_cycle = current_time;

            if (ppu_delay == 0) {
                // run ppu
            } else {
                ppu_delay--;
            }

            if (wait_cycles) {
                wait_cycles--;
            } else {
                fetch_next_instr();
            }

            //if (timer_counter++ == 256) {
            //    timer_counter = 0;
            //    memory.increment_divider();
            //}

            if (interrupt_delay) {
                if (!--interrupt_delay)
                    interrupt_enabled = true;
            }
        }

        //if (serial_regs[1] & 0x80) {
        //    std::cout << serial_regs[0];
        //    serial_regs[0] = 0;
        //    serial_regs[1] = serial_regs[1] & 0x7f;
        //}

        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_QUIT:
                    running = false;
                    SDL_Quit();
                    break;
            }
        }
    }
}

uint16_t& CPU::registers16(unsigned index) {
    return reinterpret_cast<uint16_t*>(registers)[index];
}

uint8_t& CPU::registers8(unsigned index) {
    //if (index % 2) {
    //    return reinterpret_cast<uint8_t*>(registers)[index + 1];
    //} else {
    //    return reinterpret_cast<uint8_t*>(registers)[index - 1];
    //}
    return registers[index];
}

void CPU::debug_regs_default() {
    registers8(B) = 0x42;
    registers8(C) = 0x43;
    registers8(D) = 0x44;
    registers8(E) = 0x45;
    registers8(H) = 0x48;
    registers8(L) = 0x4c;
    registers8(A) = 0x41;
    //registers8(F) = 0x46;
}

void CPU::print_registers() {
    std::cout << "Reg B: ";
    printf("%x\n", registers8(B));
    std::cout << "Reg C: ";
    printf("%x\n", registers8(C));
    std::cout << "Reg D: ";
    printf("%x\n", registers8(D));
    std::cout << "Reg E: ";
    printf("%x\n", registers8(E));
    std::cout << "Reg H: ";
    printf("%x\n", registers8(H));
    std::cout << "Reg L: ";
    printf("%x\n", registers8(L));
    std::cout << "Reg A: ";
    printf("%x\n", registers8(A));
    std::cout << "Reg F: ";
    printf("%x\n", registers8(F));
    std::cout << "==========================" << std::endl;
}

void CPU::print_registers16() {
    std::cout << "Reg BC: ";
    printf("%x\n", registers16(BC));
    std::cout << "Reg DE: ";
    printf("%x\n", registers16(DE));
    std::cout << "Reg HL: ";
    printf("%x\n", registers16(HL));
    std::cout << "Reg AF: ";
    printf("%x\n", registers16(AF));
    std::cout << "==========================" << std::endl;
}

void CPU::fetch_next_instr() {
        //std::cout << "PC: ";
        //printf("%x || ", pc);
    instruction = memory.read(pc++);
        //std::cout << "Instruction: ";
        //printf("%x\n", instruction);

    switch (instruction & 0xF8) {
        case 0x00:
        case 0x08:
        case 0x10:
        case 0x18:
        case 0x20:
        case 0x28:
        case 0x30:
        case 0x38:
            switch (instruction & 0x07) {
                case 0x0:
                    switch (instruction & 0xF8) {
                        case 0x00:
                            break;
                        case 0x08:
                            load_mem_from_imm_sp();
                            break;
                        case 0x10:
                            //stop();
                            break;
                        case 0x18:
                            jr_imm();
                            break;
                        case 0x20:
                        case 0x28:
                        case 0x30:
                        case 0x38:
                            jr_c_imm();
                            break;
                    }
                    break;
                case 0x1:
                    switch (instruction & 0x0F) {
                        case 0x1:
                            load_rr_imm();
                            break;
                        case 0x9:
                            add_rr();
                            break;
                    }
                    break;
                case 0x2:
                    switch (instruction & 0x0F) {
                        case 0x2:
                            load_mem_from_reg_r();
                            break;
                        case 0xA:
                            load_r_mem_from_reg();
                            break;
                    }
                    break;
                case 0x3:
                    switch (instruction & 0x0F) {
                        case 0x3:
                            inc_rr();
                            break;
                        case 0xB:
                            dec_rr();
                            break;
                    }
                    break;
                case 0x4:
                    inc_r();
                    //print_registers();
                    //print_registers16();
                    break;
                case 0x5:
                    dec_r();
                    break;
                case 0x6:
                    load_r_imm();
                    break;
                case 0x7:
                    switch (instruction & 0xF8) {
                        case 0x00:
                            rlca();
                            break;
                        case 0x08:
                            rrca();
                            break;
                        case 0x10:
                            rla();
                            break;
                        case 0x18:
                            rra();
                            break;
                        case 0x20:
                            daa();
                            break;
                        case 0x28:
                            cpl();
                            break;
                        case 0x30:
                            scf();
                            break;
                        case 0x38:
                            ccf();
                            break;
                    }
                    break;
            }
            break;
        case 0x40:
        case 0x48:
        case 0x50:
        case 0x58:
        case 0x60:
        case 0x68:
        case 0x70:
        case 0x78:
            if (instruction == 0x76)
                halt();
            else
                load_r_r();
            break;
        case 0x80:
        case 0x88:
        case 0x90:
        case 0x98:
        case 0xA0:
        case 0xA8:
        case 0xB0:
        case 0xB8:
            alu_r();
            break;
        case 0xC0:
        case 0xC8:
        case 0xD0:
        case 0xD8:
        case 0xE0:
        case 0xE8:
        case 0xF0:
        case 0xF8:
            switch (instruction & 0x07) {
                case 0x00:
                    if ((instruction & 0xF8) <= 0xD8)
                        ret_c();
                    else
                        switch (instruction & 0xF8) {
                            case 0xE0:
                                load_mem_from_reg_r();
                                break;
                            case 0xE8:
                                add_sp_imm();
                                break;
                            case 0xF0:
                                load_r_mem_from_reg();
                                break;
                            case 0xF8:
                                load_hl_sp_plus_simm();
                                break;
                        }
                    break;
                case 0x01:
                    if ((instruction & 0x0F) == 0x01)
                        pop();
                    else
                        switch (instruction & 0xF0) {
                            case 0xC0:
                                ret();
                                break;
                            case 0xD0:
                                reti();
                                break;
                            case 0xE0:
                                jp_hl();
                                break;
                            case 0xF0:
                                load_sp_hl();
                                break;
                        }
                    break;
                case 0x02:
                    if ((instruction & 0xF8) <= 0xD8)
                        jp_c_imm();
                    else
                        switch (instruction & 0xF8) {
                            case 0xE0:
                                load_mem_from_reg_r();
                                break;
                            case 0xE8:
                                load_mem_from_imm_r();
                                break;
                            case 0xF0:
                                load_r_mem_from_reg();
                                break;
                            case 0xF8:
                                load_r_mem_from_imm();
                                break;
                        }
                    break;
                case 0x03:
                    switch (instruction & 0xF8) {
                        case 0xC0:
                            jp_imm();
                            break;
                        case 0xC8:
                            // CB instructions
                            // skip for now
                            cb_instruction();
                            //pc++;
                            break;
                        case 0xF0:
                            di();
                            break;
                        case 0xF8:
                            ei();
                            break;
                    }
                    break;
                case 0x04:
                    call_c_imm();
                    break;
                case 0x05:
                    if (instruction == 0xCD)
                        call_imm();
                    else
                        push();
                    break;
                case 0x06:
                    alu_imm();
                    break;
                case 0x07:
                    rst();
                    break;
            }
            break;
    }

    //pc++;
}

void CPU::load_r_r() {
    //std::cout << "Entering load_r_r" << std::endl;
    if ((instruction & 0x7) == 0x6) {
        get_reg_by_index(left_reg_index(instruction)) = memory.read(registers16(HL));
    } else if ((instruction & 0x38) == 0x30) {
        memory.write(registers16(HL), get_reg_by_index(right_reg_index(instruction)));
    } else {
        //std::cout << "Reg to reg" << std::endl;
        //std::cout << "Instruction: ";
        //printf("%x\n", instruction);
        //std::cout << "Left: " << left_reg_index(instruction) << std::endl;
        //std::cout << "Right: " << right_reg_index(instruction) << std::endl;
        get_reg_by_index(left_reg_index(instruction)) = get_reg_by_index(right_reg_index(instruction));
    }
}

void CPU::load_r_imm() {
    //std::cout << "Entering load_r_imm" << std::endl;
    uint8_t data = memory.read(pc++);
        //std::cout << "Instruction: ";
        //printf("%x\n", instruction);
        //std::cout << "Data: ";
        //printf("%x\n", data);
    if (instruction == 0x36)
        memory.write(registers16(HL), data);
    else
        get_reg_by_index(left_reg_index(instruction)) = data;

        //std::cout << "Left: " << left_reg_index(instruction) << std::endl;
}

void CPU::load_mem_from_reg_r() {
    
    // Use another array/method to map to real reg indexes?
    switch (instruction) {
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
            load_mem_r(memory.read(pc++) + 0xFF00);
        case 0xE2:
            load_mem_r(registers8(C) + 0xFF00);
            break;
    }
}

void CPU::load_r_mem_from_reg() {

    switch (instruction) {
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
            load_r_mem(memory.read(pc++) + 0xFF00);
            break;
        case 0xF2:
            load_r_mem(registers8(C) + 0xFF00);
            break;
    }
}

// ********* change to inline function to get 16 bit and reduce number of functions
void CPU::load_mem_from_imm_r() {
    //uint8_t instr = memory.read(pc);
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);

    //load_mem_r((msb << 8) | lsb);
    load_mem_r(concat_bytes(lsb, msb));
}

inline uint16_t CPU::concat_bytes(uint8_t lsb, uint8_t msb) {
    //std::cout << "Concat: ";
    //printf("lsb: %x msb: %x\n", lsb, msb);
    //printf("final: %x\n", (msb << 8) | lsb);
    return (msb << 8) | lsb;
}

void CPU::load_r_mem_from_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);

    //load_r_mem((msb << 8) & lsb);
    load_r_mem(concat_bytes(lsb, msb));
}
// ********************************************************************************

void CPU::load_mem_r(uint16_t address) {
    memory.write(address, registers8(A));
}

void CPU::load_r_mem(uint16_t address) {
    //printf("Loading from address: %x\n", address);
    registers8(A) = memory.read(address);
    //printf("Register A result: %x\n", registers8(A));
}

void CPU::load_rr_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    switch (instruction) {
        case 0x01:
            registers16(BC) = concat_bytes(lsb, msb);
            break;
        case 0x11:
            registers16(DE) = concat_bytes(lsb, msb);
            break;
        case 0x21:
            registers16(HL) = concat_bytes(lsb, msb);
            break;
        case 0x31:
            sp = concat_bytes(lsb, msb);
            break;
    }
}

void CPU::load_mem_from_imm_sp() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    uint16_t address = concat_bytes(lsb, msb);
    memory.write(address, sp & 0xFF);
    memory.write(address+1, sp >> 8);
}

void CPU::load_sp_hl() {
    sp = registers16(HL);
}

void CPU::load_hl_sp_plus_simm() {
    uint8_t carry;
    uint8_t half_carry = 0;
    int8_t operand = memory.read(pc++);
    if (((sp & 0xFFF) - (operand & 0xFFF)) & 0x1000)
        half_carry = 1;
    // This might be off by one when op negative, since when operand negative 0xFFFF - 1 = 0
    carry = sp > ((0xFFFF - operand) & 0xFFFF);
    registers16(HL) = sp + (int8_t) memory.read(pc++);
    registers8(F) = (half_carry << F_HALF_CARRY_SHIFT) & (carry << F_CARRY_SHIFT);
}

void CPU::alu_r() {
    //std::cout << "Entering alu_r" << std::endl;
    switch (instruction & 0xF8) {
        case 0x80:
            alu_add(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0x88:
            alu_adc(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0x90:
            alu_sub(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0x98:
            alu_sbc(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0xA0:
            alu_and(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0xA8:
            alu_xor(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0xB0:
            alu_or(get_reg_by_index(right_reg_index(instruction)));
            break;
        case 0xB8:
            alu_cp(get_reg_by_index(right_reg_index(instruction)));
            break;
    }
}

void CPU::alu_imm() {
    uint8_t data = memory.read(pc++);
    switch (instruction) {
        case 0xC6:
            alu_add(data);
            break;
        case 0xCE:
            alu_adc(data);
            break;
        case 0xD6:
            alu_sub(data);
            break;
        case 0xDE:
            alu_sbc(data);
            break;
        case 0xE6:
            alu_and(data);
            break;
        case 0xEE:
            alu_xor(data);
            break;
        case 0xF6:
            alu_or(data);
            break;
        case 0xFE:
            alu_cp(data);
            break;
    }
}

void CPU::alu_add(uint8_t operand) {
    //std::cout << "Entering alu_add" << std::endl;
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;

    carry = registers8(A) > 0xFF - operand;
    half_carry = (registers8(A) & 0xF) > 0xF - (operand & 0xF);
    registers8(A) += operand;
    
    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
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
    if (registers8(F) & F_CARRY_MASK) {
        carry = registers8(A) >= 0xFF - operand;
        half_carry = (registers8(A) & 0xF) >= 0xF - (operand & 0xF);
        registers8(A) += operand + 1;
    } else {
        carry = registers8(A) > 0xFF - operand;
        half_carry = (registers8(A) & 0xF) > 0xF - (operand & 0xF);
        registers8(A) += operand;
    }
    
    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_sub(uint8_t operand) {
    uint8_t zero;
    uint8_t carry;
    uint8_t half_carry = 0;

    carry = registers8(A) < operand;
    //half_carry = (registers[A] & 0xF) > 0xF - (operand & 0xF);
    if (((registers8(A) & 0xF) - (operand & 0xF)) & 0x10)
        half_carry = 1;
    registers8(A) -= operand;
    
    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_sbc(uint8_t operand) {
    uint8_t zero;
    uint8_t carry;
    uint8_t half_carry = 0;
    uint8_t carry_in = registers8(F) & F_CARRY_MASK ? 1 : 0;

    carry = registers8(A) < operand + carry_in;
    if (((registers8(A) & 0xF) - (operand & 0xF) - carry_in) & 0x10)
        half_carry = 1;
    registers8(A) -= (operand + carry_in);
    
    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_and(uint8_t operand) {
    //std::cout << "Entering and" << std::endl;
    //printf("Reg A: %x\n", registers8(A));
    //printf("Operand: %x\n", operand);
    uint8_t zero = 0;
    registers8(A) &= operand;
    //printf("Reg A result: %x\n", registers8(A));

    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_HALF_CARRY_MASK;
    //printf("Reg F result: %x\n", registers8(F));
}

void CPU::alu_xor(uint8_t operand) {
    uint8_t zero = 0;
    registers8(A) ^= operand;

    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT);
}

void CPU::alu_or(uint8_t operand) {
    //print_registers();
    //print_registers16();
    uint8_t zero = 0;
    registers8(A) |= operand;

    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT);
}

void CPU::alu_cp(uint8_t operand) {
    //std::cout << "Entering cp" << std::endl;
    //printf("Operand: %x\n", operand);
    //print_registers();
    //print_registers16();
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;

    uint8_t result = registers8(A) - operand;
    carry = registers8(A) < operand ? 1 : 0;
    half_carry = (registers8(A) & 0xF) < (operand & 0xF) ? 1 : 0;
    zero = result == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);

    //std::cout << "Leaving cp" << std::endl;
    //print_registers();
    //print_registers16();
}

void CPU::add_rr() {
    uint16_t operand;
    uint8_t carry;
    uint8_t half_carry = 0;
    switch (instruction) {
        case 0x03:
            operand = registers16(BC);
            break;
        case 0x13:
            operand = registers16(DE);
            break;
        case 0x23:
            operand = registers16(HL);
            break;
        case 0x33:
            operand = sp;
            break;
    }
    if (((sp & 0xFFF) - (operand & 0xFFF)) & 0x1000)
        half_carry = 1;
    carry = sp > ((0xFFFF - operand) & 0xFFFF);

    registers16(HL) += registers16(HL);

    registers8(F) = (registers8(F) & F_ZERO_MASK) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::add_sp_imm() {
    uint8_t carry;
    uint8_t half_carry;
    int8_t operand = memory.read(pc++);

    if (((sp & 0xFFF) - (operand & 0xFFF)) & 0x1000)
        half_carry = 1;
    // This might be off by one when op negative, since when operand negative 0xFFFF - 1 = 0
    carry = sp > ((0xFFFF - operand) & 0xFFFF);

    sp += operand;
    registers8(F) = (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::inc_rr() {
    switch (instruction) {
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
    uint8_t result;
    uint8_t half_carry = 0;
    uint8_t zero = 0;
    switch (instruction) {
        case 0x04:
            half_carry = (registers8(B) & 0xF) == 0xF;
            result = registers8(B) + 1;
            registers8(B)++;
            break;
        case 0x0C:
            half_carry = (registers8(C) & 0xF) == 0xF;
            result = registers8(C) + 1;
            registers8(C)++;
            break;
        case 0x14:
            half_carry = (registers8(D) & 0xF) == 0xF;
            result = registers8(D) + 1;
            registers8(D)++;
            break;
        case 0x1C:
            half_carry = (registers8(E) & 0xF) == 0xF;
            result = registers8(E) + 1;
            registers8(E)++;
            break;
        case 0x24:
            half_carry = (registers8(H) & 0xF) == 0xF;
            result = registers8(H) + 1;
            registers8(H)++;
            break;
        case 0x2C:
            half_carry = (registers8(L) & 0xF) == 0xF;
            result = registers8(L) + 1;
            registers8(L)++;
            break;
        case 0x34:
            half_carry = (memory.read(registers16(HL)) & 0xF) == 0xF;
            result = registers16(HL) + 1;
            memory.write(registers16(HL), memory.read(registers16(HL)+1));
            break;
        case 0x3C:
            half_carry = (registers8(A) & 0xF) == 0xF;
            result = registers8(A) + 1;
            registers8(A)++;
            break;
    }
    if (!result)
        zero = 1;
    registers8(F) = (registers8(F) & 0x10) | (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT);
}

void CPU::dec_rr() {
    switch (instruction) {
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

//void CPU::dec_r() {
//    switch (instruction) {
//        case 0x05:
//            registers[B]--;
//            break;
//        case 0x0D:
//            registers[C]--;
//            break;
//        case 0x15:
//            registers[D]--;
//            break;
//        case 0x1D:
//            registers[E]--;
//            break;
//        case 0x25:
//            registers[H]--;
//            break;
//        case 0x2D:
//            registers[L]--;
//            break;
//        case 0x35:
//            memory.write(registers16(HL), memory.read(registers16(HL)-1));
//            break;
//        case 0x3D:
//            registers[A]--;
//            break;
//    }
//}

void CPU::dec_r() {
    uint8_t result;
    uint8_t half_carry = 0;
    uint8_t zero = 0;
    switch (instruction) {
        case 0x05:
            half_carry = (registers8(B) & 0xF) == 0xF;
            result = registers8(B) - 1;
            registers8(B)--;
            break;
        case 0x0D:
            half_carry = (registers8(C) & 0xF) == 0xF;
            result = registers8(C) - 1;
            registers8(C)--;
            break;
        case 0x15:
            half_carry = (registers8(D) & 0xF) == 0xF;
            result = registers8(D) - 1;
            registers8(D)--;
            break;
        case 0x1D:
            half_carry = (registers8(E) & 0xF) == 0xF;
            result = registers8(E) - 1;
            registers8(E)--;
            break;
        case 0x25:
            half_carry = (registers8(H) & 0xF) == 0xF;
            result = registers8(H) - 1;
            registers8(H)--;
            break;
        case 0x2D:
            half_carry = (registers8(L) & 0xF) == 0xF;
            result = registers8(L) - 1;
            registers8(L)--;
            break;
        case 0x35:
            half_carry = (memory.read(registers16(HL)) & 0xF) == 0xF;
            result = registers16(HL) - 1;
            memory.write(registers16(HL), memory.read(registers16(HL)-1));
            break;
        case 0x3D:
            half_carry = (registers8(A) & 0xF) == 0xF;
            result = registers8(A) - 1;
            registers8(A)--;
            break;
    }
    if (!result)
        zero = 1;
    registers8(F) = (registers8(F) & 0x1) | (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT);
}

void CPU::push() {
    switch (instruction) {
        case 0xC5:
            //memory.write(--sp, registers16(BC));
            memory.write(--sp, registers8(B));
            memory.write(--sp, registers8(C));
            //std::cout << "Pushing BC: ";
            //printf("%x\n", registers16(BC));
            break;
        case 0xD5:
            //memory.write(--sp, registers16(DE));
            memory.write(--sp, registers8(D));
            memory.write(--sp, registers8(E));
            //std::cout << "Pushing DE: ";
            //printf("%x\n", registers16(DE));
            break;
        case 0xE5:
            //memory.write(--sp, registers16(HL));
            memory.write(--sp, registers8(H));
            //printf("Wrote H to sp: %x\n", sp);
            memory.write(--sp, registers8(L));
            //printf("Wrote L to sp: %x\n", sp);
            //std::cout << "Pushing HL: ";
            //printf("%x\n", registers16(HL));
            //printf("H: %x ", registers8(H));
            //printf("L: %x\n", registers8(L));
            //printf("sp: %x\n", sp);
            break;
        case 0xF5:
            //memory.write(--sp, registers16(AF));
            memory.write(--sp, registers8(A));
            memory.write(--sp, registers8(F));
            //std::cout << "Pushing AF: ";
            //printf("%x\n", registers16(AF));
            break;
    }
}

void CPU::pop() {
    switch (instruction) {
        case 0xC1:
            //registers16(BC) = memory.read(sp++);
            registers8(C) = memory.read(sp++);
            registers8(B) = memory.read(sp++);
            //std::cout << "Popping BC: ";
            //printf("%x\n", registers16(BC));
            break;
        case 0xD1:
            //registers16(DE) = memory.read(sp++);
            registers8(E) = memory.read(sp++);
            registers8(D) = memory.read(sp++);
            //std::cout << "Popping DE: ";
            //printf("%x\n", registers16(DE));
            break;
        case 0xE1:
            //registers16(HL) = memory.read(sp++);
            //printf("Reading L from sp: %x\n", sp);
            registers8(L) = memory.read(sp++);
            //printf("Reading H from sp: %x\n", sp);
            registers8(H) = memory.read(sp++);
            //std::cout << "Popping HL: ";
            //printf("%x\n", registers16(HL));
            //printf("H: %x ", registers8(H));
            //printf("L: %x\n", registers8(L));
            //printf("sp: %x\n", sp);
            break;
        case 0xF1:
            //registers16(AF) = memory.read(sp++);
            // Least significant nibble of F is always zero
            registers8(F) = memory.read(sp++) & 0xF0;
            registers8(A) = memory.read(sp++);
            //std::cout << "Popping AF: ";
            //printf("%x\n", registers16(AF));
            break;
    }
}

void CPU::rla() {
    uint8_t high_bit = (registers8(A) & 0x80);
    registers8(A) = (registers8(A) << 1) | ((registers8(F) & F_CARRY_MASK) ? 1 : 0);
    registers8(F) = high_bit << 1;
}

void CPU::rlca() {
    uint8_t high_bit = (registers8(A) & 0x80);
    registers8(A) = (registers8(A) << 1) | (high_bit ? 1 : 0);
    registers8(F) = high_bit << 1;
}

void CPU::rra() {
    uint8_t low_bit = (registers8(A) & 0x01);
    registers8(A) = (registers8(A) >> 1) | ((registers8(F) & F_CARRY_MASK) ? 0x80 : 0);
    registers8(F) = low_bit << F_CARRY_SHIFT;
}

void CPU::rrca() {
    uint8_t low_bit = (registers8(A) & 0x01);
    registers8(A) = (registers8(A) >> 1) | (low_bit ? 0x80 : 0);
    registers8(F) = low_bit << F_CARRY_SHIFT;
}

void CPU::cb_instruction() {
    uint8_t opcode = memory.read(pc++);
    switch (opcode & 0xF8) {
        case 0x00:
            rlc_r();
            break;
        case 0x08:
            rrc_r();
            break;
        case 0x10:
            rl_r();
            break;
        case 0x18:
            rr_r();
            break;
        case 0x20:
            sla_r();
            break;
        case 0x28:
            sra_r();
            break;
        case 0x30:
            swap_r();
            break;
        case 0x38:
            srl_r();
            break;
        case 0x40:
        case 0x48:
        case 0x50:
        case 0x58:
        case 0x60:
        case 0x68:
        case 0x70:
        case 0x78:
            bit();
            break;
        case 0x80:
        case 0x88:
        case 0x90:
        case 0x98:
        case 0xA0:
        case 0xA8:
        case 0xB0:
        case 0xB8:
            res();
            break;
        case 0xC0:
        case 0xC8:
        case 0xD0:
        case 0xD8:
        case 0xE0:
        case 0xE8:
        case 0xF0:
        case 0xF8:
            set();
            break;
    }
}

void CPU::rlc_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t high_bit = (reg & 0x80);
    reg = (reg << 1) | (high_bit ? 1 : 0);
    zero = reg == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (high_bit << 1);
}

void CPU::rrc_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t low_bit = (reg & 0x01);
    reg = (reg >> 1) | (low_bit ? 0x80 : 0);
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::rl_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t high_bit = (reg & 0x80);
    reg = (reg << 1) | ((registers8(F) & F_CARRY_MASK) ? 1 : 0);
    zero = reg == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (high_bit << 1);
}

void CPU::rr_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t low_bit = (reg & 0x01);
    reg = (reg >> 1) | ((registers8(F) & F_CARRY_MASK) ? 0x80 : 0);
    zero = reg == 0 ? 1 : 0;
    registers8(F) = low_bit << F_CARRY_SHIFT;
}

void CPU::sla_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t high_bit = (reg & 0x80);
    reg = reg << 1;
    zero = reg == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (high_bit << 1);
}

void CPU::sra_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t low_bit = (reg & 0x01);
    uint8_t high_bit = (reg & 0x80);
    reg = (reg >> 1) | (high_bit);
    zero = reg == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::swap_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    reg = ((reg & 0xF0) >> 4) | ((reg & 0x0F) << 4);
    zero = reg == 0 ? 1 : 0;
    registers8(F) = zero << F_ZERO_SHIFT;
}

void CPU::srl_r() {
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero;
    uint8_t low_bit = (reg & 0x01);
    reg = reg >> 1;
    zero = reg == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::bit() {
    uint8_t bit = (0x1 << left_reg_index(instruction));
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    uint8_t zero = (reg & bit) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_HALF_CARRY_MASK | (registers8(F) & F_CARRY_MASK);
}

void CPU::res() {
    uint8_t bit = (0x1 << left_reg_index(instruction));
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    reg = reg & ~bit;
}

void CPU::set() {
    uint8_t bit = (0x1 << left_reg_index(instruction));
    uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
    reg = reg | bit;
}

void CPU::jp_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    pc = concat_bytes(lsb, msb);
}

void CPU::jp_hl() {
    pc = registers16(HL);
}

void CPU::jp_c_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    uint16_t address = concat_bytes(lsb, msb);
    if (check_condition(instruction))
        pc = address;
}

void CPU::jr_imm() {
    pc += (int8_t) memory.read(pc++);
}

void CPU::jr_c_imm() {
    int8_t offset = memory.read(pc++);
    if (check_condition(instruction))
        pc += offset;
}

void CPU::call_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    memory.write(--sp, pc >> 8);
    memory.write(--sp, pc & 0xFF);
    pc = concat_bytes(lsb, msb);
    //std::cout << "Calling function at: ";
    //printf("%x\n", pc);
}

void CPU::call_c_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    uint16_t address = concat_bytes(lsb, msb);
    if (check_condition(instruction)) {
        memory.write(--sp, pc >> 8);
        memory.write(--sp, pc & 0xFF);
        pc = address;
        //std::cout << "Calling function at: ";
        //printf("%x\n", pc);
    }
}

void CPU::ret() {
    uint8_t lsb = memory.read(sp++);
    uint8_t msb = memory.read(sp++);
    pc = concat_bytes(lsb, msb);
    //std::cout << "Returning to address: ";
    //printf("%x\n", pc);
}

void CPU::ret_c() {
    if(check_condition(instruction)) {
        uint8_t lsb = memory.read(sp++);
        uint8_t msb = memory.read(sp++);
        pc = concat_bytes(lsb, msb);
        //std::cout << "Returning to address: ";
        //printf("%x\n", pc);
    }
}

void CPU::reti() {
    ret();
    ei();
}

void CPU::rst() {
    memory.write(--sp, pc >> 8);
    memory.write(--sp, pc & 0xFF);
    switch (instruction) {
        case 0xC7:
            pc = 0x00;
            break;
        case 0xCF:
            pc = 0x08;
            break;
        case 0xD7:
            pc = 0x10;
            break;
        case 0xDF:
            pc = 0x18;
            break;
        case 0xE7:
            pc = 0x20;
            break;
        case 0xEF:
            pc = 0x28;
            break;
        case 0xF7:
            pc = 0x30;
            break;
        case 0xFF:
            pc = 0x38;
            break;
        default:
            std::cout << "Invalid call of rst\n" << std::endl;
            break;
    }
}

bool CPU::check_condition(uint8_t instr) {
    switch (instr) {
        case 0x20:
        case 0xC0:
        case 0xC2:
        case 0xC4:
            return !(registers8(F) & F_ZERO_MASK);
        case 0x30:
        case 0xD0:
        case 0xD2:
        case 0xD4:
            return !(registers8(F) & F_CARRY_MASK);
        case 0x28:
        case 0xC8:
        case 0xCA:
        case 0xCC:
            return registers8(F) & F_ZERO_MASK;
        case 0x38:
        case 0xD8:
        case 0xDA:
        case 0xDC:
            return registers8(F) & F_CARRY_MASK;
        default:
            return false;
    }
}

void CPU::ei() {
    interrupt_delay = 2;
}

void CPU::di() {
    interrupt_enabled = false;
    interrupt_delay = 0;
}

void CPU::halt() {
    halted = true;
}

void CPU::cpl() {
    registers8(A) = ~registers8(A);
    registers8(F) = registers8(F) | F_NEG_MASK | F_HALF_CARRY_MASK;
}

void CPU::ccf() {
    registers8(F) = (registers8(F) & 0x80) | ~(registers8(F) & 0x10);
}

void CPU::scf() {
    registers8(F) = (registers8(F) & 0x80) | 0x10;
}

void CPU::daa() {
    uint8_t carry = 0;
    uint8_t zero;
    // simpler if you just gather all together and negate if N flag
    if (!(registers8(F) & F_NEG_MASK)) {
        uint8_t a_value = registers8(A);
        if (registers8(A) > /*0x99*/0x9F || (registers8(F) & F_CARRY_MASK)) {
            registers8(A) += 0x60;
            carry = 1;
        }
        if ((registers8(A) & 0xF) > 0x09 || (registers8(F) & F_HALF_CARRY_MASK))
            registers8(A) += 0x6;
    } else {
        if (registers8(F) & F_CARRY_MASK) {
            registers8(A) -= 0x60;
            //carry = 1;
        }
        if (registers8(F) & F_HALF_CARRY_MASK)
            registers8(A) -= 0x6;
    }

    zero = (registers8(A) == 0);

    registers8(F) = (registers8(F) & 0x50/*0x10*/) | (zero << F_ZERO_SHIFT) | (carry << F_CARRY_SHIFT);
}

uint8_t& CPU::get_reg_by_index(unsigned index) {
    if (index == 7)
        return registers8(A);
    else
        return registers8(registers_order[index]);
}

uint8_t CPU::read_reg(unsigned index) {
    // Do this using an array matching index to actual reg pos?
    if (index == 7)
        return registers8(A);
    else if (index == 6)
        return memory.read(registers16(HL));
    else
        return registers8(index);
}

void CPU::write_reg(unsigned index, uint8_t data) {
    if (index == 7)
        registers8(A) = data;
    else if (index == 6)
        memory.write(registers16(HL), data);
    else
        registers8(index) = data;
}

inline unsigned CPU::left_reg_index(uint8_t instr) {
    return (instr & 0x38) >> 3;
}

inline unsigned CPU::right_reg_index(uint8_t instr) {
    return instr & 0x07;
}

//void CPU::increment_timer() {
//    if (timer_regs[1]++ == 0) { // overflow
//        timer_regs[1] = timer_regs[2];
//        // interrupt requested
//    }
//}
