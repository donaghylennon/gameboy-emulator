#include "CPU.h"

#include <chrono>

CPU::CPU(std::string rom_path) {
    memory.load_rom(rom_path);
    memory.load_boot_rom("Resources/dmg_boot.bin");
}

void CPU::run() {
    int i = 0;

    pc = 0;

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

            if (!wait_cycles) {
                handle_interrupts();

                fetch_next_instr();

                if (!cb_prefixed)
                    wait_cycles += cycle_table[instruction];
                else {
                    wait_cycles += cb_cycle_table[instruction];
                    cb_prefixed = false;
                }
            }
            wait_cycles--;

            ppu.run_cycle();

            if (divider_counter++ == 256) {
                divider_counter = 0;
                memory.increment_divider();
            }

            if ((memory.read(0xFF07) & 0x4) && timer_counter++ >= timer_control_values[memory.read(0xFF07) & 0x3]) {
                timer_counter = 0;
                memory.increment_timer();
            }

            if (interrupt_delay) {
                if (!--interrupt_delay)
                    interrupt_enabled = true;
            }
            if (!wait_event_poll--) {
                SDL_Event event;
                while(SDL_PollEvent(&event)) {
                    switch(event.type) {
                        case SDL_QUIT:
                            running = false;
                            SDL_Quit();
                            break;
                    }
                }
                wait_event_poll = 60;
            }
        }
    }
}

uint16_t& CPU::registers16(unsigned index) {
    return reinterpret_cast<uint16_t*>(registers)[index];
}

uint8_t& CPU::registers8(unsigned index) {
    return registers[index];
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
    instruction = memory.read(pc++);

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
                            cb_instruction();
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
}

void CPU::load_r_r() {
    if ((instruction & 0x7) == 0x6) {
        get_reg_by_index(left_reg_index(instruction)) = memory.read(registers16(HL));
    } else if ((instruction & 0x38) == 0x30) {
        memory.write(registers16(HL), get_reg_by_index(right_reg_index(instruction)));
    } else {
        get_reg_by_index(left_reg_index(instruction)) = get_reg_by_index(right_reg_index(instruction));
    }
}

void CPU::load_r_imm() {
    uint8_t data = memory.read(pc++);
    if (instruction == 0x36)
        memory.write(registers16(HL), data);
    else
        get_reg_by_index(left_reg_index(instruction)) = data;
}

void CPU::load_mem_from_reg_r() {
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
            break;
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

void CPU::load_mem_from_imm_r() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);

    load_mem_r(concat_bytes(lsb, msb));
}

inline uint16_t CPU::concat_bytes(uint8_t lsb, uint8_t msb) {
    return (msb << 8) | lsb;
}

void CPU::load_r_mem_from_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);

    load_r_mem(concat_bytes(lsb, msb));
}

void CPU::load_mem_r(uint16_t address) {
    memory.write(address, registers8(A));
}

void CPU::load_r_mem(uint16_t address) {
    registers8(A) = memory.read(address);
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
    uint8_t half_carry;
    int8_t operand = memory.read(pc++);

    carry = (sp & 0xFF) + (uint8_t)operand > 0xFF;
    half_carry = (sp & 0xF) + ((uint8_t)operand & 0xF) > 0xF;

    registers16(HL) = sp + (int8_t) operand;
    registers8(F) = (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::alu_r() {
    uint8_t operand;
    if ((instruction & 0x7) == 0x6) {
        operand = memory.read(registers16(HL));
    } else {
        operand = get_reg_by_index(right_reg_index(instruction));
    }
    switch (instruction & 0xF8) {
        case 0x80:
            alu_add(operand);
            break;
        case 0x88:
            alu_adc(operand);
            break;
        case 0x90:
            alu_sub(operand);
            break;
        case 0x98:
            alu_sbc(operand);
            break;
        case 0xA0:
            alu_and(operand);
            break;
        case 0xA8:
            alu_xor(operand);
            break;
        case 0xB0:
            alu_or(operand);
            break;
        case 0xB8:
            alu_cp(operand);
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
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;

    carry = registers8(A) > 0xFF - operand;
    half_carry = (registers8(A) & 0xF) > 0xF - (operand & 0xF);
    registers8(A) += operand;
    
    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
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
    uint8_t zero = 0;
    registers8(A) &= operand;

    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_HALF_CARRY_MASK;
}

void CPU::alu_xor(uint8_t operand) {
    uint8_t zero = 0;
    registers8(A) ^= operand;

    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT);
}

void CPU::alu_or(uint8_t operand) {
    uint8_t zero = 0;
    registers8(A) |= operand;

    zero = registers8(A) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT);
}

void CPU::alu_cp(uint8_t operand) {
    uint8_t carry;
    uint8_t half_carry;
    uint8_t zero;

    uint8_t result = registers8(A) - operand;
    carry = registers8(A) < operand ? 1 : 0;
    half_carry = (registers8(A) & 0xF) < (operand & 0xF) ? 1 : 0;
    zero = result == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_NEG_MASK | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::add_rr() {
    uint16_t operand;
    uint8_t carry;
    uint8_t half_carry = 0;
    switch (instruction) {
        case 0x09:
            operand = registers16(BC);
            break;
        case 0x19:
            operand = registers16(DE);
            break;
        case 0x29:
            operand = registers16(HL);
            break;
        case 0x39:
            operand = sp;
            break;
    }
    if (((registers16(HL) & 0xFFF) + (operand & 0xFFF)) & 0x1000)
        half_carry = 1;
    carry = registers16(HL) > ((0xFFFF - operand) & 0xFFFF);

    registers16(HL) += operand;

    registers8(F) = (registers8(F) & F_ZERO_MASK) | (half_carry << F_HALF_CARRY_SHIFT) | (carry << F_CARRY_SHIFT);
}

void CPU::add_sp_imm() {
    uint8_t carry = 0;
    uint8_t half_carry = 0;
    int8_t operand = memory.read(pc++);

    carry = (sp & 0xFF) + (uint8_t)operand > 0xFF;
    half_carry = (sp & 0xF) + ((uint8_t)operand & 0xF) > 0xF;

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
            result = memory.read(registers16(HL)) + 1;
            memory.write(registers16(HL), memory.read(registers16(HL))+1);
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

void CPU::dec_r() {
    uint8_t result;
    uint8_t half_carry = 0;
    uint8_t zero = 0;
    switch (instruction) {
        case 0x05:
            half_carry = (registers8(B) & 0xF) == 0x0;
            result = registers8(B) - 1;
            registers8(B)--;
            break;
        case 0x0D:
            half_carry = (registers8(C) & 0xF) == 0x0;
            result = registers8(C) - 1;
            registers8(C)--;
            break;
        case 0x15:
            half_carry = (registers8(D) & 0xF) == 0x0;
            result = registers8(D) - 1;
            registers8(D)--;
            break;
        case 0x1D:
            half_carry = (registers8(E) & 0xF) == 0x0;
            result = registers8(E) - 1;
            registers8(E)--;
            break;
        case 0x25:
            half_carry = (registers8(H) & 0xF) == 0x0;
            result = registers8(H) - 1;
            registers8(H)--;
            break;
        case 0x2D:
            half_carry = (registers8(L) & 0xF) == 0x0;
            result = registers8(L) - 1;
            registers8(L)--;
            break;
        case 0x35:
            half_carry = (memory.read(registers16(HL)) & 0xF) == 0x0;
            result = memory.read(registers16(HL)) - 1;
            memory.write(registers16(HL), memory.read(registers16(HL))-1);
            break;
        case 0x3D:
            half_carry = (registers8(A) & 0xF) == 0x0;
            result = registers8(A) - 1;
            registers8(A)--;
            break;
    }
    if (!result)
        zero = 1;
    registers8(F) = (registers8(F) & 0x10) | F_NEG_MASK | (zero << F_ZERO_SHIFT) | (half_carry << F_HALF_CARRY_SHIFT);
}

void CPU::push() {
    switch (instruction) {
        case 0xC5:
            memory.write(--sp, registers8(B));
            memory.write(--sp, registers8(C));
            break;
        case 0xD5:
            memory.write(--sp, registers8(D));
            memory.write(--sp, registers8(E));
            break;
        case 0xE5:
            memory.write(--sp, registers8(H));
            memory.write(--sp, registers8(L));
            break;
        case 0xF5:
            memory.write(--sp, registers8(A));
            memory.write(--sp, registers8(F));
            break;
    }
}

void CPU::pop() {
    switch (instruction) {
        case 0xC1:
            registers8(C) = memory.read(sp++);
            registers8(B) = memory.read(sp++);
            break;
        case 0xD1:
            registers8(E) = memory.read(sp++);
            registers8(D) = memory.read(sp++);
            break;
        case 0xE1:
            registers8(L) = memory.read(sp++);
            registers8(H) = memory.read(sp++);
            break;
        case 0xF1:
            // Least significant nibble of F is always zero
            registers8(F) = memory.read(sp++) & 0xF0;
            registers8(A) = memory.read(sp++);
            break;
    }
}

void CPU::rla() {
    uint8_t high_bit = (registers8(A) & 0x80);
    registers8(A) = (registers8(A) << 1) | ((registers8(F) & F_CARRY_MASK) ? 1 : 0);
    registers8(F) = high_bit >> 3;
}

void CPU::rlca() {
    uint8_t high_bit = (registers8(A) & 0x80);
    registers8(A) = (registers8(A) << 1) | (high_bit ? 1 : 0);
    registers8(F) = high_bit >> 3;
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
    cb_prefixed = true;
    instruction = memory.read(pc++);
    switch (instruction & 0xF8) {
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
    uint8_t zero;
    uint8_t high_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        high_bit = (byte & 0x80);
        byte = (byte << 1) | (high_bit ? 1 : 0);
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        high_bit = (reg & 0x80);
        reg = (reg << 1) | (high_bit ? 1 : 0);
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (high_bit >> 3);
}

void CPU::rrc_r() {
    uint8_t zero;
    uint8_t low_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        low_bit = (byte & 0x01);
        byte = (byte >> 1) | (low_bit ? 0x80 : 0);
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        low_bit = (reg & 0x01);
        reg = (reg >> 1) | (low_bit ? 0x80 : 0);
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::rl_r() {
    uint8_t zero;
    uint8_t high_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        high_bit = (byte & 0x80);
        byte = (byte << 1) | ((registers8(F) & F_CARRY_MASK) ? 1 : 0);
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        high_bit = (reg & 0x80);
        reg = (reg << 1) | ((registers8(F) & F_CARRY_MASK) ? 1 : 0);
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (high_bit >> 3);
}

void CPU::rr_r() {
    uint8_t zero;
    uint8_t low_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        low_bit = (byte & 0x01);
        byte = (byte >> 1) | ((registers8(F) & F_CARRY_MASK) ? 0x80 : 0);
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        low_bit = (reg & 0x01);
        reg = (reg >> 1) | ((registers8(F) & F_CARRY_MASK) ? 0x80 : 0);
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::sla_r() {
    uint8_t zero;
    uint8_t high_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        high_bit = (byte & 0x80);
        byte = byte << 1;
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        high_bit = (reg & 0x80);
        reg = reg << 1;
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (high_bit >> 3);
}

void CPU::sra_r() {
    uint8_t zero;
    uint8_t high_bit;
    uint8_t low_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        low_bit = (byte & 0x01);
        high_bit = (byte & 0x80);
        byte = (byte >> 1) | (high_bit);
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        low_bit = (reg & 0x01);
        high_bit = (reg & 0x80);
        reg = (reg >> 1) | (high_bit);
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::swap_r() {
    uint8_t zero;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4);
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        reg = ((reg & 0xF0) >> 4) | ((reg & 0x0F) << 4);
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = zero << F_ZERO_SHIFT;
}

void CPU::srl_r() {
    uint8_t zero;
    uint8_t low_bit;
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        low_bit = (byte & 0x01);
        byte = byte >> 1;
        zero = byte == 0 ? 1 : 0;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        low_bit = (reg & 0x01);
        reg = reg >> 1;
        zero = reg == 0 ? 1 : 0;
    }
    registers8(F) = (zero << F_ZERO_SHIFT) | (low_bit << F_CARRY_SHIFT);
}

void CPU::bit() {
    uint8_t bit = (0x1 << left_reg_index(instruction));
    uint8_t zero;
    uint8_t value;
    if ((instruction & 0x7) == 0x6) {
        value = memory.read(registers16(HL));
    } else {
        value = get_reg_by_index(right_reg_index(instruction));
    }
    zero = (value & bit) == 0 ? 1 : 0;
    registers8(F) = (zero << F_ZERO_SHIFT) | F_HALF_CARRY_MASK | (registers8(F) & F_CARRY_MASK);
}

void CPU::res() {
    uint8_t bit = (0x1 << left_reg_index(instruction));
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        byte = byte & ~bit;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        reg = reg & ~bit;
    }
}

void CPU::set() {
    uint8_t bit = (0x1 << left_reg_index(instruction));
    if ((instruction & 0x7) == 0x6) {
        uint8_t byte = memory.read(registers16(HL));
        byte = byte | bit;
        memory.write(registers16(HL), byte);
    } else {
        uint8_t& reg = get_reg_by_index(right_reg_index(instruction));
        reg = reg | bit;
    }
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
    if (check_condition(instruction)) {
        wait_cycles += 1;
        pc = address;
    }
}

void CPU::jr_imm() {
    int8_t offset = memory.read(pc++);
    pc += offset;
}

void CPU::jr_c_imm() {
    int8_t offset = memory.read(pc++);
    if (check_condition(instruction)) {
        wait_cycles += 1;
        pc += offset;
    }
}

void CPU::call_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    memory.write(--sp, pc >> 8);
    memory.write(--sp, pc & 0xFF);
    pc = concat_bytes(lsb, msb);
}

void CPU::call_c_imm() {
    uint8_t lsb = memory.read(pc++);
    uint8_t msb = memory.read(pc++);
    uint16_t address = concat_bytes(lsb, msb);
    if (check_condition(instruction)) {
        wait_cycles += 3;
        memory.write(--sp, pc >> 8);
        memory.write(--sp, pc & 0xFF);
        pc = address;
    }
}

void CPU::ret() {
    uint8_t lsb = memory.read(sp++);
    uint8_t msb = memory.read(sp++);
    pc = concat_bytes(lsb, msb);
}

void CPU::ret_c() {
    if(check_condition(instruction)) {
        wait_cycles += 3;
        uint8_t lsb = memory.read(sp++);
        uint8_t msb = memory.read(sp++);
        pc = concat_bytes(lsb, msb);
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
    registers8(F) = (registers8(F) & 0x80) | ((~(registers8(F) & 0x10)) & 0x10);
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
        if (registers8(A) > 0x9F || (registers8(F) & F_CARRY_MASK)) {
            registers8(A) += 0x60;
            carry = 1;
        }
        if ((registers8(A) & 0xF) > 0x09 || (registers8(F) & F_HALF_CARRY_MASK))
            registers8(A) += 0x6;
    } else {
        if (registers8(F) & F_CARRY_MASK) {
            registers8(A) -= 0x60;
        }
        if (registers8(F) & F_HALF_CARRY_MASK)
            registers8(A) -= 0x6;
    }

    zero = (registers8(A) == 0);

    registers8(F) = (registers8(F) & 0x50) | (zero << F_ZERO_SHIFT) | (carry << F_CARRY_SHIFT);
}

uint8_t& CPU::get_reg_by_index(unsigned index) {
    if (index == 7)
        return registers8(A);
    else
        return registers8(registers_order[index]);
}

uint8_t CPU::read_reg(unsigned index) {
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

void CPU::handle_interrupts() {
    if (interrupt_enabled) {
        if ((memory.read(0xFFFF) & 0x1) && (memory.read(0xFF0F) & 0x1)) {
            memory.write(0xFF0F, memory.read(0xFF0F) & ~0x1);
            di();
            memory.write(--sp, pc >> 8);
            memory.write(--sp, pc & 0xFF);
            pc = 0x40;
        } else if ((memory.read(0xFFFF) & 0x2) && (memory.read(0xFF0F) & 0x2)) {
            memory.write(0xFF0F, memory.read(0xFF0F) & ~0x2);
            di();
            memory.write(--sp, pc >> 8);
            memory.write(--sp, pc & 0xFF);
            pc = 0x48;
        } else if ((memory.read(0xFFFF) & 0x4) && (memory.read(0xFF0F) & 0x4)) {
            memory.write(0xFF0F, memory.read(0xFF0F) & ~0x4);
            di();
            memory.write(--sp, pc >> 8);
            memory.write(--sp, pc & 0xFF);
            pc = 0x50;
        }
    }
}
