#pragma once

#include "Memory.h"

#include <cstdint>
#include <fstream>
#include <iostream>

#define CYCLE_TIME 0.2384185791

#define F_ZERO_MASK 0x80
#define F_NEG_MASK 0x40
#define F_HALF_CARRY_MASK 0x20
#define F_CARRY_MASK 0x10
#define F_ZERO_SHIFT 7
#define F_NEG_SHIFT 6
#define F_HALF_CARRY_SHIFT 5
#define F_CARRY_SHIFT 4

enum Registers8 {
    B, C, D, E, H, L, A, F
};

enum Registers16 {
    BC, DE, HL, AF
};

class CPU {
public:
    CPU(std::string rom_path);
    void run();
private:
    std::ifstream rom_file;

    uint8_t reg_A;
    uint8_t reg_F;
    uint8_t reg_B;
    uint8_t reg_C;
    uint8_t reg_D;
    uint8_t reg_E;
    uint8_t reg_H;
    uint8_t reg_L;
    uint16_t sp;
    uint16_t pc;

    uint8_t registers[8];
    uint16_t& registers16(unsigned index);

    Memory memory;
    uint8_t rom[0x7fff];
    uint8_t temp_memory[0x10000];
    uint8_t joypad_reg;
    uint8_t serial_regs[2] = {0};
    uint8_t timer_regs[4] = {0};
    uint8_t& get_mem(uint16_t address);

    unsigned wait_cycles = 0;

    void debug_regs_default();
    void print_registers();

    void fetch_next_instr();

    void load_r_r();
    void load_r_imm();
    void load_mem_from_reg_r();
    void load_r_mem_from_reg();
    void load_mem_from_imm_r();
    void load_r_mem_from_imm();

    void load_mem_r(uint16_t address);
    void load_r_mem(uint16_t address);

    void load_rr_imm();
    void load_mem_from_imm_sp();
    void load_sp_hl();
    void load_hl_sp_plus_simm();

    void alu_r();
    void alu_imm();

    void alu_add(uint8_t operand);
    void alu_adc(uint8_t operand);
    void alu_sub(uint8_t operand);
    void alu_sbc(uint8_t operand);
    void alu_and(uint8_t operand);
    void alu_xor(uint8_t operand);
    void alu_or(uint8_t operand);
    void alu_cp(uint8_t operand);

    void inc_rr();
    void inc_r();
    void dec_rr();
    void dec_r();

    void push();
    void pop();

    void rla();
    void rlca();
    void rra();
    void rrca();

    void jp_imm();
    void jp_hl();
    void jp_c_imm();

    bool check_condition(uint8_t code);

    uint8_t& get_reg_by_index(unsigned index);
    uint8_t read_reg(unsigned index);
    void write_reg(unsigned index, uint8_t data);
    inline unsigned left_reg_index(uint8_t instr);
    inline unsigned right_reg_index(uint8_t instr);
    
    inline uint16_t concat_bytes(uint8_t lsb, uint8_t msb);

    void increment_timer();
};
