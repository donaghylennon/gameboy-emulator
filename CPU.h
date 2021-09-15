#pragma once

#include "Memory.h"
#include "PPU.h"

#include <cstdint>
#include <fstream>
#include <iostream>

//#define CYCLE_TIME 0.2384185791
#define CYCLE_TIME 0.953674316406

#define F_ZERO_MASK 0x80
#define F_NEG_MASK 0x40
#define F_HALF_CARRY_MASK 0x20
#define F_CARRY_MASK 0x10
#define F_ZERO_SHIFT 7
#define F_NEG_SHIFT 6
#define F_HALF_CARRY_SHIFT 5
#define F_CARRY_SHIFT 4

static unsigned cycle_table[256] = {
    1, 3, 2, 2, 1, 1, 2, 1, 5, 2, 2, 2, 1, 1, 2, 1,
    0, 3, 2, 2, 1, 1, 2, 1, 3, 2, 2, 2, 1, 1, 2, 1,
    2, 3, 2, 2, 1, 1, 2, 1, 2, 2, 2, 2, 1, 1, 2, 1,
    2, 3, 2, 2, 3, 3, 3, 1, 2, 2, 2, 2, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    2, 2, 2, 2, 2, 2, 0, 2, 1, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    2, 3, 3, 4, 3, 4, 2, 4, 2, 4, 3, 0, 3, 6, 2, 4,
    2, 3, 3, 0, 3, 4, 2, 4, 2, 4, 3, 0, 3, 0, 2, 4,
    3, 3, 2, 0, 0, 4, 2, 4, 4, 1, 4, 0, 0, 0, 2, 4,
    3, 3, 2, 1, 0, 4, 2, 4, 3, 2, 4, 1, 0, 0, 2, 4,
};

static unsigned cb_cycle_table[256] = {
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
    2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
    2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
};

enum Registers8 {
    C, B, E, D, L, H, F, A
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
    unsigned wait_event_poll = 0;

    uint16_t sp;
    uint16_t pc;

    uint8_t instruction;
    bool cb_prefixed = false;

    uint8_t registers[8];
    uint16_t& registers16(unsigned index);
    uint8_t& registers8(unsigned index);
    unsigned registers_order[8] = { B, C, D, E, H, L, A, F };

    Memory memory;
    PPU ppu = PPU(memory);
    unsigned ppu_delay = 0;
    unsigned interrupt_delay = 0;
    bool interrupt_enabled = true;

    unsigned divider_counter = 0;
    unsigned timer_counter = 0;
    unsigned timer_control_values[4] = { 1024, 16, 64, 256 };

    bool halted = false;

    unsigned wait_cycles = 0;

    void print_registers();
    void print_registers16();

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

    void add_rr();
    void add_sp_imm();

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

    void cb_instruction();

    void rlc_r();
    void rrc_r();
    void rl_r();
    void rr_r();
    void sla_r();
    void sra_r();
    void swap_r();
    void srl_r();
    void bit();
    void res();
    void set();

    void jp_imm();
    void jp_hl();
    void jp_c_imm();
    void jr_imm();
    void jr_c_imm();
    void call_imm();
    void call_c_imm();
    void ret();
    void ret_c();
    void reti();
    void rst();

    bool check_condition(uint8_t code);

    void ei();
    void di();

    void halt();

    void cpl();
    void ccf();
    void scf();

    void daa();

    uint8_t& get_reg_by_index(unsigned index);
    uint8_t read_reg(unsigned index);
    void write_reg(unsigned index, uint8_t data);
    inline unsigned left_reg_index(uint8_t instr);
    inline unsigned right_reg_index(uint8_t instr);
    
    inline uint16_t concat_bytes(uint8_t lsb, uint8_t msb);

    void handle_interrupts();
};
