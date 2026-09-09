# Timing Diagnosis Report (Revised Using Research Baseline)

## Summary
Using `/home/runner/work/gameboy-emulator/gameboy-emulator/GAMEBOY_TIMING_RESEARCH_REPORT.md` as the timing baseline, the primary timing fault is now clear:

1. **Timers are clocked at the wrong scale (4× too slow), plus an off-by-one on DIV.**
2. **CPU real-time stepping drops elapsed time (no catch-up), causing speed drift.**
3. **STAT/LYC interrupt semantics are incorrect, causing timing-sensitive interrupt behavior to diverge.**

These are more impactful than the earlier diagnosis alone and better explain broad timing instability.

## Evidence from current implementation

### A) DIV is incremented too slowly and with an off-by-one
- File: `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.cpp`
- Code path increments DIV when:
  - `if (divider_counter++ == 256) { ... }`

Problems:
- **Unit mismatch**: emulator loop is one **M-cycle** step; DIV on DMG should tick every **64 M-cycles** (16384 Hz), not every 256.
- **Off-by-one**: post-increment equality check makes it fire every **257** M-cycles.

Quick check performed:
- Expected DIV: 16384 Hz.
- Current logic effective rate: ~4080.06 Hz (`1048576 / 257`), ~**4× too slow**.

### B) TIMA scheduling table is 4× too large for M-cycle stepping
- File: `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.h`
- Timer periods configured as:
  - `unsigned timer_control_values[4] = { 1024, 16, 64, 256 };`
- File: `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.cpp`
- Used once per emulated M-cycle in:
  - `timer_counter++ >= timer_control_values[...]`

Research baseline for DMG (M-cycles): `{256, 4, 16, 64}`.

Result:
- All four TAC frequencies are configured **4× too slow**.
- This alone can break game logic and interrupt pacing even if CPU instruction timing were otherwise correct.

### C) Real-time pacing loses elapsed host time
- File: `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.cpp`
- Logic:
  - compute `dt`
  - if `dt > CYCLE_TIME`, execute exactly one emulated M-cycle and set `prev_cycle = current_time`

Problem:
- If host scheduling delay exceeds one cycle, only one cycle is emulated and the remainder is discarded.
- This produces drift/slowdown under normal OS jitter or render load.

### D) STAT behavior is not hardware-correct and can cause interrupt timing errors
- File: `/home/runner/work/gameboy-emulator/gameboy-emulator/PPU.cpp`
- Current behavior includes:
  - forcibly OR-ing STAT source bits (`0x08/0x10/0x20`) on mode transitions
  - setting `INT_LCDSTAT` directly on transitions
  - writing `0x40` when `LY==LYC`

Problems versus hardware semantics:
- STAT interrupt source enable bits are CPU-controlled configuration; PPU should not force-enable them.
- LYC coincidence flag is STAT bit 2, not bit 6.
- STAT interrupt should follow enable bits and rising-edge behavior of the combined source line.

This can create false/missing interrupts and visible timing desync in games using STAT raster timing.

### E) Additional timing-model gaps (secondary but important)
- `halted` is set in `CPU::halt()` but not used in the run loop, so HALT timing behavior is effectively absent.
- Interrupt service entry cost (5 M-cycles total on hardware) is not explicitly modeled.
- PPU mode 3 is fixed to 172 dots (minimum) and does not model variable penalties (SCX/window/OBJ).

These are not the first-order cause of the current gross timing drift, but they matter for compatibility.

## Prioritized root-cause diagnosis

1. **Highest impact**: timer unit mismatch (`DIV` period and `timer_control_values`) causes systemic 4× timer-rate error.
2. **High impact**: dropped elapsed time in CPU pacing loop causes emulator speed drift/slowdown.
3. **High compatibility impact**: incorrect STAT/LYC semantics produce interrupt timing divergence.
4. **Secondary**: HALT/interrupt-latency/mode3-variability omissions reduce cycle accuracy.

## Test activity performed for this diagnosis

1. Source-level timing audit against:
   - `/home/runner/work/gameboy-emulator/gameboy-emulator/GAMEBOY_TIMING_RESEARCH_REPORT.md`
   - `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.cpp`
   - `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.h`
   - `/home/runner/work/gameboy-emulator/gameboy-emulator/PPU.cpp`
2. Numerical timing sanity check (executed in-shell) confirming:
   - DIV current effective rate near 4.08 kHz versus 16.384 kHz expected.
   - TAC timing table entries all 4× expected M-cycle periods.
3. Build attempt:
   - `make` fails in this environment due missing SDL2 headers (`SDL2/SDL.h` not installed), so runtime ROM tests were not possible here.

## Conclusion
The timing issue is not just a minor off-by-one; it is primarily a **clock-domain scaling error** in timer/divider logic, compounded by **host-time cycle dropping** and **incorrect STAT interrupt semantics**. This revised diagnosis supersedes the earlier report and is consistent with the deeper timing research baseline.
