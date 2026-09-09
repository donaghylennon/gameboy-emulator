# Timing Diagnosis Report (Work-in-Progress Game Boy Emulator)

## Summary
The emulator timing issue is primarily caused by the main CPU loop advancing emulation by **at most one machine cycle per outer-loop iteration**, even when host time has advanced by many cycles. This causes emulation speed drift/slowdown under normal scheduler jitter and rendering load.

## Key Findings

1. **No catch-up when host time advances by more than one cycle**
   - In `/home/runner/work/gameboy-emulator/gameboy-emulator/CPU.cpp`, `CPU::run()` computes `dt` and only executes one emulation step when `dt > CYCLE_TIME`.
   - `prev_cycle` is then set directly to `current_time`, dropping any extra elapsed time instead of consuming all pending cycles.
   - Effect: timing falls behind real time and game speed becomes unstable/slower.

2. **Off-by-one timing in divider increment**
   - In `CPU::run()`, divider logic uses:
     - `if (divider_counter++ == 256) { ... }`
   - This triggers after 257 increments rather than 256.
   - Effect: divider/timer-related behavior drifts from hardware timing.

3. **Off-by-one timing in TIMA scheduling gate**
   - Timer logic uses:
     - `if ((memory.read(0xFF07) & 0x4) && timer_counter++ >= timer_control_values[...]) { ... }`
   - Because of post-increment with `>=`, threshold behavior is shifted by one cycle.
   - Effect: TIMA increment cadence is late, impacting game logic relying on timer interrupts.

4. **Potential precision/stability concern from float microsecond thresholding**
   - `CYCLE_TIME` is compared against `float dt` (microseconds).
   - Float precision and threshold comparison at sub-microsecond scale can add jitter.
   - Not the main bug, but contributes to unstable pacing.

## Why this explains the observed timing problem
- CPU, PPU, DIV, and TIMA all depend on cycle-accurate stepping.
- If CPU stepping drops elapsed time, every derived subsystem runs late.
- Additional off-by-one errors in divider/timer accumulation compound long-run drift.

## Recommended Fix Strategy
1. Replace single-step threshold logic with an accumulator/catch-up loop that executes as many emulated cycles as required by elapsed host time.
2. Correct divider/timer counters to fire exactly on expected cycle boundaries.
3. Use integer/nanosecond-based timing math (or high-precision accumulator units) to reduce floating-point jitter.
4. Re-test with known timing ROMs (e.g., Blargg timing/timer tests) after fixes.

## Notes
- PPU stepping ratio appears intentionally modeled at 4 PPU dots per CPU machine cycle (`PPU::run_cycle()` called once per CPU machine cycle and internally iterating 4 times), so the central timing issue is in CPU pacing and timer boundary handling rather than that ratio itself.
