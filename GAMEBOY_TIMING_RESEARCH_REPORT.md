# In-Depth Game Boy Timing Research Report

## Scope
This report summarizes hardware timing behavior that materially affects emulator correctness, with emphasis on CPU/PPU/timer/interrupt/DMA interactions and common implementation pitfalls.

## 1) Core clock model and cycle units

- DMG/CGB master clock (normal speed): **4.194304 MHz**.  
  Source: Pan Docs Specifications.
- System clock is **master/4**, so one CPU machine cycle (M-cycle) is 4 T-cycles (dots) in normal speed.
- PPU timing is expressed in **dots** (aka T-cycles).  
  Source: Pan Docs Rendering overview and STAT docs.

Practical emulator implication:
- Keep one canonical base unit (usually dot/T-cycle or half-dot abstraction if needed later).
- Derive all subsystems from that shared clock to avoid drift.

## 2) Frame and scanline timing (PPU)

- A frame is **154 scanlines** (LY 0..153), with visible lines 0..143 and VBlank lines 144..153.
- Each scanline is **456 dots**.
- PPU modes:
  - Mode 2 (OAM scan): **80 dots**
  - Mode 3 (drawing): **172..289 dots** (variable)
  - Mode 0 (HBlank): **376 - Mode3 dots** (complements to 456)
  - Mode 1 (VBlank): **4560 dots** (10 lines)
- Typical frame duration is ~**16.74 ms** (~59.73 Hz on handhelds), not exactly 60 Hz.

Why Mode 3 must be variable:
- Scroll/window/object behavior introduces per-line penalties that lengthen Mode 3 and shorten Mode 0 accordingly.
- Emulators with fixed Mode 3 often break raster effects and some STAT timing cases.

## 3) VRAM/OAM access windows and correctness impact

- During rendering activity, CPU access to video memory is restricted:
  - VRAM accessible in Modes 0-2, not in Mode 3.
  - OAM accessible in Modes 0-1, not in Modes 2-3.
- Reads during blocked periods typically return undefined data (often $FF); writes are ignored.

Implication:
- Correct memory-access gating is not optional for many titles and timing test ROMs.
- If blocking is wrong, behavior around sprite fetch, tile streaming, and mid-frame effects diverges.

## 4) Timer subsystem (DIV/TIMA/TMA/TAC)

### 4.1 Baseline rates

- DIV (FF04) increments at **16384 Hz** on DMG normal speed (and 32768 Hz in CGB double speed).
- TIMA rate selected by TAC bits:
  - 00: every 256 M-cycles (4096 Hz)
  - 01: every 4 M-cycles (262144 Hz)
  - 10: every 16 M-cycles (65536 Hz)
  - 11: every 64 M-cycles (16384 Hz)
- TIMA overflow reloads from TMA and requests timer interrupt.

### 4.2 Obscure but important hardware behavior

Pan Docs’ timer obscure behavior notes:
- DIV is the visible portion of a continuously incrementing system counter.
- Writing DIV or changing TAC can cause edge-triggered effects (extra tick behavior under specific bit transitions).
- TIMA overflow signaling is **not instantaneous**:
  - On overflow cycle A, TIMA becomes 00.
  - On cycle B (one M-cycle later), TMA is copied and IF.Timer is set.
- Writes during the A/B window have special behavior and can suppress/override expected outcomes.

Implication:
- “Simple counter every N cycles” is enough for many games, but high-accuracy compatibility needs edge-based modeling (or equivalent event model).

## 5) Interrupt timing and CPU control flow hazards

- IME gates global interrupt handling.
- `ei` is delayed by one instruction.
- Interrupt dispatch consumes **5 M-cycles** total (2 wait + push PC over 2 cycles + vector set over 1 cycle).
- Priority order is VBlank > STAT > Timer > Serial > Joypad (bit order).

### HALT behavior

- CPU wakes from HALT when `(IE & IF) != 0`.
- With IME=0 and pending interrupt, the HALT bug can occur: PC increment behavior is abnormal and next opcode fetch sequencing can differ.

Implication:
- Correct IME delay + HALT semantics is required for many CPU test ROMs and for interrupt-heavy software timing patterns.

## 6) DMA timing constraints

### OAM DMA (FF46)

- OAM DMA copies $A0 bytes, duration **160 M-cycles** (640 dots normal speed).
- On DMG, CPU is effectively limited to HRAM during DMA (practically requiring HRAM wait routines in software).
- Interrupts during DMA can corrupt expected behavior due to stack/ROM access.
- DMA during active rendering can produce predictable sprite glitches/visibility artifacts depending on mode.

Implication:
- Immediate-copy implementations that do not model transfer time and bus conflict can desynchronize sprite behavior and timing-sensitive logic.

## 7) Double-speed mode (CGB) timing split

- In CGB double speed, CPU/timer/serial/OAM DMA run faster.
- LCD/PPU dot timing does **not** speed up accordingly.

Implication:
- Never bind LCD progression directly to “CPU instructions executed”; bind to dot-time and convert CPU progression by current speed mode.

## 8) Common emulator timing failure patterns

1. **Host-time step dropping** (no catch-up loop): emulator loses elapsed time under OS jitter.
2. **Fixed-latency PPU Mode 3**: breaks raster effects and STAT edges.
3. **Off-by-one divider/timer counters**: causes subtle drift and failed timer tests.
4. **No delayed TIMA overflow model**: incorrect IF/TIMA behavior in edge cases.
5. **Incorrect EI/HALT sequencing**: control-flow divergence in test ROMs.
6. **Instant OAM DMA** with no timing/bus conflict: sprite anomalies and logic mismatch.

## 9) Recommended architecture for reliable timing

1. Adopt a single monotonic emulated clock (dot or sub-dot event time).
2. Run CPU as scheduled events in M-cycles mapped onto that clock.
3. Run PPU as a state machine with variable Mode 3 duration and exact LY/STAT transitions.
4. Model timers from system-counter edges (or equivalent deterministic edge events).
5. Implement interrupt pipeline explicitly (IME state, EI delay, dispatch latency).
6. Model OAM DMA as a timed transfer with mode/bus effects.
7. Use host-time only to decide *how much emulated time to execute*, never to directly skip subsystem steps.

## 10) Practical validation strategy

- CPU correctness: Blargg CPU instruction tests + interrupt/HALT-focused suites.
- Timer correctness: Mooneye timer/div/tima-edge tests.
- PPU correctness: STAT/LYC/raster-focused tests (including STAT blocking cases).
- DMA correctness: OAM DMA timing/conflict tests and sprite behavior repro ROMs.
- Run tests with deterministic emulation stepping (fixed seed/order), then add real-time pacing once deterministic core passes.

## 11) Direct relevance to this repository’s current issue

The previously observed slowdown/drift pattern aligns with known failure pattern #1 (host-time step dropping), and existing timer boundary behavior risks pattern #3. Even without full cycle-perfect emulation, fixing:

- elapsed-time catch-up,
- exact divider/timer boundary counting,
- and a deterministic subsystem clock model

should materially improve correctness before deeper edge cases (TIMA A/B windows, HALT bug corner cases, DMA bus conflicts) are implemented.

## Sources

Primary technical references used:

- Pan Docs (repository): https://github.com/gbdev/pandocs
- Specifications: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Specifications.md
- Rendering overview: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Rendering.md
- LCD Status (STAT/LY/LYC): https://raw.githubusercontent.com/gbdev/pandocs/master/src/STAT.md
- Accessing VRAM and OAM: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Accessing_VRAM_and_OAM.md
- Timer and Divider Registers: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Timer_and_Divider_Registers.md
- Timer Obscure Behaviour: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Timer_Obscure_Behaviour.md
- Interrupts: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Interrupts.md
- Interrupt Sources: https://raw.githubusercontent.com/gbdev/pandocs/master/src/Interrupt_Sources.md
- HALT behavior: https://raw.githubusercontent.com/gbdev/pandocs/master/src/halt.md
- OAM DMA Transfer: https://raw.githubusercontent.com/gbdev/pandocs/master/src/OAM_DMA_Transfer.md
- CGB registers and double-speed notes: https://raw.githubusercontent.com/gbdev/pandocs/master/src/CGB_Registers.md
