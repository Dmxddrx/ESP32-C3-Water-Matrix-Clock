Randomized layer-based SandClock module

Files:
- sand_clock.h
- sand_clock.cpp

Integration:
1. Put both files in the same PlatformIO component/source folder as max7219.h/.cpp.
2. In general.cpp include: #include "sand_clock.h"
3. After MAX7219_Init() and brightness setup, call SandClock::begin();
4. In the LEVEL branch of drawMatrixFrame(nowMs), call SandClock::update(nowMs);
   Do not call MAX7219_Clear() or MAX7219_Update() around SandClock::update().
5. When entering LEVEL mode from the mode button or Bluetooth, call
   SandClock::reset(millis());

Pattern assumptions:
- The first three top source layers match the user's pattern exactly:
  {1}, {7,16}, {6,15,24}.
- Remaining source layers continue the diagonal sweep. LED 8 is placed in
  the 8-grain centre layer because LED 1 has already been used. This assigns
  all 64 top LEDs exactly once and matches the 1,2,...,8,...,1 layer sizes.
- Bottom destinations are anti-diagonal layers expanding from LED 128:
  {128}, {120,127}, {112,119,126}, etc.
- Source and destination lists are shuffled independently within each layer,
  then paired by index. No source or destination is reused.
- A single moving LED follows a continuous path from bottom-module LED 65
  (local coordinate 0,0) to the selected destination. Earlier settled pixels
  are preserved.

Timing:
- DROP_STEP_MS = 75 ms per path pixel.
- GRAIN_GAP_MS = 120 ms between completed grains.
- COMPLETE_HOLD_MS = 1800 ms before a new cycle.

Call update() regularly. For smooth motion, MATRIX_FRAME_MS should be around
40-60 ms (or at least not much larger than DROP_STEP_MS).
