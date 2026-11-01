#include "sand_clock.h"
#include "max7219.h"
#include "display_manager.h"

#include <esp_system.h>   // ESP32-C3 hardware random number generator
#include <string.h>

namespace SandClock {
namespace {

constexpr uint8_t MATRIX_SIZE = 8;
constexpr uint8_t GRAIN_TOTAL = 64;


// Animation timing. update() must be called frequently from general_run().
constexpr uint32_t DROP_STEP_MS = 75;       // Time between moving-grain pixels
constexpr uint32_t GRAIN_GAP_MS = 120;      // Pause after one grain settles
constexpr uint32_t COMPLETE_HOLD_MS = 1800; // Pause before a new randomized cycle

// Source and destination selection orders generated independently
// from their respective 8x8 priority maps.
uint8_t sourceOrder[GRAIN_TOTAL];       // Top LED numbers: 1..64
uint8_t destinationOrder[GRAIN_TOTAL];  // Bottom LED numbers: 65..128

bool sourceRemoved[GRAIN_TOTAL] = {};
bool lowerSettled[MATRIX_SIZE][MATRIX_SIZE] = {}; // [row][local column]

uint8_t grainIndex = 0;
bool grainInFlight = false;
bool animationComplete = false;
bool initialized = false;

// Current moving grain's path through the lower module.
// The route starts at bottom-module LED 65 (local coordinate 0,0).
uint8_t dropPathX[MATRIX_SIZE];
uint8_t dropPathY[MATRIX_SIZE];
uint8_t dropPathLength = 0;
uint8_t dropPathIndex = 0;
uint8_t currentDestination = 0;

uint32_t lastDropStepMs = 0;
uint32_t lastGrainFinishMs = 0;
uint32_t completedMs = 0;

/*
 * TOP SLAVE SOURCE PRIORITY MAP
 *
 * Each cell represents the selection priority of one LED.
 *
 * Lower priority numbers are selected earlier.
 * Equal priority numbers are randomly shuffled.
 *
 * This creates the centre-outward, progressive sand-emptying
 * behaviour specified in the hourglass probability diagram.
 */
static const uint8_t SOURCE_PRIORITY[MATRIX_SIZE][MATRIX_SIZE] = {
    {  1,  2,  4,  6,  8, 10, 14, 19 },
    {  2,  3,  5,  7,  9, 13, 17, 22 },
    {  4,  5,  6,  8, 11, 15, 20, 25 },
    {  6,  7,  8,  9, 12, 18, 24, 27 },
    {  8,  9, 11, 12, 16, 21, 26, 29 },
    { 10, 13, 15, 18, 21, 23, 28, 31 },
    { 14, 17, 20, 24, 26, 28, 30, 32 },
    { 19, 22, 25, 27, 29, 31, 32, 33 }
};

constexpr uint8_t SOURCE_MAX_PRIORITY = 33;

/*
 * BOTTOM MASTER DESTINATION PRIORITY MAP
 *
 * Priority 1 starts at the outer bottom tip.
 * The priority increases towards the neck.
 *
 * Equal priority numbers are randomly shuffled.
 *
 * This controls the order in which the lower chamber receives
 * and accumulates the sand particles.
 */
static const uint8_t DESTINATION_PRIORITY[MATRIX_SIZE][MATRIX_SIZE] = {
    { 35, 34, 33, 32, 30, 28, 26, 24 },
    { 34, 31, 29, 28, 27, 25, 23, 22 },
    { 33, 29, 21, 20, 19, 18, 17, 16 },
    { 32, 28, 20, 15, 14, 13, 12, 11 },
    { 30, 27, 19, 14, 10,  9,  8,  7 },
    { 28, 25, 18, 13,  9,  6,  5,  4 },
    { 26, 23, 17, 12,  8,  5,  3,  2 },
    { 24, 22, 16, 11,  7,  4,  2,  1 }
};

constexpr uint8_t DESTINATION_MAX_PRIORITY = 35;




/*
 * Rotate an 8x8 LED position 90 degrees anticlockwise.
 *
 * Input and output LED numbers are local module numbers: 1..64.
 *
 * Coordinate transformation:
 *
 *   New X = Old Y
 *   New Y = 7 - Old X
 *
 * This is the standard rotation used by the MASTER module.
 */
uint8_t rotateLedCCW(uint8_t led)
{
    const uint8_t index = led - 1;

    const uint8_t x = index % MATRIX_SIZE;
    const uint8_t y = index / MATRIX_SIZE;

    const uint8_t newX = y;
    const uint8_t newY = 7 - x;

    return (uint8_t)(newY * MATRIX_SIZE + newX + 1);
}


uint8_t rotateSlaveLedCCW(uint8_t led)
{
    const uint8_t index = led - 1;

    const uint8_t x = index % MATRIX_SIZE;
    const uint8_t y = index / MATRIX_SIZE;

    const uint8_t newX = x;
    const uint8_t newY = y;

    return (uint8_t)(newY * MATRIX_SIZE + newX + 1);
}

/*
 * Uniformly shuffle LEDs having the same priority.
 *
 * The priority map determines which group is selected first.
 * This function only randomizes the order inside that group.
 *
 * Every LED is preserved exactly once.
 */
void shufflePriorityGroup(uint8_t *leds, uint8_t count)
{
    if (leds == nullptr || count < 2) {
        return;
    }

    // Fisher-Yates shuffle using the ESP32-C3 hardware RNG.
    for (int16_t i = (int16_t)count - 1; i > 0; --i)
    {
        const uint8_t j =
            (uint8_t)(esp_random() % (uint32_t)(i + 1));

        const uint8_t temp = leds[i];
        leds[i] = leds[j];
        leds[j] = temp;
    }
}

/*
 * BUILD PRIORITY ORDER
 *
 * Read an 8x8 priority matrix and generate the corresponding LED order.
 *
 * Process:
 *
 * 1. Start from priority 1.
 * 2. Find all LEDs with that priority.
 * 3. Convert their coordinates into LED numbers.
 * 4. Apply the appropriate module rotation.
 * 5. Randomize LEDs sharing the same priority.
 * 6. Append them to the final LED order.
 * 7. Continue with the next priority.
 *
 * sourceModule = true:
 *     Use top slave LED numbering and slave rotation correction.
 *
 * sourceModule = false:
 *     Use bottom master LED numbering and normal master rotation.
 *
 * This ensures that the generated order follows the supplied
 * probability diagram exactly.
 */
bool buildPriorityOrder(
    const uint8_t priorityMap[MATRIX_SIZE][MATRIX_SIZE],
    uint8_t maximumPriority,
    bool sourceModule,
    uint8_t *outputOrder
)
{
    if (priorityMap == nullptr || outputOrder == nullptr) {
        return false;
    }

    uint8_t outputOffset = 0;

    // Progress through the priority levels in ascending order.
    for (uint8_t priority = 1;
         priority <= maximumPriority;
         ++priority)
    {
        uint8_t priorityGroup[GRAIN_TOTAL] = {};
        uint8_t groupCount = 0;

        // Find all LEDs having the current priority.
        for (uint8_t row = 0; row < MATRIX_SIZE; ++row)
        {
            for (uint8_t col = 0; col < MATRIX_SIZE; ++col)
            {
                if (priorityMap[row][col] != priority) {
                    continue;
                }

                // Convert matrix coordinates into local LED number.
                const uint8_t localLed =
                    (uint8_t)(row * MATRIX_SIZE + col + 1);

                uint8_t finalLed;

                if (sourceModule)
                {
                    // Top slave: anticlockwise rotation with
                    // the existing LED 1 / LED 57 correction.
                    finalLed = rotateSlaveLedCCW(localLed);
                }
                else
                {
                    // Bottom master: normal anticlockwise rotation.
                    // No slave corner correction is applied.
                    finalLed =
                        (uint8_t)(64 + rotateLedCCW(localLed));
                }

                // Protect the priority group from exceeding its array capacity.
                if (groupCount >= GRAIN_TOTAL) {
                    return false;
                }

                priorityGroup[groupCount++] = finalLed;
            }
        }

        // Every priority from 1 to maximumPriority must exist.
        if (groupCount == 0) {
            return false;
        }

        // Randomize only LEDs that share the same priority.
        shufflePriorityGroup(priorityGroup, groupCount);

        // Append the completed priority group to the final order.
        for (uint8_t i = 0; i < groupCount; ++i)
        {
            if (outputOffset >= GRAIN_TOTAL) {
                return false;
            }

            outputOrder[outputOffset++] = priorityGroup[i];
        }
    }

    // Both modules must produce exactly 64 LED positions.
    return outputOffset == GRAIN_TOTAL;
}

/*
 * BUILD COMPLETE TRANSFER PLAN
 *
 * SOURCE:
 *     Uses SOURCE_PRIORITY.
 *     Lower priority numbers are selected first.
 *     Equal-priority LEDs are randomly shuffled.
 *
 * DESTINATION:
 *     Uses DESTINATION_PRIORITY.
 *     Lower priority numbers are selected first.
 *     Equal-priority LEDs are randomly shuffled.
 *
 * The source and destination orders are generated independently.
 * The grain state machine pairs them using grainIndex.
 *
 * IMPORTANT:
 *     This only changes LED selection order.
 *     It does NOT modify the falling path or animation timing.
 */
bool buildTransferPlan()
{
    // Generate the top slave source sequence.
    if (!buildPriorityOrder(
            SOURCE_PRIORITY,
            SOURCE_MAX_PRIORITY,
            true,
            sourceOrder))
    {
        return false;
    }

    // Generate the bottom master destination sequence.
    if (!buildPriorityOrder(
            DESTINATION_PRIORITY,
            DESTINATION_MAX_PRIORITY,
            false,
            destinationOrder))
    {
        return false;
    }

    return true;
}

// ------------------------------------------------------------
// MOVING GRAIN PATH
// ------------------------------------------------------------

void appendPathPoint(uint8_t x, uint8_t y)
{
    if (dropPathLength >= MATRIX_SIZE) {
        return;
    }

    dropPathX[dropPathLength] = x;
    dropPathY[dropPathLength] = y;

    ++dropPathLength;
}

void buildDropPath(uint8_t bottomLed)
{
    // Bottom LED numbers are 65..128.
    // Convert to local 0..7 coordinates.
    const uint8_t localIndex = (uint8_t)(bottomLed - 65);

    const uint8_t targetX = localIndex % MATRIX_SIZE;
    const uint8_t targetY = localIndex / MATRIX_SIZE;

    dropPathLength = 0;
    dropPathIndex = 0;

    // After 90-degree anticlockwise rotation, the neck is at (0,7).
    uint8_t x = 0;
    uint8_t y = 7;

    // The rotated central falling path is:
    // 121 -> 114 -> 107 -> 100 -> 93 -> 86 -> 79 -> 72.
    //
    // Follow the diagonal first, then move along the remaining
    // horizontal or vertical distance to the selected destination.
    appendPathPoint(x, y);

    while (x < targetX && y > targetY)
    {
        ++x;
        --y;

        appendPathPoint(x, y);
    }

    while (x < targetX)
    {
        ++x;
        appendPathPoint(x, y);
    }

    while (y > targetY)
    {
        --y;
        appendPathPoint(x, y);
    }
}

// ------------------------------------------------------------
// FRAMEBUFFER RENDERING
// ------------------------------------------------------------

void render()
{
    MAX7219_Clear();

    // Upper slave: all grains start ON; a source pixel disappears as soon
    // as its grain begins moving.
    for (uint8_t i = 0; i < GRAIN_TOTAL; ++i) {
        if (!sourceRemoved[i]) {
            const uint8_t x = i % MATRIX_SIZE;
            const uint8_t y = i / MATRIX_SIZE;

            MAX7219_SetPixel(x, y, 1);
        }
    }

    // Lower master: preserve all grains that have already settled.
    for (uint8_t y = 0; y < MATRIX_SIZE; ++y) {
        for (uint8_t x = 0; x < MATRIX_SIZE; ++x) {

            if (lowerSettled[y][x]) {
                MAX7219_SetPixel(
                    (int16_t)(MATRIX_SIZE + x),
                    y,
                    1
                );
            }
        }
    }

    // Show exactly one moving grain at its current position.
    if (grainInFlight &&
        dropPathLength > 0 &&
        dropPathIndex < dropPathLength) {

        MAX7219_SetPixel(
            (int16_t)(MATRIX_SIZE + dropPathX[dropPathIndex]),
            dropPathY[dropPathIndex],
            1
        );
    }

    MAX7219_Update();
}

// ------------------------------------------------------------
// GRAIN STATE MACHINE
// ------------------------------------------------------------

void startNextGrain(uint32_t nowMs)
{
    if (grainIndex >= GRAIN_TOTAL) {
        animationComplete = true;
        completedMs = nowMs;
        return;
    }

    const uint8_t sourceLed = sourceOrder[grainIndex];
    currentDestination = destinationOrder[grainIndex];

    // Turn OFF the selected source pixel immediately.
    sourceRemoved[sourceLed - 1] = true;

    // Build a visible route from lower-module LED 65 to this grain's
    // randomized resting position.
    buildDropPath(currentDestination);

    grainInFlight = true;
    lastDropStepMs = nowMs;

    render();
}

void finishCurrentGrain(uint32_t nowMs)
{
    const uint8_t localIndex =
        (uint8_t)(currentDestination - 65);

    const uint8_t x = localIndex % MATRIX_SIZE;
    const uint8_t y = localIndex / MATRIX_SIZE;

    // Keep the destination LED ON permanently for the rest of this cycle.
    lowerSettled[y][x] = true;

    grainInFlight = false;
    ++grainIndex;

    lastGrainFinishMs = nowMs;

    if (grainIndex >= GRAIN_TOTAL) {
        animationComplete = true;
        completedMs = nowMs;
    }

    render();
}

} // namespace

// ------------------------------------------------------------
// PUBLIC API
// ------------------------------------------------------------

void begin()
{
    initialized = true;
    reset(millis());
}

void reset(uint32_t nowMs)
{
    if (!initialized) {
        initialized = true;
    }

    memset(sourceRemoved, 0, sizeof(sourceRemoved));
    memset(lowerSettled, 0, sizeof(lowerSettled));
    memset(sourceOrder, 0, sizeof(sourceOrder));
    memset(destinationOrder, 0, sizeof(destinationOrder));

    grainIndex = 0;
    grainInFlight = false;
    animationComplete = false;
    currentDestination = 0;

    dropPathLength = 0;
    dropPathIndex = 0;

    // Generate a fresh centre-weighted random transfer plan for every cycle.
    // Every source and destination LED is still included exactly once.
    if (!buildTransferPlan()) {
    DisplayManager::logMessage("SandClock: Transfer plan generation failed!");
    return;
}

    // Start the first grain on the next update() call.
    lastGrainFinishMs = nowMs - GRAIN_GAP_MS;
    lastDropStepMs = nowMs;
    completedMs = 0;

    // Initial frame: top 1..64 ON, bottom 65..128 OFF.
    render();
}

void update(uint32_t nowMs)
{
    if (!initialized) {
        begin();
        return;
    }

    if (animationComplete) {
        if ((uint32_t)(nowMs - completedMs) >= COMPLETE_HOLD_MS) {

            // A new cycle receives a fresh weighted randomized pairing
            // independently for every layer.
            reset(nowMs);
        }

        return;
    }

    if (grainInFlight) {

        if ((uint32_t)(nowMs - lastDropStepMs) >= DROP_STEP_MS) {
            lastDropStepMs = nowMs;

            if ((uint8_t)(dropPathIndex + 1) < dropPathLength) {
                ++dropPathIndex;
                render();
            } else {
                finishCurrentGrain(nowMs);
            }
        }

        return;
    }

    if (grainIndex >= GRAIN_TOTAL) {
        animationComplete = true;
        completedMs = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - lastGrainFinishMs) >= GRAIN_GAP_MS) {
        startNextGrain(nowMs);
    }
}

} // namespace SandClock
