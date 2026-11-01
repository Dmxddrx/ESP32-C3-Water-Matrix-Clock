
#include "sand_clock.h"
#include "max7219.h"

#include <esp_system.h>   // ESP32-C3 hardware random number generator
#include <string.h>

namespace SandClock {
namespace {

constexpr uint8_t MATRIX_SIZE = 8;
constexpr uint8_t GRAIN_TOTAL = 64;
constexpr uint8_t LAYER_TOTAL = 15;

// Animation timing. update() must be called frequently from general_run().
constexpr uint32_t DROP_STEP_MS = 75;       // Time between moving-grain pixels
constexpr uint32_t GRAIN_GAP_MS = 120;      // Pause after one grain settles
constexpr uint32_t COMPLETE_HOLD_MS = 1800; // Pause before a new randomized cycle

/*
 * TOP SLAVE source layers, using the LED numbering supplied by the user.
 * Each row is one layer. LEDs are 1..64, row-major within the top module.
 *
 * First layers follow the requested pattern:
 *   Layer 0: 1
 *   Layer 1: 7, 16
 *   Layer 2: 6, 15, 24
 *
 * The remaining layers continue the same diagonal sweep. LED 8 is placed
 * in the eight-grain centre layer because LED 1 was already used in layer 0.
 * This makes every top LED 1..64 appear exactly once, with matching layer
 * sizes for the lower chamber.
 */
static const uint8_t SOURCE_LAYERS[LAYER_TOTAL][MATRIX_SIZE] = {
    { 1,  0,  0,  0,  0,  0,  0,  0 },
    { 7, 16,  0,  0,  0,  0,  0,  0 },
    { 6, 15, 24,  0,  0,  0,  0,  0 },
    { 5, 14, 23, 32,  0,  0,  0,  0 },
    { 4, 13, 22, 31, 40,  0,  0,  0 },
    { 3, 12, 21, 30, 39, 48,  0,  0 },
    { 2, 11, 20, 29, 38, 47, 56,  0 },
    { 8, 10, 19, 28, 37, 46, 55, 64 },
    { 9, 18, 27, 36, 45, 54, 63,  0 },
    {17, 26, 35, 44, 53, 62,  0,  0 },
    {25, 34, 43, 52, 61,  0,  0,  0 },
    {33, 42, 51, 60,  0,  0,  0,  0 },
    {41, 50, 59,  0,  0,  0,  0,  0 },
    {49, 58,  0,  0,  0,  0,  0,  0 },
    {57,  0,  0,  0,  0,  0,  0,  0 }
};

static const uint8_t LAYER_COUNTS[LAYER_TOTAL] = {
    1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1
};

// Source physics selection zone.
// Layer indices are zero-based:
// Layer index 2 = physical 3rd diagonal row.
// Layer index 10 = physical 11th diagonal row.
constexpr uint8_t SOURCE_PHYSICS_FIRST_LAYER = 2;
constexpr uint8_t SOURCE_PHYSICS_LAST_LAYER = 10;

constexpr uint8_t SOURCE_PHYSICS_LAYER_COUNT =
    SOURCE_PHYSICS_LAST_LAYER -
    SOURCE_PHYSICS_FIRST_LAYER + 1;

// A grain's source and destination are paired by their index in these arrays.
// Each layer is weighted-shuffled independently so grains never cross layer
// boundaries. Centre positions are more likely to be selected earlier.
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

// ------------------------------------------------------------
// RANDOMIZATION
// ------------------------------------------------------------

/*
 * Return a relative probability weight for a position in a layer.
 *
 * The middle positions have the highest selection probability.
 * The outer positions have the lowest probability.
 *
 * Example weights:
 *
 *   Count 1:  [1]
 *   Count 2:  [1, 1]
 *   Count 3:  [1, 4, 1]
 *   Count 4:  [1, 4, 4, 1]
 *   Count 5:  [1, 4, 7, 4, 1]
 *   Count 6:  [1, 4, 7, 7, 4, 1]
 *   Count 7:  [1, 4, 7, 10, 7, 4, 1]
 *   Count 8:  [1, 4, 7, 10, 10, 7, 4, 1]
 *
 * The weights apply to both source LEDs and destination LEDs.
 * They do not remove any candidates. Every LED is eventually selected.
 */
uint8_t centerProbabilityWeight(uint8_t position, uint8_t count)
{
    if (count <= 2) {
        return 1;
    }

    // Distance from the nearest edge of the candidate list.
    const uint8_t distanceFromEdge =
        (position < (count - 1U - position))
            ? position
            : (count - 1U - position);

    // Centre positions receive greater selection weights.
    return (uint8_t)(1U + (3U * distanceFromEdge));
}

/*
 * Weighted Fisher-Yates shuffle without replacement.
 *
 * Unlike a normal Fisher-Yates shuffle, each candidate has a different
 * probability of being selected for the next position.
 *
 * The selected value and its weight are swapped together, preserving the
 * original position weight of every candidate.
 *
 * This produces a weighted random order while guaranteeing that every
 * candidate appears exactly once.
 */
void weightedShuffle(uint8_t *values, uint8_t count)
{
    if (values == nullptr || count < 2) {
        return;
    }

    uint8_t weights[MATRIX_SIZE] = {};

    // Assign a probability weight to every original layer position.
    for (uint8_t i = 0; i < count; ++i) {
        weights[i] = centerProbabilityWeight(i, count);
    }

    // Weighted random permutation using the ESP32-C3 hardware RNG.
    for (uint8_t i = 0; i < count; ++i) {

        uint16_t totalWeight = 0;

        // Calculate the total weight of the remaining candidates.
        for (uint8_t j = i; j < count; ++j) {
            totalWeight += weights[j];
        }

        if (totalWeight == 0) {
            return;
        }

        // Select one remaining candidate using its relative weight.
        uint16_t randomValue =
            (uint16_t)(esp_random() % totalWeight);

        uint8_t selected = i;

        for (uint8_t j = i; j < count; ++j) {

            if (randomValue < weights[j]) {
                selected = j;
                break;
            }

            randomValue -= weights[j];
        }

        // Move the selected LED and its probability weight together.
        const uint8_t tempValue = values[i];
        values[i] = values[selected];
        values[selected] = tempValue;

        const uint8_t tempWeight = weights[i];
        weights[i] = weights[selected];
        weights[selected] = tempWeight;
    }
}

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


/*
 * Rotate the SLAVE module 90 degrees anticlockwise.
 *
 * Apply the additional LED 1 / LED 57 corner correction
 * only to the SLAVE module.
 */
uint8_t rotateSlaveLedCCW(uint8_t led)
{
    uint8_t rotatedLed = rotateLedCCW(led);

    // Swap only the two corner positions on the SLAVE.
    if (rotatedLed == 1) {
        rotatedLed = 57;
    }
    else if (rotatedLed == 57) {
        rotatedLed = 1;
    }

    return rotatedLed;
}

/*
 * SOURCE PHYSICS SELECTION
 *
 * The middle particles of a diagonal layer are selected before its
 * outer particles become eligible.
 *
 * However, the next layer's middle particles can compete with the
 * previous layer's outer particles.
 *
 * This prevents the source chamber from emptying one complete row
 * at a time and produces a more natural centre-first sand movement.
 */

struct SourceCandidate {
    uint8_t layer;
    uint8_t position;
    uint8_t weight;
};


/*
 * Calculate how far a particle is from the nearest edge of its layer.
 *
 * Examples:
 *
 *   Count 3:  [0, 1, 0]
 *   Count 4:  [0, 1, 1, 0]
 *   Count 5:  [0, 1, 2, 1, 0]
 *   Count 6:  [0, 1, 2, 2, 1, 0]
 *
 * Higher distance means closer to the centre.
 */
uint8_t sourcePositionDistance(uint8_t position, uint8_t count)
{
    const uint8_t rightDistance =
        count - 1U - position;

    return (position < rightDistance)
        ? position
        : rightDistance;
}


/*
 * Determine whether all particles in the current centre-outward
 * shell of a source layer have already been selected.
 *
 * A layer can only expose its next outer shell when its current
 * shell has been completely selected.
 */
bool isSourceShellComplete(
    uint8_t layer,
    int8_t shellDistance,
    const uint8_t layerCounts[SOURCE_PHYSICS_LAYER_COUNT],
    const bool selected[SOURCE_PHYSICS_LAYER_COUNT][MATRIX_SIZE]
)
{
    const uint8_t count = layerCounts[layer];

    for (uint8_t position = 0; position < count; ++position)
    {
        if (sourcePositionDistance(position, count) ==
            (uint8_t)shellDistance)
        {
            if (!selected[layer][position]) {
                return false;
            }
        }
    }

    return true;
}


/*
 * Build the source LED selection order.
 *
 * SELECTION PHASE 1:
 *   Physical rows 1 and 2 retain their original behaviour.
 *
 * SELECTION PHASE 2:
 *   Physical row 3 middle LED is forced to be selected first.
 *
 * SELECTION PHASE 3:
 *   Physical rows 3..11 use progressive centre-first selection.
 *
 *   Each layer exposes its central shell first. Once that shell
 *   is completed, its next outer shell becomes available.
 *
 *   At the same time, completion of a layer's central shell
 *   activates the next layer's central shell.
 *
 *   Therefore a deeper middle particle can be selected before
 *   an outer particle in a previous layer.
 *
 * SELECTION PHASE 4:
 *   Physical rows 12..15 retain the original layer-by-layer
 *   weighted random selection.
 *
 * IMPORTANT:
 *   This function only controls SOURCE LED selection.
 *   Destination selection and falling animation are independent.
 */
bool buildSourceOrder()
{
    uint8_t sourceOffset = 0;

    // --------------------------------------------------------
    // PHASE 1: FIRST TWO SOURCE LAYERS
    // --------------------------------------------------------

    for (uint8_t layer = 0;
         layer < SOURCE_PHYSICS_FIRST_LAYER;
         ++layer)
    {
        const uint8_t count = LAYER_COUNTS[layer];

        uint8_t sourceLayer[MATRIX_SIZE] = {};

        // Copy and rotate the original source LEDs.
        for (uint8_t i = 0; i < count; ++i)
        {
            const uint8_t led = SOURCE_LAYERS[layer][i];

            if (led < 1 || led > 64) {
                return false;
            }

            sourceLayer[i] = rotateSlaveLedCCW(led);
        }

        // Preserve the existing weighted selection for these rows.
        weightedShuffle(sourceLayer, count);

        for (uint8_t i = 0; i < count; ++i)
        {
            sourceOrder[sourceOffset++] = sourceLayer[i];
        }
    }


    // --------------------------------------------------------
    // PHASE 2: PREPARE THE PHYSICS SELECTION ZONE
    // --------------------------------------------------------

    uint8_t physicsLeds
        [SOURCE_PHYSICS_LAYER_COUNT][MATRIX_SIZE] = {};

    uint8_t physicsCounts
        [SOURCE_PHYSICS_LAYER_COUNT] = {};

    uint8_t maxDistance
        [SOURCE_PHYSICS_LAYER_COUNT] = {};

    int8_t currentShell
        [SOURCE_PHYSICS_LAYER_COUNT] = {};

    bool selected
        [SOURCE_PHYSICS_LAYER_COUNT][MATRIX_SIZE] = {};


    uint8_t remainingGrains = 0;

    for (uint8_t localLayer = 0;
         localLayer < SOURCE_PHYSICS_LAYER_COUNT;
         ++localLayer)
    {
        const uint8_t actualLayer =
            SOURCE_PHYSICS_FIRST_LAYER + localLayer;

        const uint8_t count = LAYER_COUNTS[actualLayer];

        physicsCounts[localLayer] = count;

        // Maximum centre distance for this layer.
        maxDistance[localLayer] =
            (count - 1U) / 2U;

        // Initially, all physics layers are locked.
        currentShell[localLayer] = -1;

        remainingGrains += count;

        // Store the rotated source LEDs.
        for (uint8_t position = 0;
             position < count;
             ++position)
        {
            const uint8_t led =
                SOURCE_LAYERS[actualLayer][position];

            if (led < 1 || led > 64) {
                return false;
            }

            physicsLeds[localLayer][position] =
                rotateSlaveLedCCW(led);
        }
    }


    // --------------------------------------------------------
    // PHASE 3: FORCE THE THIRD ROW MIDDLE PARTICLE FIRST
    // --------------------------------------------------------

    // Physical third row contains three LEDs.
    // Its middle particle must be selected before its two edges.
    const uint8_t firstMiddlePosition =
        physicsCounts[0] / 2U;

    sourceOrder[sourceOffset++] =
        physicsLeds[0][firstMiddlePosition];

    selected[0][firstMiddlePosition] = true;
    --remainingGrains;

    // The third row's middle shell is complete.
    // Its two outer LEDs become available.
    currentShell[0] =
        (int8_t)maxDistance[0] - 1;


    // --------------------------------------------------------
    // PHASE 4: ACTIVATE THE FOURTH ROW MIDDLE PARTICLES
    // --------------------------------------------------------

    // The next layer starts with its central shell.
    // For four LEDs this means the two middle positions.
    currentShell[1] = maxDistance[1];

    // The first two physics layers are now active:
    //   Row 3: outer particles
    //   Row 4: middle particles
    uint8_t activeLayers = 2;


    // --------------------------------------------------------
    // PHASE 5: PROGRESSIVE CENTRE-TO-EDGE SELECTION
    // --------------------------------------------------------

    while (remainingGrains > 0)
    {
        SourceCandidate candidates
            [SOURCE_PHYSICS_LAYER_COUNT * MATRIX_SIZE];

        uint8_t candidateCount = 0;

        uint16_t totalWeight = 0;


        // Collect the currently available particles.
        // Only the current shell of each active layer is eligible.
        for (uint8_t layer = 0;
             layer < activeLayers;
             ++layer)
        {
            if (currentShell[layer] < 0) {
                continue;
            }

            const uint8_t count = physicsCounts[layer];

            for (uint8_t position = 0;
                 position < count;
                 ++position)
            {
                const uint8_t distance =
                    sourcePositionDistance(position, count);

                // Ignore already selected particles.
                if (selected[layer][position]) {
                    continue;
                }

                // Only particles in the current shell are eligible.
                if (distance != (uint8_t)currentShell[layer]) {
                    continue;
                }

                const uint8_t weight =
                    centerProbabilityWeight(position, count);

                candidates[candidateCount].layer = layer;
                candidates[candidateCount].position = position;
                candidates[candidateCount].weight = weight;

                ++candidateCount;

                totalWeight += weight;
            }
        }

        // No candidate should remain unavailable while grains remain.
        if (candidateCount == 0 || totalWeight == 0) {
            return false;
        }


        // ----------------------------------------------------
        // WEIGHTED RANDOM SELECTION
        // ----------------------------------------------------

        // Select among currently exposed particles.
        // Centre particles have higher probability.
        uint16_t randomValue =
            (uint16_t)(esp_random() % totalWeight);

        uint8_t selectedCandidate = 0;

        for (uint8_t i = 0; i < candidateCount; ++i)
        {
            if (randomValue < candidates[i].weight)
            {
                selectedCandidate = i;
                break;
            }

            randomValue -= candidates[i].weight;
        }

        const uint8_t selectedLayer =
            candidates[selectedCandidate].layer;

        const uint8_t selectedPosition =
            candidates[selectedCandidate].position;


        // Append the selected source LED to the transfer order.
        sourceOrder[sourceOffset++] =
            physicsLeds[selectedLayer][selectedPosition];

        selected[selectedLayer][selectedPosition] = true;

        --remainingGrains;


        // ----------------------------------------------------
        // CHECK WHETHER THE CURRENT SHELL IS COMPLETE
        // ----------------------------------------------------

        const int8_t completedShell =
            currentShell[selectedLayer];

        if (isSourceShellComplete(
                selectedLayer,
                completedShell,
                physicsCounts,
                selected))
        {
            // The completed shell no longer needs to be selected.
            // Expose the next shell towards the outside.
            --currentShell[selectedLayer];


            /*
             * IMPORTANT PHYSICS BEHAVIOUR:
             *
             * When the central shell of a layer is completed,
             * activate the next layer's central shell.
             *
             * Previous layers may still contain outer particles.
             * These particles remain available, but their weights
             * are lower than the newly exposed middle particles.
             *
             * Example:
             *
             *   Row 4 middle LEDs: completed.
             *   Row 4 edge LEDs:   still available, weight 1.
             *   Row 5 middle LED:  newly available, weight 7.
             *
             * Therefore row 5 middle has seven times the relative
             * selection weight of each individual row 4 edge LED.
             */
            if (completedShell ==
                (int8_t)maxDistance[selectedLayer])
            {
                if (activeLayers < SOURCE_PHYSICS_LAYER_COUNT)
                {
                    // Activate the next physical row at its centre.
                    currentShell[activeLayers] =
                        maxDistance[activeLayers];

                    ++activeLayers;
                }
            }
        }
    }


    // --------------------------------------------------------
    // PHASE 6: REMAINING FOUR SOURCE LAYERS
    // --------------------------------------------------------

    // Layers 12..15 keep their previous layer-by-layer behaviour.
    for (uint8_t layer =
             SOURCE_PHYSICS_LAST_LAYER + 1;
         layer < LAYER_TOTAL;
         ++layer)
    {
        const uint8_t count = LAYER_COUNTS[layer];

        uint8_t sourceLayer[MATRIX_SIZE] = {};

        for (uint8_t i = 0; i < count; ++i)
        {
            const uint8_t led = SOURCE_LAYERS[layer][i];

            if (led < 1 || led > 64) {
                return false;
            }

            sourceLayer[i] = rotateSlaveLedCCW(led);
        }

        weightedShuffle(sourceLayer, count);

        for (uint8_t i = 0; i < count; ++i)
        {
            sourceOrder[sourceOffset++] = sourceLayer[i];
        }
    }

    return sourceOffset == GRAIN_TOTAL;
}

bool buildTransferPlan()
{
    uint8_t sourceOffset = 0;
    uint8_t destinationOffset = 0;

    for (uint8_t layer = 0; layer < LAYER_TOTAL; ++layer) {
        const uint8_t count = LAYER_COUNTS[layer];

        uint8_t sourceLayer[MATRIX_SIZE] = {};
        uint8_t destinationLayer[MATRIX_SIZE] = {};

        // Copy the requested source LEDs for this layer.
        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t led = SOURCE_LAYERS[layer][i];

            if (led < 1 || led > 64) {
                return false;
            }

            sourceLayer[i] = rotateSlaveLedCCW(led);
        }

        // Lower-module destination layers expand from its outer bottom tip.
        // Local coordinates: col + row = 14 - layer.
        // Examples:
        //   Layer 0: 128
        //   Layer 1: 120, 127
        //   Layer 2: 112, 119, 126
        const int8_t diagonalSum = 14 - (int8_t)layer;
        uint8_t found = 0;

        for (int8_t row = 0; row < (int8_t)MATRIX_SIZE; ++row) {
            const int8_t col = diagonalSum - row;

            if (col >= 0 && col < (int8_t)MATRIX_SIZE) {
                const uint8_t bottomLed =
                    (uint8_t)(65 + row * 8 + col);

                // Convert global bottom LED number (65..128) to local (1..64),
                // rotate the BOTTOM MASTER pattern anticlockwise,
                // then convert back to global numbering.
                destinationLayer[found++] =
                    (uint8_t)(64 + rotateLedCCW(bottomLed - 64));
            }
        }

        if (found != count) {
            return false;
        }

        /*
        * Independently weighted-shuffle source and destination LEDs.
        *
        * SOURCE:
        * Centre particles in the upper slave have a higher probability
        * of being selected first.
        *
        * DESTINATION:
        * Centre positions in the lower master have a higher probability
        * of being selected first.
        *
        * Both use the SAME probability distribution:
        *
        *   Outer position:       Weight 1
        *   Second position:      Weight 4
        *   Third position:       Weight 7
        *   Centre position:      Weight 10
        *
        * Pairing by index gives randomized one-to-one combinations.
        * No source or destination LED is reused.
        *
        * IMPORTANT:
        * Destination weighting changes WHERE the grain settles.
        * It does NOT change the falling animation path.
        */
        weightedShuffle(sourceLayer, count);
        weightedShuffle(destinationLayer, count);

        for (uint8_t i = 0; i < count; ++i) {
            sourceOrder[sourceOffset++] = sourceLayer[i];
            destinationOrder[destinationOffset++] = destinationLayer[i];
        }
    }

    return sourceOffset == GRAIN_TOTAL &&
           destinationOffset == GRAIN_TOTAL;
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
    (void)buildTransferPlan();

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
