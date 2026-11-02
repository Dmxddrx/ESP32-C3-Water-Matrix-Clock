#include "water_animation.h"

#include "max7219.h"
#include "mpu6050.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace WaterAnimation {
namespace {

constexpr uint8_t MATRIX_SIZE = 8;

constexpr uint8_t MATRIX_WIDTH = MATRIX_SIZE * 2;
constexpr uint8_t MATRIX_HEIGHT = MATRIX_SIZE;

constexpr uint16_t CELL_TOTAL =
    MATRIX_WIDTH * MATRIX_HEIGHT;

constexpr uint8_t WATER_TOTAL = 64;

// ------------------------------------------------------------
// WATER ANIMATION TIMING
// ------------------------------------------------------------

// Time between liquid movement updates.
constexpr uint32_t FLOW_STEP_MS = 55;

// Maximum number of water pixels allowed to move during
// one flow update. This controls the apparent liquid speed.
constexpr uint8_t MAX_MOVES_PER_STEP = 4;

// A new orientation must remain stable for this duration
// before the matrix display is rotated.
constexpr uint32_t POSITION_STABLE_MS = 400;

// Accelerometer thresholds matching DisplayManager
// and SandClock orientation detection.
constexpr int16_t GRAVITY_THRESHOLD = 10000;
constexpr int16_t DIAGONAL_THRESHOLD = 9000;

// Minimum downhill movement required before a pixel moves.
// This helps prevent small sensor fluctuations from moving water.
constexpr float MIN_FLOW_PROJECTION = 0.12f;

// ------------------------------------------------------------
// WATER FRAMEBUFFER
// ------------------------------------------------------------

// Logical 16x8 water framebuffer.
//
// water[y][x]:
//     true  = water pixel ON
//     false = empty position
//
// The total number of true cells must always remain 64.
bool water[MATRIX_HEIGHT][MATRIX_WIDTH] = {};

// Animation state.
bool initialized = false;

uint32_t lastFlowStepMs = 0;

// Rotating scan start prevents movement selection from
// always favouring the same framebuffer coordinates.
uint16_t scanCursor = 0;

// ------------------------------------------------------------
// AUTOMATIC WATER ANIMATION ORIENTATION
// ------------------------------------------------------------

// Current complete matrix orientation.
bool upsideDown = false;

// Candidate orientation being checked for stability.
bool candidateUpsideDown = false;
bool orientationCandidateValid = false;

uint32_t candidateOrientationSinceMs = 0;

/*
 * MPU6050 TO MATRIX AXIS MAPPING
 *
 * MPU6050 mounting:
 *
 *     Z axis = Matrix top-to-bottom.
 *     Y axis = Matrix right-to-left.
 *     X axis = Outside the 2D matrix plane.
 *
 * Gravity conversion:
 *
 *     Matrix X = +AY
 *     Matrix Y = -AZ
 *
 * The signs account for the accelerometer measuring
 * specific force opposite to gravity.
 *
 * Reverse a sign only if physical testing shows that
 * the water flows in the opposite direction.
 */
constexpr float SENSOR_Y_TO_MATRIX_X_SIGN = 1.0f;
constexpr float SENSOR_Z_TO_MATRIX_Y_SIGN = -1.0f;

// Minimum usable acceleration projection onto the matrix plane.
constexpr float MIN_IN_PLANE_ACCELERATION = 2000.0f;

float gravityX = 0.0f;
float gravityY = 1.0f;

// ------------------------------------------------------------
// UPDATE WATER ORIENTATION
// ------------------------------------------------------------

/*
 * UPDATE WATER ORIENTATION
 *
 * Uses the same accelerometer thresholds and stability period
 * as DisplayManager and SandClock.
 *
 * Normal position:
 *     AZ < -10000
 *
 * Upside-down position:
 *     AZ > +10000
 *
 * Diagonal position:
 *     AX > 9000 && AY > 9000
 *
 * Sideways and transition positions are ignored.
 *
 * The previously selected display orientation is maintained
 * while the device is moving or held sideways.
 *
 * Returns true when the active orientation changes.
 */
bool updateUpsideDownOrientation(uint32_t nowMs)
{
    const MPU6050::Data &imu = MPU6050::getData();

    if (!imu.valid) {
        return false;
    }

    // Match the diagonal-position detection precedence
    // used by DisplayManager.
    if (imu.axRaw > DIAGONAL_THRESHOLD &&
        imu.ayRaw > DIAGONAL_THRESHOLD)
    {
        orientationCandidateValid = false;
        return false;
    }

    bool detectedUpsideDown;

    // Detect the two main hourglass orientations.
    if (imu.azRaw > GRAVITY_THRESHOLD)
    {
        detectedUpsideDown = true;
    }
    else if (imu.azRaw < -GRAVITY_THRESHOLD)
    {
        detectedUpsideDown = false;
    }
    else
    {
        // Ignore sideways and transition positions.
        orientationCandidateValid = false;
        return false;
    }

    // Start checking a new candidate orientation.
    if (!orientationCandidateValid ||
        detectedUpsideDown != candidateUpsideDown)
    {
        candidateUpsideDown = detectedUpsideDown;
        candidateOrientationSinceMs = nowMs;
        orientationCandidateValid = true;

        return false;
    }

    // Wait until the candidate orientation remains stable.
    if ((uint32_t)(nowMs - candidateOrientationSinceMs) <
        POSITION_STABLE_MS)
    {
        return false;
    }

    // Change the display orientation only when necessary.
    if (upsideDown != candidateUpsideDown)
    {
        upsideDown = candidateUpsideDown;
        return true;
    }

    return false;
}

/*
 * UPDATE GRAVITY DIRECTION
 *
 * Uses the raw MPU6050 accelerometer readings.
 *
 * Physical axis mapping:
 *
 *     MPU Y -> Matrix horizontal direction.
 *     MPU Z -> Matrix vertical direction.
 *
 * Gravity calculation:
 *
 *     gravityX = +AY
 *     gravityY = -AZ
 *
 * MPU X is ignored because it is perpendicular to the
 * 2D matrix plane.
 *
 * The gravity vector is normalized so the movement
 * algorithm can compare horizontal, vertical and
 * diagonal movements.
 *
 * IMPORTANT:
 *     The raw accelerometer already detects the reversal
 *     of gravity when the device is flipped upside down.
 *
 *     Therefore, do NOT manually reverse gravityX/gravityY
 *     using the upsideDown variable.
 *
 *     upsideDown is used only for matrix display rotation.
 */
void updateGravityDirection()
{
    const MPU6050::Data &imu = MPU6050::getData();

    if (!imu.valid) {
        // Keep the last valid gravity direction.
        return;
    }

    // Convert MPU6050 sensor axes into matrix gravity axes.
    float gx =
        (float)imu.ayRaw * SENSOR_Y_TO_MATRIX_X_SIGN;

    float gy =
        (float)imu.azRaw * SENSOR_Z_TO_MATRIX_Y_SIGN;

    // Calculate the in-plane gravity magnitude.
    const float magnitude =
        sqrtf((gx * gx) + (gy * gy));

    // If gravity has almost no projection onto the matrix plane,
    // retain the previous valid direction.
    if (magnitude < MIN_IN_PLANE_ACCELERATION) {
        return;
    }

    // Normalize the gravity vector.
    gx /= magnitude;
    gy /= magnitude;

    gravityX = gx;
    gravityY = gy;
}

// ------------------------------------------------------------
// WATER INITIALIZATION
// ------------------------------------------------------------

/*
 * INITIAL WATER DISTRIBUTION
 *
 * The logical framebuffer contains two 8x8 chambers:
 *
 *     X = 0..7   Slave chamber
 *     X = 8..15  Master chamber
 *
 * Initially, each chamber contains 32 water pixels.
 *
 * Rows 4..7 are ON in both chambers.
 *
 * Initial water count:
 *     16 columns x 4 rows = 64 pixels.
 *
 * Every subsequent movement exchanges one occupied cell
 * with one empty cell, preserving the total water count.
 */
void initializeWater()
{
    memset(water, 0, sizeof(water));

    for (uint8_t y = MATRIX_SIZE / 2;
         y < MATRIX_HEIGHT;
         ++y)
    {
        for (uint8_t x = 0;
             x < MATRIX_WIDTH;
             ++x)
        {
            water[y][x] = true;
        }
    }
}

// ------------------------------------------------------------
// WATER PIXEL ROTATION
// ------------------------------------------------------------

/*
 * SET WATER PIXEL
 *
 * The WaterAnimation uses a logical 16x8 framebuffer:
 *
 *     X = 0..7    Slave module
 *     X = 8..15   Master module
 *
 * Normal orientation:
 *     Draw pixels at their original coordinates.
 *
 * Upside-down orientation:
 *
 *     SLAVE MODULE:
 *         Moves to the opposite module position.
 *         Rotates 90 degrees clockwise.
 *
 *         New X = 8 + 7 - Old Y
 *         New Y = Old X
 *
 *     MASTER MODULE:
 *         Moves to the opposite module position.
 *         Rotates 90 degrees anticlockwise.
 *
 *         New X = Old Y
 *         New Y = 7 - (Old X - 8)
 *
 * This uses the same module transformation as SandClock.
 */
void setWaterPixel(uint8_t x, uint8_t y, uint8_t state)
{
    // Protect against invalid framebuffer coordinates.
    if (x >= MATRIX_WIDTH || y >= MATRIX_HEIGHT) {
        return;
    }

    if (upsideDown)
    {
        // Preserve the original coordinates before transformation.
        const uint8_t oldX = x;
        const uint8_t oldY = y;

        if (oldX < MATRIX_SIZE)
        {
            // ------------------------------------------------
            // SLAVE MODULE
            // Move to the opposite module position.
            // Rotate 90 degrees CLOCKWISE.
            // ------------------------------------------------

            const uint8_t localX = oldX;

            x = (uint8_t)(
                MATRIX_SIZE + (MATRIX_SIZE - 1 - oldY)
            );

            y = localX;
        }
        else
        {
            // ------------------------------------------------
            // MASTER MODULE
            // Move to the opposite module position.
            // Rotate 90 degrees ANTICLOCKWISE.
            // ------------------------------------------------

            const uint8_t localX =
                (uint8_t)(oldX - MATRIX_SIZE);

            x = oldY;

            y = (uint8_t)(
                MATRIX_SIZE - 1 - localX
            );
        }
    }

    MAX7219_SetPixel(x, y, state);
}

// ------------------------------------------------------------
// FRAMEBUFFER RENDERING
// ------------------------------------------------------------

/*
 * RENDER WATER FRAME
 *
 * Every occupied framebuffer cell is drawn.
 *
 * The framebuffer always contains exactly 64 water pixels.
 */
void render()
{
    MAX7219_Clear();

    for (uint8_t y = 0; y < MATRIX_HEIGHT; ++y)
    {
        for (uint8_t x = 0; x < MATRIX_WIDTH; ++x)
        {
            if (water[y][x])
            {
                setWaterPixel(x, y, 1);
            }
        }
    }

    MAX7219_Update();
}

// ------------------------------------------------------------
// WATER MOVEMENT
// ------------------------------------------------------------

/*
 * TRY TO MOVE ONE WATER PIXEL
 *
 * Each water pixel can move to one of its eight neighbouring
 * positions:
 *
 *     NW   N   NE
 *      \   |   /
 *       \  |  /
 *     W -- P -- E
 *       /  |  \
 *      /   |   \
 *     SW   S   SE
 *
 * A movement is allowed only when:
 *
 * 1. The starting position contains water.
 * 2. The destination is inside the 16x8 framebuffer.
 * 3. The destination is currently empty.
 * 4. The movement is in the gravity direction.
 *
 * The movement with the strongest downhill projection
 * is selected.
 *
 * Exactly one water pixel moves during this function.
 *
 * Returns true when a pixel successfully moves.
 */
bool moveOneWaterPixel()
{
    constexpr int8_t NEIGHBOUR_X[8] = {
        0,  1, -1,
        1, -1,
        0,  1, -1
    };

    constexpr int8_t NEIGHBOUR_Y[8] = {
        1,  1,  1,
        0,  0,
       -1, -1, -1
    };

    float bestProjection = MIN_FLOW_PROJECTION;

    int8_t bestFromX = -1;
    int8_t bestFromY = -1;
    int8_t bestToX = -1;
    int8_t bestToY = -1;

    // Rotate the scan start to avoid consistently favouring
    // the same framebuffer positions when several movements
    // have the same gravity projection.
    const uint16_t scanStart = scanCursor;

    scanCursor = (uint16_t)(
        (scanCursor + 17) % CELL_TOTAL
    );

    // Search the complete 16x8 framebuffer.
    for (uint16_t scan = 0;
         scan < CELL_TOTAL;
         ++scan)
    {
        const uint16_t index =
            (uint16_t)((scanStart + scan) % CELL_TOTAL);

        const uint8_t fromX =
            (uint8_t)(index % MATRIX_WIDTH);

        const uint8_t fromY =
            (uint8_t)(index / MATRIX_WIDTH);

        // Only occupied cells can move.
        if (!water[fromY][fromX]) {
            continue;
        }

        // Check the eight neighbouring cells.
        for (uint8_t direction = 0;
             direction < 8;
             ++direction)
        {
            const int8_t dx = NEIGHBOUR_X[direction];
            const int8_t dy = NEIGHBOUR_Y[direction];

            const int16_t destinationX =
                (int16_t)fromX + dx;

            const int16_t destinationY =
                (int16_t)fromY + dy;

            // Destination must remain inside the framebuffer.
            if (destinationX < 0 ||
                destinationX >= MATRIX_WIDTH ||
                destinationY < 0 ||
                destinationY >= MATRIX_HEIGHT)
            {
                continue;
            }

            const uint8_t toX =
                (uint8_t)destinationX;

            const uint8_t toY =
                (uint8_t)destinationY;

            // Water cannot move into an occupied cell.
            if (water[toY][toX]) {
                continue;
            }

            // Calculate the movement projection onto gravity.
            const float projection =
                (dx * gravityX) + (dy * gravityY);

            // Reject movements that are sideways or uphill.
            if (projection <= bestProjection) {
                continue;
            }

            bestProjection = projection;

            bestFromX = (int8_t)fromX;
            bestFromY = (int8_t)fromY;

            bestToX = (int8_t)toX;
            bestToY = (int8_t)toY;
        }
    }

    // No valid downhill movement was found.
    if (bestFromX < 0 ||
        bestFromY < 0 ||
        bestToX < 0 ||
        bestToY < 0)
    {
        return false;
    }

    // Move one water pixel without changing the total count.
    water[(uint8_t)bestFromY][(uint8_t)bestFromX] = false;
    water[(uint8_t)bestToY][(uint8_t)bestToX] = true;

    return true;
}

// ------------------------------------------------------------
// WATER SIMULATION STEP
// ------------------------------------------------------------

/*
 * UPDATE WATER FLOW
 *
 * Refresh the gravity vector from the MPU6050.
 *
 * Move up to MAX_MOVES_PER_STEP water pixels in the
 * detected gravity direction.
 *
 * Every movement exchanges one occupied cell with one
 * empty cell. Therefore, the total number of water pixels
 * remains exactly 64.
 *
 * Returns true when at least one water pixel moves.
 */
bool updateWaterFlow()
{
    updateGravityDirection();

    bool waterMoved = false;

    for (uint8_t i = 0;
         i < MAX_MOVES_PER_STEP;
         ++i)
    {
        if (!moveOneWaterPixel()) {
            break;
        }

        waterMoved = true;
    }

    return waterMoved;
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

    // Detect the current device orientation before rendering.
    updateUpsideDownOrientation(nowMs);

    // Create the initial 64-pixel water distribution.
    initializeWater();

    // Update gravity using the current MPU6050 reading.
    updateGravityDirection();

    scanCursor = 0;
    lastFlowStepMs = nowMs;

    // Initial frame: 32 water pixels in each chamber.
    render();
}

void update(uint32_t nowMs)
{
    if (!initialized) {
        begin();
        return;
    }

    // Detect orientation changes continuously.
    // When the device becomes upside down or returns to normal,
    // redraw the complete water framebuffer using the new
    // slave/master module rotation.
    if (updateUpsideDownOrientation(nowMs))
    {
        updateGravityDirection();
        render();
    }

    // Run the water simulation at a controlled interval.
    if ((uint32_t)(nowMs - lastFlowStepMs) >= FLOW_STEP_MS)
    {
        lastFlowStepMs = nowMs;

        // Render only when at least one water pixel moves.
        if (updateWaterFlow())
        {
            render();
        }
    }
}

} // namespace WaterAnimation
