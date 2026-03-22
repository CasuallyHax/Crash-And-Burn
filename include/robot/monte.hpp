#pragma once

#include "globals.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include <array>

// ============================================================
//  Monte Carlo Localization (MCL)
//
//  Field: 140" × 140", walls at ±70", origin at center.
//  Heading: LemLib convention — 0° = +Y (north), clockwise.
//
//  Usage:
//    1. Start background task in initialize():
//         pros::Task mclTask(MCL::taskFn);
//
//    2. After every chassis.setPose(...) call, sync particles:
//         MCL::localizer.init(x, y, theta);
//
//  Auto-correction and sensor masking run continuously.
//  To disable auto-correction temporarily (e.g. during a
//  sensitive manoeuvre): MCL::localizer.autoCorrect = false;
//
//  Manual sensor masking still available if needed:
//         MCL::localizer.enableEast(false);
//         // ... move ...
//         MCL::localizer.enableEast(true);
// ============================================================

namespace MCL {

// ── Tuning constants ─────────────────────────────────────────────────────────
static constexpr int   NUM_PARTICLES  = 150;    // TODO: TUNE — reduce if loop time exceeds 50ms
static constexpr float FIELD_HALF     = 70.0f;  // inches, wall to center
static constexpr float MAX_RANGE_IN   = 72.0f;  // TODO: TUNE — field half is 70", 72" gives small buffer for offsets
static constexpr float SENSOR_SIGMA   = 1.5f;   // TODO: TUNE — measure actual sensor variance against a wall
static constexpr float POS_NOISE      = 0.25f;  // TODO: TUNE — increase if particles diverge during fast movement
static constexpr float THETA_NOISE    = 0.4f;   // TODO: TUNE — increase if heading drifts under fast rotation
static constexpr float MAX_DELTA_XY   = 0.3f;   // TODO: TUNE — reduce if correction causes jerky movement
static constexpr float MAX_DELTA_THETA= 0.5f;   // TODO: TUNE — reduce if correction causes jerky rotation

// ── Internal types ───────────────────────────────────────────────────────────
struct Particle { float x, y, theta, weight; };
struct AABB     { float x1, y1, x2, y2; };

enum class SensorId { NORTH, SOUTH, EAST, WEST };

// A circular region on the field. When the robot's estimated position falls
// inside it, the named sensor is automatically masked for that MCL cycle.
struct DangerZone {
    float    cx, cy, radius;  // center and radius in inches
    SensorId sensor;          // sensor to disable when inside
};

// ── Main class ───────────────────────────────────────────────────────────────
class Localizer {
public:
    Localizer();

    // Scatter particles around a known pose.
    // spreadXY in inches, spreadTheta in degrees.
    void init(float x, float y, float theta,
              float spreadXY = 2.0f, float spreadTheta = 1.5f);

    // One MCL iteration — called automatically by taskFn every 50 ms.
    void update();

    // Thread-safe read of the current MCL pose estimate.
    lemlib::Pose getPose() const;

    // Snap the chassis pose to the MCL estimate (x, y, theta).
    void applyCorrection();

    // Snap only x/y to the MCL estimate; keeps the odometry heading.
    void applyPositionOnly();

    // When true (default), update() applies a rate-limited correction to the
    // chassis pose every cycle. Set false to pause corrections temporarily.
    bool autoCorrect{true};

    // Manual overrides — sensor masking based on position is already automatic,
    // but these let you force-disable a sensor regardless of position.
    void enableNorth(bool en) { northEnabled = en; }
    void enableSouth(bool en) { southEnabled = en; }
    void enableEast (bool en) { eastEnabled  = en; }
    void enableWest (bool en) { westEnabled  = en; }

private:
    std::array<Particle, NUM_PARTICLES> particles;
    lemlib::Pose lastOdomPose;
    lemlib::Pose estimate;
    mutable pros::Mutex mtx;

    bool northEnabled{true}, southEnabled{true},
         eastEnabled {true}, westEnabled {true};

    // Set each cycle by updateDangerZones(). Separate from the manual enables
    // so user overrides and auto-masking don't interfere with each other.
    bool northZoneMasked{false}, southZoneMasked{false},
         eastZoneMasked {false}, westZoneMasked {false};

    void updateDangerZones();

    // Returns distance (inches) to nearest wall/obstacle along (dx,dy).
    float raycast(float sx, float sy, float dx, float dy) const;

    // Returns true if an obstacle is hit before a wall along (dx,dy).
    bool raycastHitsObstacle(float sx, float sy, float dx, float dy) const;

    float gaussLikelihood(float measured, float expected) const;

    // Internal update steps — all called with mtx already held.
    void motionUpdate(float dx, float dy, float dtheta);
    void sensorUpdate();
    void resample();
    void computeEstimate();
};

// Global instance (defined in monte.cpp).
extern Localizer localizer;

// Pass this to pros::Task in initialize().
void taskFn(void*);

} // namespace MCL
