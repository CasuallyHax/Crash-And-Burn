#include "robot/monte.hpp"
#include "globals.hpp"
#include "robot/auton.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

namespace MCL {

// ── Field obstacles (AABB, inches) ───────────────────────────────────────────
//  All Pushback field structures (middle goal, long goal supports, corner
//  cylinders/match loaders) have air gaps or complex shapes that make AABB
//  modelling unreliable — sensors read through gaps inconsistently, which
//  would penalize correct particles in raycast(). Masking is handled entirely
//  by danger zones. This array is intentionally empty but kept for future use
//  if solid obstacles are identified.
static const AABB OBSTACLES[] = {};
static constexpr int N_OBS = sizeof(OBSTACLES) / sizeof(AABB);

// ── Danger zones ──────────────────────────────────────────────────────────────
//  Circular regions on the field. When the robot's estimated position falls
//  inside one, the named sensor is masked for that entire MCL cycle and
//  re-enabled automatically once the robot leaves the zone.
//
//  Convention: place zone centers on the side of each obstacle that faces the
//  robot when the sensor ray would hit that obstacle (not the far wall).
//  Tune radii in testing — start generous and tighten as needed.
static const DangerZone DANGER_ZONES[] = {
    // ── Middle goal (footprint ±9" from center) ────────────────────────────
    // Robot south of goal → North sensor faces goal
    {  0.0f, -16.0f, 18.0f, SensorId::NORTH }, // TODO: TUNE radius
    // Robot north of goal → South sensor faces goal
    {  0.0f,  16.0f, 18.0f, SensorId::SOUTH }, // TODO: TUNE radius
    // Robot east of goal  → West sensor faces goal
    { 16.0f,   0.0f, 18.0f, SensorId::WEST  }, // TODO: TUNE radius
    // Robot west of goal  → East sensor faces goal
    {-16.0f,   0.0f, 18.0f, SensorId::EAST  }, // TODO: TUNE radius

    // ── Long goal supports (y ≈ ±47") ─────────────────────────────────────
    // Top-left (-22.5, 47) and top-right (22.5, 47): robot below → North masked
    {-22.5f,  38.0f, 14.0f, SensorId::NORTH }, // TODO: TUNE radius
    { 22.5f,  38.0f, 14.0f, SensorId::NORTH }, // TODO: TUNE radius
    // Bottom-left (-22.5, -47) and bottom-right (22.5, -47): robot above → South masked
    {-22.5f, -38.0f, 14.0f, SensorId::SOUTH }, // TODO: TUNE radius
    { 22.5f, -38.0f, 14.0f, SensorId::SOUTH }, // TODO: TUNE radius
    // Robot to the right of left supports → West masked
    {-32.0f,  47.0f, 14.0f, SensorId::WEST  }, // top-left support     // TODO: TUNE radius & center
    {-32.0f, -47.0f, 14.0f, SensorId::WEST  }, // bottom-left support  // TODO: TUNE radius & center
    // Robot to the left of right supports → East masked
    { 32.0f,  47.0f, 14.0f, SensorId::EAST  }, // top-right support    // TODO: TUNE radius & center
    { 32.0f, -47.0f, 14.0f, SensorId::EAST  }, // bottom-right support // TODO: TUNE radius & center

    // ── Corner cylinders (hugging east/west walls, y ≈ ±46.5") ───────────
    // Left-side cylinders: robot to the east → West sensor masked
    {-58.0f, -46.5f, 16.0f, SensorId::WEST  }, // cylinder bottom-left  // TODO: TUNE radius & center
    {-58.0f,  46.5f, 16.0f, SensorId::WEST  }, // cylinder top-left     // TODO: TUNE radius & center
    // Right-side cylinders: robot to the west → East sensor masked
    { 58.0f, -46.5f, 16.0f, SensorId::EAST  }, // cylinder bottom-right // TODO: TUNE radius & center
    { 58.0f,  46.5f, 16.0f, SensorId::EAST  }, // cylinder top-right    // TODO: TUNE radius & center
    // All cylinders: robot approaching from south → North sensor masked
    {-67.5f, -38.0f, 14.0f, SensorId::NORTH }, // cylinder bottom-left  // TODO: TUNE radius & center
    { 67.5f, -38.0f, 14.0f, SensorId::NORTH }, // cylinder bottom-right // TODO: TUNE radius & center
    {-67.5f,  38.0f, 14.0f, SensorId::NORTH }, // cylinder top-left     // TODO: TUNE radius & center
    { 67.5f,  38.0f, 14.0f, SensorId::NORTH }, // cylinder top-right    // TODO: TUNE radius & center
    // All cylinders: robot approaching from north → South sensor masked
    {-67.5f, -55.0f, 14.0f, SensorId::SOUTH }, // cylinder bottom-left  // TODO: TUNE radius & center
    { 67.5f, -55.0f, 14.0f, SensorId::SOUTH }, // cylinder bottom-right // TODO: TUNE radius & center
    {-67.5f,  55.0f, 14.0f, SensorId::SOUTH }, // cylinder top-left     // TODO: TUNE radius & center
    { 67.5f,  55.0f, 14.0f, SensorId::SOUTH }, // cylinder top-right    // TODO: TUNE radius & center

    // Note: match loaders are the same structures as the corner cylinders above.
    // Their danger zones are already covered by the cylinder zones.
};
static constexpr int N_ZONES = sizeof(DANGER_ZONES) / sizeof(DangerZone);

// ── Global instance ───────────────────────────────────────────────────────────
Localizer localizer;

// ── Minimal PRNG (Box-Muller) ─────────────────────────────────────────────────
//  Avoids heap allocations from std::mt19937 / std::normal_distribution.
static uint32_t lcgState = 0xDEADBEEF;

static float lcgUniform() {
    lcgState = lcgState * 1664525u + 1013904223u;
    return (lcgState >> 8) * (1.0f / (1 << 24));
}

static float randn() {
    // Box-Muller transform — returns one N(0,1) sample per call.
    static bool   hasSpare = false;
    static float  spare;
    if (hasSpare) { hasSpare = false; return spare; }
    float u, v, s;
    do {
        u = lcgUniform() * 2.0f - 1.0f;
        v = lcgUniform() * 2.0f - 1.0f;
        s = u*u + v*v;
    } while (s >= 1.0f || s < 1e-9f);
    float mul = std::sqrt(-2.0f * std::log(s) / s);
    spare    = v * mul;
    hasSpare = true;
    return u * mul;
}

// ── Constructor ───────────────────────────────────────────────────────────────
Localizer::Localizer()
    : lastOdomPose(0, 0, 0), estimate(0, 0, 0) {
    float w = 1.0f / NUM_PARTICLES;
    for (auto& p : particles) p = {0, 0, 0, w};
}

// ── init ──────────────────────────────────────────────────────────────────────
void Localizer::init(float x, float y, float theta,
                     float spreadXY, float spreadTheta) {
    lcgState ^= (uint32_t)pros::millis(); // re-seed so every init differs
    mtx.take(TIMEOUT_MAX);
    float w = 1.0f / NUM_PARTICLES;
    for (auto& p : particles) {
        p.x      = x     + randn() * spreadXY;
        p.y      = y     + randn() * spreadXY;
        p.theta  = theta + randn() * spreadTheta;
        p.weight = w;
    }
    lastOdomPose = {x, y, theta};
    estimate     = {x, y, theta};
    mtx.give();
}

// ── raycast ───────────────────────────────────────────────────────────────────
//  Returns the distance (inches) from sensor position (sx, sy) along unit
//  direction (dx, dy) to the nearest wall or obstacle.
float Localizer::raycast(float sx, float sy, float dx, float dy) const {
    float minT = 1e9f;

    // Perimeter walls
    if (std::abs(dx) > 1e-6f) {
        float t = (dx > 0 ? FIELD_HALF - sx : -FIELD_HALF - sx) / dx;
        if (t > 1e-4f && t < minT) minT = t;
    }
    if (std::abs(dy) > 1e-6f) {
        float t = (dy > 0 ? FIELD_HALF - sy : -FIELD_HALF - sy) / dy;
        if (t > 1e-4f && t < minT) minT = t;
    }

    // AABB obstacles — slab intersection test
    for (int i = 0; i < N_OBS; ++i) {
        const AABB& o = OBSTACLES[i];
        float txMin, txMax, tyMin, tyMax;

        if (std::abs(dx) > 1e-6f) {
            txMin = (o.x1 - sx) / dx;
            txMax = (o.x2 - sx) / dx;
            if (txMin > txMax) std::swap(txMin, txMax);
        } else {
            if (sx <= o.x1 || sx >= o.x2) continue; // parallel & outside
            txMin = -1e9f; txMax = 1e9f;
        }

        if (std::abs(dy) > 1e-6f) {
            tyMin = (o.y1 - sy) / dy;
            tyMax = (o.y2 - sy) / dy;
            if (tyMin > tyMax) std::swap(tyMin, tyMax);
        } else {
            if (sy <= o.y1 || sy >= o.y2) continue; // parallel & outside
            tyMin = -1e9f; tyMax = 1e9f;
        }

        float tEnter = std::max(txMin, tyMin);
        float tExit  = std::min(txMax, tyMax);
        if (tExit >= tEnter && tEnter > 1e-4f && tEnter < minT)
            minT = tEnter;
    }

    return minT;
}

// ── raycastHitsObstacle ───────────────────────────────────────────────────────
//  Returns true if an obstacle is hit before the nearest wall.
//  Used for auto-masking: if the sensor ray from the current estimated pose
//  hits a goal structure, that reading is unreliable and gets skipped.
bool Localizer::raycastHitsObstacle(float sx, float sy, float dx, float dy) const {
    // Nearest wall distance
    float wallT = 1e9f;
    if (std::abs(dx) > 1e-6f) {
        float t = (dx > 0 ? FIELD_HALF - sx : -FIELD_HALF - sx) / dx;
        if (t > 1e-4f && t < wallT) wallT = t;
    }
    if (std::abs(dy) > 1e-6f) {
        float t = (dy > 0 ? FIELD_HALF - sy : -FIELD_HALF - sy) / dy;
        if (t > 1e-4f && t < wallT) wallT = t;
    }

    // Check if any obstacle is closer than the nearest wall
    for (int i = 0; i < N_OBS; ++i) {
        const AABB& o = OBSTACLES[i];
        float txMin, txMax, tyMin, tyMax;

        if (std::abs(dx) > 1e-6f) {
            txMin = (o.x1 - sx) / dx;
            txMax = (o.x2 - sx) / dx;
            if (txMin > txMax) std::swap(txMin, txMax);
        } else {
            if (sx <= o.x1 || sx >= o.x2) continue;
            txMin = -1e9f; txMax = 1e9f;
        }

        if (std::abs(dy) > 1e-6f) {
            tyMin = (o.y1 - sy) / dy;
            tyMax = (o.y2 - sy) / dy;
            if (tyMin > tyMax) std::swap(tyMin, tyMax);
        } else {
            if (sy <= o.y1 || sy >= o.y2) continue;
            tyMin = -1e9f; tyMax = 1e9f;
        }

        float tEnter = std::max(txMin, tyMin);
        float tExit  = std::min(txMax, tyMax);
        if (tExit >= tEnter && tEnter > 1e-4f && tEnter < wallT)
            return true;
    }
    return false;
}

// ── gaussLikelihood ───────────────────────────────────────────────────────────
float Localizer::gaussLikelihood(float measured, float expected) const {
    float d = measured - expected;
    return std::exp(-0.5f * d * d / (SENSOR_SIGMA * SENSOR_SIGMA));
}

// ── motionUpdate ──────────────────────────────────────────────────────────────
//  Propagate each particle by the odometry delta, plus Gaussian noise scaled
//  by how much the robot actually moved.
void Localizer::motionUpdate(float dx, float dy, float dtheta) {
    float dist     = std::sqrt(dx*dx + dy*dy);
    float posNoise = POS_NOISE + dist * 0.05f;          // TODO: TUNE scaling factor (0.05f) for your drivetrain slip
    float angNoise = THETA_NOISE + std::abs(dtheta) * 0.02f; // TODO: TUNE scaling factor (0.02f) for your heading drift

    for (auto& p : particles) {
        p.x     += dx     + randn() * posNoise;
        p.y     += dy     + randn() * posNoise;
        p.theta += dtheta + randn() * angNoise;
        p.x = std::max(-FIELD_HALF, std::min(FIELD_HALF, p.x));
        p.y = std::max(-FIELD_HALF, std::min(FIELD_HALF, p.y));
    }
}

// ── sensorUpdate ─────────────────────────────────────────────────────────────
//  Weight each particle by how well its expected sensor readings match reality.
//
//  LemLib heading θ (0° = +Y, clockwise):
//    forward  = ( sinθ,  cosθ )
//    right    = ( cosθ, -sinθ )
//    backward = (-sinθ, -cosθ )
//    left     = (-cosθ,  sinθ )
void Localizer::sensorUpdate() {
    float mN = dNorth.get_distance() / 25.4f;
    float mS = dSouth.get_distance() / 25.4f;
    float mE = dEast.get_distance()  / 25.4f;
    float mW = dWest.get_distance()  / 25.4f;

    // Auto-mask: use the previous estimate to check whether each sensor's ray
    // hits an obstacle before a wall. If so, the reading is unreliable and
    // gets skipped for this cycle.
    float etr  = estimate.theta * (float)M_PI / 180.0f;
    float eSin = std::sin(etr), eCos = std::cos(etr);

    bool useN = northEnabled && !northZoneMasked && mN < MAX_RANGE_IN &&
                !raycastHitsObstacle(estimate.x + eSin * nDistCenter,
                                     estimate.y + eCos * nDistCenter,
                                     eSin, eCos);
    bool useS = southEnabled && !southZoneMasked && mS < MAX_RANGE_IN &&
                !raycastHitsObstacle(estimate.x - eSin * sDistCenter,
                                     estimate.y - eCos * sDistCenter,
                                    -eSin, -eCos);
    bool useE = eastEnabled  && !eastZoneMasked  && mE < MAX_RANGE_IN &&
                !raycastHitsObstacle(estimate.x + eCos * eDistCenter,
                                     estimate.y - eSin * eDistCenter,
                                     eCos, -eSin);
    bool useW = westEnabled  && !westZoneMasked  && mW < MAX_RANGE_IN &&
                !raycastHitsObstacle(estimate.x - eCos * wDistCenter,
                                     estimate.y + eSin * wDistCenter,
                                    -eCos, eSin);

    float totalW = 0;
    for (auto& p : particles) {
        float tr   = p.theta * (float)M_PI / 180.0f;
        float sinT = std::sin(tr), cosT = std::cos(tr);

        float w = 1.0f;

        if (useN) { // North sensor: mounted nDistCenter inches forward
            float ex = raycast(p.x + sinT * nDistCenter,
                               p.y + cosT * nDistCenter,
                               sinT, cosT);
            w *= gaussLikelihood(mN, ex);
        }
        if (useS) { // South sensor: mounted sDistCenter inches backward
            float ex = raycast(p.x - sinT * sDistCenter,
                               p.y - cosT * sDistCenter,
                              -sinT, -cosT);
            w *= gaussLikelihood(mS, ex);
        }
        if (useE) { // East sensor: mounted eDistCenter inches to the right
            float ex = raycast(p.x + cosT * eDistCenter,
                               p.y - sinT * eDistCenter,
                               cosT, -sinT);
            w *= gaussLikelihood(mE, ex);
        }
        if (useW) { // West sensor: mounted wDistCenter inches to the left
            float ex = raycast(p.x - cosT * wDistCenter,
                               p.y + sinT * wDistCenter,
                              -cosT, sinT);
            w *= gaussLikelihood(mW, ex);
        }

        p.weight = w;
        totalW  += w;
    }

    if (totalW < 1e-9f) {
        // Weight collapse: scatter particles around last known odometry pose
        // so the filter can recover instead of staying degenerate.
        float w = 1.0f / NUM_PARTICLES;
        for (auto& p : particles) {
            p.x      = lastOdomPose.x     + randn() * 5.0f;
            p.y      = lastOdomPose.y     + randn() * 5.0f;
            p.theta  = lastOdomPose.theta + randn() * 3.0f;
            p.weight = w;
        }
        return;
    }

    for (auto& p : particles) p.weight /= totalW;
}

// ── resample (low-variance) ───────────────────────────────────────────────────
void Localizer::resample() {
    std::array<Particle, NUM_PARTICLES> next;
    float step = 1.0f / NUM_PARTICLES;
    float r    = lcgUniform() * step;
    float c    = particles[0].weight;
    int   idx  = 0;

    for (int m = 0; m < NUM_PARTICLES; ++m) {
        float U = r + (float)m * step;
        while (U > c && idx < NUM_PARTICLES - 1) {
            ++idx;
            c += particles[idx].weight;
        }
        next[m]        = particles[idx];
        next[m].weight = step;
    }
    particles = next;
}

// ── computeEstimate ───────────────────────────────────────────────────────────
//  Weighted mean position; circular mean for heading to handle 0°/360° wrap.
void Localizer::computeEstimate() {
    float sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
    for (const auto& p : particles) {
        sumX   += p.weight * p.x;
        sumY   += p.weight * p.y;
        float tr = p.theta * (float)M_PI / 180.0f;
        sumSin += p.weight * std::sin(tr);
        sumCos += p.weight * std::cos(tr);
    }
    estimate = {sumX, sumY,
                std::atan2(sumSin, sumCos) * 180.0f / (float)M_PI};
}

// ── updateDangerZones ─────────────────────────────────────────────────────────
//  Checks each danger zone against the current estimate and sets the zone-mask
//  booleans. Called once per update cycle before sensorUpdate().
void Localizer::updateDangerZones() {
    northZoneMasked = southZoneMasked = eastZoneMasked = westZoneMasked = false;
    for (int i = 0; i < N_ZONES; ++i) {
        const DangerZone& z = DANGER_ZONES[i];
        float dx = estimate.x - z.cx;
        float dy = estimate.y - z.cy;
        if (dx*dx + dy*dy < z.radius * z.radius) {
            switch (z.sensor) {
                case SensorId::NORTH: northZoneMasked = true; break;
                case SensorId::SOUTH: southZoneMasked = true; break;
                case SensorId::EAST:  eastZoneMasked  = true; break;
                case SensorId::WEST:  westZoneMasked  = true; break;
            }
        }
    }
}

// ── update ────────────────────────────────────────────────────────────────────
void Localizer::update() {
    mtx.take(TIMEOUT_MAX);

    lemlib::Pose cur = chassis.getPose();
    float dx     = cur.x     - lastOdomPose.x;
    float dy     = cur.y     - lastOdomPose.y;
    float dtheta = cur.theta - lastOdomPose.theta;
    while (dtheta >  180) dtheta -= 360;
    while (dtheta < -180) dtheta += 360;
    lastOdomPose = cur;

    motionUpdate(dx, dy, dtheta);
    updateDangerZones();
    sensorUpdate();
    resample();
    computeEstimate();

    // Rate-limited continuous correction: nudge chassis pose toward the MCL
    // estimate by at most MAX_DELTA_XY inches and MAX_DELTA_THETA degrees per
    // cycle. Small enough to never cause a visible jerk mid-path.
    lemlib::Pose correction(0, 0, 0);
    bool doCorrect = autoCorrect;
    if (doCorrect) {
        float cx = estimate.x     - cur.x;
        float cy = estimate.y     - cur.y;
        float ct = estimate.theta - cur.theta;
        while (ct >  180) ct -= 360;
        while (ct < -180) ct += 360;

        auto clampf = [](float v, float limit) {
            return v > limit ? limit : (v < -limit ? -limit : v);
        };
        correction.x     = cur.x     + clampf(cx, MAX_DELTA_XY);
        correction.y     = cur.y     + clampf(cy, MAX_DELTA_XY);
        correction.theta = cur.theta + clampf(ct, MAX_DELTA_THETA);
    }

    mtx.give();

    // Apply outside the mutex so chassis.setPose() doesn't race with getPose().
    if (doCorrect)
        chassis.setPose(correction.x, correction.y, correction.theta);
}

// ── getPose ───────────────────────────────────────────────────────────────────
lemlib::Pose Localizer::getPose() const {
    mtx.take(TIMEOUT_MAX);
    lemlib::Pose p = estimate;
    mtx.give();
    return p;
}

// ── applyCorrection ───────────────────────────────────────────────────────────
void Localizer::applyCorrection() {
    lemlib::Pose p = getPose();
    chassis.setPose(p.x, p.y, p.theta);
}

// ── applyPositionOnly ─────────────────────────────────────────────────────────
void Localizer::applyPositionOnly() {
    lemlib::Pose p = getPose();
    chassis.setPose(p.x, p.y, chassis.getPose().theta);
}

// ── background task ───────────────────────────────────────────────────────────
void taskFn(void*) {
    pros::delay(500); // wait for chassis.calibrate() to finish
    lemlib::Pose s = chassis.getPose();
    localizer.init(s.x, s.y, s.theta);
    while (true) {
        localizer.update();
        pros::delay(50);
    }
}

} // namespace MCL
