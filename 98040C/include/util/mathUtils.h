#pragma once

#include <cmath>
#include <vector>

// radians -> degrees
constexpr float radToDeg(float rad) { return rad * 180.0f / M_PI; }
// degrees -> radians
constexpr float degToRad(float deg) { return deg * M_PI / 180.0f; }

/**
 * Wraps an angle into a standard positive range: [0, 2pi) if radians, else
 * [0, 360). Sensor headings (e.g. from the IMU) and target headings need to
 * be in the same range before you can compare them meaningfully.
 */
float sanitizeAngle(float angle, bool radians = false);

/**
 * Shortest signed difference between two angles, returned as `x - y` wrapped
 * into [-pi, pi] radians. Ported from Echo's `angleDifference`.
 *
 * This is the workhorse for odometry: to find how far the robot's heading
 * turned during one tick you take angleDifference(currentHeading,
 * previousHeading), and it stays correct even when the heading rolls across
 * the 0 / 2pi seam (e.g. 350 deg -> 10 deg reads as +20, not -340).
 *
 * Takes radians. The IMU itself reports clockwise-positive degrees, so flip
 * the sign and convert once, right where you read the sensor (see
 * drivetrain::getAngle()) - that way neither degrees nor the backwards sign
 * convention ever leak into the rest of the odometry math.
 *
 * Note: this is the SAME math as angleError() above. angleError is phrased for
 * control loops (how far is `position` from `target`) and can flip between
 * radians/degrees via its `radians` flag; angleDifference is the general "how
 * far apart are these two headings" phrasing Echo uses in odom, radians-only.
 */
float angleDifference(float current, float past);

/**
 * Sign of a value: -1 if negative, 1 otherwise (0 counts as positive, by
 * convention). Java analogy: like Math.signum(), but this codebase doesn't
 * special-case 0 the way Math.signum() does.
 */
template <typename T>
constexpr T sgn(T value) {
    return value < 0 ? -1 : 1;
}

/**
 * Caps how far `current` is allowed to move toward `target` in one call.
 * Pass 0 for maxChange to disable the cap. Useful for smoothing motor output
 * so it doesn't jump instantly (e.g. limiting acceleration in a drive command).
 */
float slew(float target, float current, float maxChange);

// average of a list of values; returns 0 for an empty list
float avg(const std::vector<float>& values);
