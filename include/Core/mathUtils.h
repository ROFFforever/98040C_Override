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
float sanitizeAngle(float angle, bool radians = true);

/**
 * Shortest signed angular distance to turn from `position` to reach `target`,
 * handling wraparound correctly (e.g. going from 350 degrees to 10 degrees
 * is +20, not -340).
 */
float angleError(float target, float position, bool radians = true);

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
