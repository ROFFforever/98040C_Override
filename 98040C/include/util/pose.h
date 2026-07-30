#pragma once

/**
 * A robot's position and heading in 2D space: x, y, theta.
 *
 * Convention: STANDARD math frame (unit circle), NOT compass style.
 * theta is in radians, counterclockwise is positive, and theta = 0 means
 * facing along global +X. The VEX IMU reports clockwise-positive degrees
 * instead - drivetrain::getAngle() flips it at the sensor boundary, so
 * everything touching a Pose can assume this convention. That also makes
 * theta directly consistent with what atan2 (used in angle()) and rotate()
 * below already assume, so all the geometry here is self-consistent.
 * theta is unbounded (keeps growing past 2pi on full spins); wrap it with
 * sanitizeAngle() only when displaying or comparing headings.
 *
 * This is the actual "answer" an odometry system computes every tick: given
 * how far the wheels rolled and what the IMU reads, where is the robot on
 * the field right now? Java analogy: a small immutable-style data class,
 * like a record, that also happens to have a few geometry helper methods.
 */
class Pose {
public:
    float x;
    float y;
    float theta;

    Pose(float x, float y, float theta = 0);

    Pose();

    // adds two poses component-wise; heading is kept from this pose, not other
    Pose operator+(const Pose& other) const;
    // subtracts two poses component-wise; heading is kept from this pose, not other
    Pose operator-(const Pose& other) const;

    // straight-line distance between this pose and other, ignoring heading
    float distance(const Pose& other) const;
    // angle (radians) from this pose toward other, ignoring heading
    float angle(const Pose& other) const;
    // this pose's x/y rotated by `angle` radians about the origin; heading is kept
    Pose rotate(float angle) const;
};
