#include "util/OdomScreen.h"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// raw inches rolled by an odom wheel, read straight from the sensor so this
// doesn't touch odom_wheel's prev/current values that update_pos() relies on
static double wheelInches(odom_wheel* w) {
    if (w == nullptr || w->odom_sensor == nullptr) return NAN;
    return w->odom_sensor->get_position() * w->wheel_diameter * M_PI / 36000.0;
}

void OdomScreen::start() {
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::white);
    // below default priority so control code always wins; ~15 Hz is plenty to glance at
    pros::Task([this] {
        while (true) {
            draw();
            pros::delay(66);
        }
    }, TASK_PRIORITY_DEFAULT - 2, TASK_STACK_DEPTH_DEFAULT, "OdomScreen");
}

void OdomScreen::draw() {
    Pose p = chassis->gpos();
    // rows placed by pixel (x, y) so they never overlap; ROW = gap between rows.
    // trailing spaces wipe leftover characters when a number gets shorter
    const int X = 20, TOP = 20, ROW = 35;
    pros::screen::print(pros::E_TEXT_MEDIUM, X, TOP + 0 * ROW, "X:     %7.2f in    ", p.x);
    pros::screen::print(pros::E_TEXT_MEDIUM, X, TOP + 1 * ROW, "Y:     %7.2f in    ", p.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, X, TOP + 2 * ROW, "H:     %7.2f deg   ", p.theta * 180.0 / M_PI);
    pros::screen::print(pros::E_TEXT_MEDIUM, X, TOP + 3 * ROW, "Vert:  %7.2f in    ", wheelInches(chassis->vert_odom));
    pros::screen::print(pros::E_TEXT_MEDIUM, X, TOP + 4 * ROW, "Horiz: %7.2f in    ", wheelInches(chassis->horiz_odom));
    // motor-encoder distance (left/right side), right column - compare against Vert to check wheelRPM
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, TOP + 3 * ROW, "MotL: %7.2f in    ", chassis->getLeftDistance());
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, TOP + 4 * ROW, "MotR: %7.2f in    ", chassis->getRightDistance());
    if (liftAngle) pros::screen::print(pros::E_TEXT_MEDIUM, X, TOP + 5 * ROW, "Lift:  %7.1f deg   ", liftAngle());
}
