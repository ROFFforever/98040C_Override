#include "util/timer.h"
#include "pros/rtos.hpp"

Timer::Timer(uint32_t time) : period(time) {
    lastTime = pros::millis();
}

uint32_t Timer::getTimeSet() {
    const uint32_t time = pros::millis();
    if (!paused) timeWaited += time - lastTime;
    lastTime = time;
    return period;
}

uint32_t Timer::getTimeLeft() {
    const uint32_t time = pros::millis();
    if (!paused) timeWaited += time - lastTime;
    lastTime = time;
    const int delta = period - timeWaited;
    return (delta > 0) ? delta : 0;
}

uint32_t Timer::getTimePassed() {
    const uint32_t time = pros::millis();
    if (!paused) timeWaited += time - lastTime;
    lastTime = time;
    return timeWaited;
}

bool Timer::isDone() {
    const uint32_t time = pros::millis();
    if (!paused) timeWaited += time - lastTime;
    lastTime = time;
    const int delta = period - timeWaited;
    return delta <= 0;
}

bool Timer::isPaused() {
    const uint32_t time = pros::millis();
    if (!paused) timeWaited += time - lastTime;
    return paused;
}

void Timer::set(uint32_t time) {
    period = time;
    reset();
}

void Timer::reset() {
    timeWaited = 0;
    lastTime = pros::millis();
}

void Timer::pause() {
    if (!paused) lastTime = pros::millis();
    paused = true;
}

void Timer::resume() {
    if (paused) lastTime = pros::millis();
    paused = false;
}
