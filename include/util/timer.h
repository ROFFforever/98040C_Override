#pragma once

#include <cstdint>

/**
 * A non-blocking countdown timer.
 *
 * Java analogy: instead of Thread.sleep() blocking the whole program until
 * time's up, this is like storing System.currentTimeMillis() at the start
 * and checking "has enough time passed yet?" yourself, every tick, without
 * ever blocking. That matters here because a Command's execute()/isFinished()
 * run once per scheduler tick - if you blocked inside one waiting on a
 * delay, you'd freeze every other Subsystem and Command in the program too.
 */
class Timer {
public:
    explicit Timer(uint32_t time);

    uint32_t getTimeSet();
    uint32_t getTimeLeft();
    uint32_t getTimePassed();

    bool isDone();
    bool isPaused();

    void set(uint32_t time);
    void reset();
    void pause();
    void resume();

private:
    uint32_t period;
    uint32_t lastTime;
    uint32_t timeWaited = 0;
    bool paused = false;
};
