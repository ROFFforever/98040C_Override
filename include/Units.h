#pragma once

// A grab-bag of robot-wide constants. Java analogy: this is exactly like a
// class full of `public static final` fields - you never make a Units
// object, you just read the fields off the class name directly.
class Units {
public:
    static constexpr double WHEEL_325 = 3.25;
    static constexpr int ERROR = -1;
    static constexpr int MISSING_SENSORS = -2;

    // every drive motor cartridge on this robot is blue (600rpm internal speed)
    static constexpr double CARTRIDGE_RPM = 600.0;

    static constexpr double RPM_450 = 450.0;
    static constexpr double RPM_360 = 360.0;

    

};
