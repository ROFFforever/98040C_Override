#include "Autons/OdomAccuracyTest.h"
#include "Commands/WaitCommand.h"
#include "Commands/Tuning/RawDrive.h"
#include "Commands/Tuning/RawTurn.h"

Sequence* odom_accuracy_test(drivetrain* chassis, double sideLength, int driveVoltageMv, int turnVoltageMv) {
    return new Sequence({
        new RawDrive(chassis, sideLength, false, driveVoltageMv),
        new WaitCommand(300),
        new RawDrive(chassis, sideLength, true, driveVoltageMv),
        new WaitCommand(300),

        new RawDrive(chassis, sideLength, false, driveVoltageMv),
        new WaitCommand(200),
        new RawTurn(chassis, 90, turnVoltageMv),
        new WaitCommand(200),

        new RawDrive(chassis, sideLength, false, driveVoltageMv),
        new WaitCommand(200),
        new RawTurn(chassis, 90, turnVoltageMv),
        new WaitCommand(200),

        new RawDrive(chassis, sideLength, false, driveVoltageMv),
        new WaitCommand(200),
        new RawTurn(chassis, 90, turnVoltageMv),
        new WaitCommand(200),

        new RawDrive(chassis, sideLength, false, driveVoltageMv),
        new WaitCommand(200),
        new RawTurn(chassis, 90, turnVoltageMv),
        new WaitCommand(200),
    });
}
