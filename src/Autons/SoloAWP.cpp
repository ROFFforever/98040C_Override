#include "Autons/SoloAWP.h"
#include "Commands/Rotate.h"
#include "Commands/LiftMoveToCommand.h"
#include "Commands/WaitCommand.h"
#include "Commands/InstantCommand.h"
#include "Commands/tank_motion_profile.hpp"
#include "CommandScheduler/Parallel.h"
#include "util/mathUtils.h"

Sequence* first_alliance_pin(drivetrain* chassis, piston* piston, Lift* lift){
    //one-time setup: seed odometry to this auton's actual starting position on the
    //field, before the chain below ever runs - not a Sequence step, just a plain call
    chassis->setPose(-6.5, -61.3, degToRad(270));

    return new Sequence({
        chassis->moveForward(15, true, 50),
        new WaitCommand(100),
        chassis->rotate(0, Speed::NORMAL),
        new Parallel({
            lift->moveToCommand(-1200),
            chassis->moveToPoint(-15.4, -44.5, true, Speed::NORMAL, 1500, Units::AUTO)
        }),
        chassis->rotate_to_point(-23.4, -44, true),
        lift->moveToCommand(500),
        new InstantCommand([piston]{ piston->toggle(); }),
        new WaitCommand(300)

    });
}

Sequence* get_second_pin(drivetrain* chassis, piston* piston, Lift* lift){

    return new Sequence({
        chassis->moveForward(6, false, 50),
        new WaitCommand(100),
        new Parallel({
            lift->moveToCommand(300),
            chassis->moveToPoint(-20.589, -27.2, true)
        }),
        new InstantCommand([piston]{ piston->toggle(); }),
        new WaitCommand(200),
        new Parallel({
            lift->moveToCommand(-5000),
            new Sequence({
                new WaitCommand(300),
                chassis->moveToPoint(-25, -37.5, true, Speed::NORMAL, 1.2)
            })
        }),
        chassis->moveForward(2.5, false, 60),
           lift->moveToCommand(100),
        new InstantCommand([piston]{ piston->toggle(); }),
        
        new WaitCommand(100),
        chassis->rotate(0),
        chassis->moveForward(15, false, 70),
          chassis->rotate(270),
        chassis->moveForward(33, false, 60, 1.2),
        chassis->moveForward(10, true, 60, 1),
        chassis->moveForward(26, false, 60, 1),
        
        
    });
}
