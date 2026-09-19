#include "WallLocalization.h"
#include "util/mathUtils.h"
#include "pros/error.h"
#include "Telemetry/telemetry.h"
#include <cmath>
#include <format>

namespace {
const char* sideToStr(WallSensor::Side side){
    switch(side){
        case WallSensor::Side::BACK:  return "BACK";
        case WallSensor::Side::LEFT:  return "LEFT";
        case WallSensor::Side::FRONT: return "FRONT";
        case WallSensor::Side::RIGHT: return "RIGHT";
    }
    return "UNKNOWN";
}
}

//Regular command boilerplate stuff:

void WallLocalization::initialize(){ //nothing to do here yet

}

bool WallLocalization::isFinished(){
    return false; //NEVER STOP >:) 
}

void WallLocalization::execute(){
    reset_pose();
}

void WallLocalization::end(bool interrupted){

}

std::vector<Subsystem*> WallLocalization::getRequirements(){
    return {}; //we don't actually need any hardware to send power to 
}
// My thought process for reset_pose():

// 1.) I need a way to determine which distance sensor to use 
// for resetting x and y. I can do this by checking which robot side
// is currently facing "straight"(in the positive Y direction) and
// then calculating the right sensor based on coords and whether 
// x or y should be reset.

// 2.) Second, I need to create a function that can report a distance 
// from the robot pose to the wall, with a perpendicular angle.
// I derived a math equation from some geometry, accounting for
// both global heading and distance sensor offsets. I'll 
// encapsulate that into a function so I can easily call it.

// 3.) Lastly, I need to actually reset the pose, or at least bias it to some 
// degree with this new distance sensor data. I'll add a bias_rate float
// which determines how much the distance sensor readings influence
// pose. And next, I'll only reset if I'm confident about __ things:

//a.) We shouldn't even scan if the robot is near the extrememties of the 45 degree
// bucket from a cardinal angle; It's just way too risky and unpredictable.
// an allowed range should probably only deviate like 20 to 25 degrees 
// from a cardinal angle, but we can test further on that.

// b.) Robot pose and wall derived pose are within some tolerance. Say 2.5 inches.
// However I'll also add a param to just skip all checks in case I know robot pose 
// really off and I can reliably and accurately reset.

// c.) Second issue is that the sensor could accidentally beam one of those cones
// near the edge of the field, which since it's close enough to the field edge and 
// thin enough(2.5 inches) it might squeeze past the robot pose vs wall pose 
// detection step. To fix this issue: 

    // I.) I'll run a confidence test first, since hitting a wall usually gives a 
    //solid 63 confidence rating. 

    // II.) I'll also run a loose object size test. If it's clearly too small(like <90)
    // then throw it out

    // III.) 




//That should be the entire process

void WallLocalization::reset_pose(float bias_rate, bool override_checks){

    //wall localize on 33hz(because dist. sensors only refresh that fast)
    int now = pros::millis();
    if(now - lastResetTime >= 30){
        lastResetTime = now;
    }else{
        return; //cut execution entirely since we have stale readings, and a robot 
                //can move a surprisingly significant distance in only 30ms.
    }

    // current robot position
    float x = chassis->gpos().x;
    float y = chassis->gpos().y;
    float globalTheta = radToDeg(chassis->gpos().theta); // In degrees

    //check if we're at the extremeties of a cardinal angle, if yes, stop execution
    float normalized = std::fmod(std::fmod(globalTheta, 360.0f) + 360.0f, 360.0f); // normalize angle first into 0 to 360(could be like 745 or -25)
    float angleDeviation = std::abs(std::fmod(std::fmod(normalized + 45.0f, 90.0f) + 90.0f, 90.0f) - 45.0f);
    if(angleDeviation > 33){
        if(telemetryEnabled){
            TELEMETRY.send(std::format(
                "{{\"t\": {}, \"axis\": \"none\", \"side\": \"NONE\", \"accepted\": false, \"reject\": \"too_slanted\", "
                "\"distFromWall\": 0, \"poseVal\": 0, \"wallVal\": 0, \"errorIn\": 0, \"confidence\": 0, \"objectSize\": 0, "
                "\"theta\": {:.1f}, \"angleDev\": {:.1f}}}\n",
                now, globalTheta, angleDeviation));
        }
        return; //we're too slanted
    }

    //find which sensor to use
    WallSensor::Side front_side  = get_side_facing_front(globalTheta);
    WallSensor::Side desired_sensor_side_x = (x > 0 ? front_side - 1 : front_side + 1);//Need to know whether it's on the negative or positive side of the axis
    WallSensor::Side desired_sensor_side_y = (y > 0 ? front_side : front_side - 2);

    apply_axis(x, desired_sensor_side_x, "x", globalTheta, bias_rate, override_checks, now);
    apply_axis(y, desired_sensor_side_y, "y", globalTheta, bias_rate, override_checks, now);

    //set pose
    chassis->setPose(x,y);

}

void WallLocalization::apply_axis(float& val, WallSensor::Side side, const char* axisName, float globalTheta, float bias_rate, bool override_checks, int now){
    double dist_from_wall = get_dist_from_wall(side, globalTheta);

    if(dist_from_wall == 9999 || dist_from_wall == PROS_ERR){
        if(telemetryEnabled){
            TELEMETRY.send(std::format(
                "{{\"t\": {}, \"axis\": \"{}\", \"side\": \"{}\", \"accepted\": false, \"reject\": \"{}\", "
                "\"distFromWall\": 0, \"poseVal\": {:.2f}, \"wallVal\": 0, \"errorIn\": 0, \"confidence\": 0, \"objectSize\": 0, "
                "\"theta\": {:.1f}, \"angleDev\": 0}}\n",
                now, axisName, sideToStr(side), dist_from_wall == PROS_ERR ? "no_sensor" : "sensor_range_error",
                val, globalTheta));
        }
        return;
    }

    double wallVal = sgn(val) * (70 - dist_from_wall);
    double preVal = val;
    WallSensor* sensor = find_sensor(side);
    bool badReading = sensor->isObviouslyBad() && !override_checks;
    if(badReading) wallVal = val;

    bool applied = fabs(val - wallVal) < 2.5 || override_checks;
    if(applied){
        val -= (val - wallVal) * bias_rate;
    }

    bool accepted = applied && !badReading;

    if(telemetryEnabled){
        std::string reject = accepted ? "" : (badReading ? "bad_reading" : "pose_mismatch");
        TELEMETRY.send(std::format(
            "{{\"t\": {}, \"axis\": \"{}\", \"side\": \"{}\", \"accepted\": {}, \"reject\": \"{}\", "
            "\"distFromWall\": {:.2f}, \"poseVal\": {:.2f}, \"wallVal\": {:.2f}, \"errorIn\": {:.2f}, "
            "\"confidence\": {}, \"objectSize\": {}, \"theta\": {:.1f}, \"angleDev\": 0}}\n",
            now, axisName, sideToStr(side), accepted ? "true" : "false", reject,
            dist_from_wall, preVal, wallVal, preVal - wallVal,
            sensor->getConfidence(), sensor->getObjectSize(), globalTheta));
    }
}

bool WallLocalization::set_initial_pose(float headingDeg, Quadrant quadrant){
    float signX = (quadrant == Quadrant::PosXPosY || quadrant == Quadrant::PosXNegY) ? 1.0f : -1.0f;
    float signY = (quadrant == Quadrant::PosXPosY || quadrant == Quadrant::NegXPosY) ? 1.0f : -1.0f;

    WallSensor::Side front_side = get_side_facing_front(headingDeg);
    WallSensor::Side side_x = (signX > 0 ? front_side - 1 : front_side + 1);
    WallSensor::Side side_y = (signY > 0 ? front_side : front_side - 2);

    double dist_x = get_dist_from_wall(side_x, headingDeg);
    double dist_y = get_dist_from_wall(side_y, headingDeg);

    WallSensor* sensor_x = find_sensor(side_x);
    WallSensor* sensor_y = find_sensor(side_y);

    bool xOk = dist_x != 9999 && dist_x != PROS_ERR && sensor_x != nullptr && !sensor_x->isObviouslyBad();
    bool yOk = dist_y != 9999 && dist_y != PROS_ERR && sensor_y != nullptr && !sensor_y->isObviouslyBad();

    if(telemetryEnabled){
        TELEMETRY.send(std::format(
            "{{\"t\": {}, \"event\": \"set_initial_pose\", \"headingDeg\": {:.1f}, \"sideX\": \"{}\", \"sideY\": \"{}\", "
            "\"distX\": {:.2f}, \"distY\": {:.2f}, \"xOk\": {}, \"yOk\": {}}}\n",
            pros::millis(), headingDeg, sideToStr(side_x), sideToStr(side_y),
            dist_x, dist_y, xOk ? "true" : "false", yOk ? "true" : "false"));
    }

    if(!xOk || !yOk) return false;

    float x = signX * (70 - dist_x);
    float y = signY * (70 - dist_y);
    chassis->setPose(x, y, degToRad(headingDeg));
    return true;
}

WallSensor* WallLocalization::find_sensor(WallSensor::Side side){
    for (WallSensor* s : sensors) {
        if (s->side==side) { return s;}
    }
    return nullptr;
}

WallSensor::Side WallLocalization::get_side_facing_front(float angle){
    
        // normalize angle first into 0 to 360(could be like 745 or -25)
    float normalized = std::fmod(std::fmod(angle, 360.0f) + 360.0f, 360.0f);

    // 2. Shift by 45 and divide by 90 to get an index (0, 1, 2, or 3)
    // Adding 45 makes 315-45 become the "0" bucket.
    int index = static_cast<int>((normalized + 45.0f)) / 90;

    // 3. Wrap index 4 back to 0 (for the 315-360 range)
    index %= 4;

    if (index == 0) { //we are facing to the right wall, so we return left sensor
        return (WallSensor::Side::LEFT);
    } else if (index == 1) { //we are facing to the front wall, so return the front sensor
        return (WallSensor::Side::FRONT); //if within (45, 135) basically facing 90 degrees
    } else if (index == 2) { //we are facing to the left wall, so return the left sensor
        return (WallSensor::Side::RIGHT); //if within (135, 225) basically facing 180 degrees
    } else if (index == 3) { //we are facing to the back wall, so return the back sensor
        return (WallSensor::Side::BACK); //if within (225, 315) basically facing 270 degrees
    } else {
        return WallSensor::Side::FRONT; // shouldn't be possible
    }
}

double WallLocalization::get_dist_from_wall(WallSensor::Side side, float globalTheta){
    WallSensor* sensor = find_sensor(side);
    if(sensor == nullptr) return PROS_ERR; //nah there is no sensor available for that side
    float dist = sensor->getDist();
    if(dist == 9999){ //ERROR VAL
        return 9999; //error val
    }
    float localTheta = std::fmod(std::fmod(globalTheta + 45.0f, 90.0f) + 90.0f, 90.0f) -
                       45.0f; // add 45 in the beginning to account for negative values
    localTheta = localTheta * (std::numbers::pi_v<float> / 180.0f);
    return (dist + sensor->vertOffset) * std::cos(localTheta) +
           (sensor->horizOffset * std::sin(localTheta)); // get dist from center or robot to wall
}
