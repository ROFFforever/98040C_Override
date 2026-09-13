#include "WallLocalization.h"
#include "util/mathUtils.h"
#include "pros/error.h"
#include <cmath>

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

// a.) Robot pose and wall derived pose are within some tolerance. Say 4 inches.
// However I'll also add a param to just skip all checks in case I know robot pose 
// really off and I can reliably and accurately reset.


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
    float globalTheta = chassis->gpos().theta; // In degrees
    float new_y = y;

    //find which sensor to use
    WallSensor::Side front_side  = get_side_facing_front();
    WallSensor::Side desired_sensor_side_x = (x > 0 ? front_side - 1 : front_side + 1);//Need to know whether it's on the negative or positive side of the axis
    WallSensor::Side desired_sensor_side_y = (y > 0 ? front_side : front_side - 2);

    //Now calculate how far away we are from walls with ray casting 
    double dist_from_wall_x = get_dist_from_wall(desired_sensor_side_x);
    double dist_from_wall_y = get_dist_from_wall(desired_sensor_side_y);


}

WallSensor* WallLocalization::find_sensor(WallSensor::Side side){
    for (WallSensor* s : sensors) {
        if (s->side==side) { return s;}
    }
    return nullptr;
}

WallSensor::Side WallLocalization::get_side_facing_front(){
    double angle = radToDeg(chassis->gpos().theta);
    
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

double WallLocalization::get_dist_from_wall(WallSensor::Side side){
    float globalTheta = radToDeg(chassis->gpos().theta); // In degrees
    WallSensor* sensor = find_sensor(side);
    if(sensor == nullptr) return PROS_ERR; //nah there is no sensor available for that side
    float dist = sensor->getDist();
    if(dist == PROS_ERR){ //ERROR VAL
        return PROS_ERR; //error val
    }
    float localTheta = std::fmod(std::fmod(globalTheta + 45.0f, 90.0f) + 90.0f, 90.0f) -
                       45.0f; // add 45 in the beginning to account for negative values
    localTheta = localTheta * (std::numbers::pi_v<float> / 180.0f);
    return (dist + sensor->vertOffset) * std::cos(localTheta) +
           (sensor->horizOffset * std::sin(localTheta)); // get dist from center or robot to wall
}
