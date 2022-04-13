#include "./Forces.h"
#include "./Moments.h"
#define RPM_TO_RADPERSEC PI/30


namespace Dynamics {
    VehicleParamStruct SetVehicleProps() {
        VehicleParamStruct VehicleParam;
        VehicleParam.WingSpan = WingProps::WingSpan;
        VehicleParam.WingArea = std::pow(WingProps::WingSpan,2)/WingProps::AR;
        
        VEC_3X1 RightRotorPos;
        VEC_3X1 LeftRotorPos;
        VEC_3X1 RearRotorPos;
        VEC_3X1 RudderPos;
        VEC_3X1 ElevatorPos;

        RightRotorPos << 0,0,0;
        LeftRotorPos << 0,0,0;
        RearRotorPos << 0,0,0;
        RudderPos << 0,0,0;
        ElevatorPos << 0,0,0;

        VehicleParam.RightRotorPos = RightRotorPos;
        VehicleParam.LeftRotorPos = LeftRotorPos;
        VehicleParam.RearRotorPos = RearRotorPos;
        VehicleParam.RudderPos = RudderPos;
        VehicleParam.ElevatorPos = ElevatorPos;
    };

    VEC_12X1 MarchForward(float deltaT, VEC_12X1 prevState, float ThrottlePosition, float YokeTwist, float YokeDepression, 
    float PedalDepression, float WingRotorControl, float LeftRotorPitch, float RightRotorPitch) {

    ControlStruct controls;
    //Process ThrottlePosition input
    //Assume throttle is used to control rear rotor speed
    float MAX_RPM = 3000;
    controls.RearRotorSpeed = MAX_RPM*(ThrottlePosition/100);

    //Process YokeTwist input
    //Assumes yoke twist directly commands deflection of ailerons
    //Assume YokeTwist input is within [-90,90] and is input as degrees
    float MAX_AILERON_DEFLECTION = PI/10;
    controls.AileronDeflection = MAX_AILERON_DEFLECTION*(YokeTwist/90);


    //Process YokeDepression input
    //Assume YokeDepression is within [-50,50]
    float MAX_FLAP_DEFLECTION = PI/10;
    float FLAP_ELEVATOR_CROSS_SENSITIVITY = 0.3; //Elevators on the tail balance upward pitching moment from flaps
    controls.FlapDeflection = MAX_FLAP_DEFLECTION*(YokeDepression/50);
    controls.ElevatorDeflection = FLAP_ELEVATOR_CROSS_SENSITIVITY*MAX_FLAP_DEFLECTION*(YokeDepression/50);

    //Process PedalDepression
    //Assumed to vary between [-50,50]
    /*Positive values turn rudder through positive deflection 
    (hence creating negative yaw moment due to the direction of moment arm)*/
    float MAX_RUDDER_DEFLECTION = PI/10;
    controls.RudderDeflection = MAX_RUDDER_DEFLECTION*(PedalDepression/50);


    //Process WingRotorControl
    /*IN PROGRESS -> What kind of physical controls would this use?
     Assume WingRotorControl is set and varies within [0,100]*/
    float MAX_RPM = 3000;
    controls.LeftRotorSpeed = MAX_RPM*(WingRotorControl/100);
    controls.RightRotorSpeed = MAX_RPM*(WingRotorControl/100);

    //Process WingRotor Pitch
    controls.LeftRotorPitch = LeftRotorPitch;
    controls.RightRotorPitch = RightRotorPitch;

    VehicleParamStruct VehicleParam = SetVehicleProps();

    VEC_12X1 StateDer;





    
    };


}
