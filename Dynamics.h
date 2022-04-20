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

    VEC_12X1 MarchForward(float t, float deltaT, VEC_12X1 prevState, float ThrottlePosition, float YokeTwist, float YokeDepression, 
    float PedalDepression, float WingRotorControl, float LeftRotorPitch, float RightRotorPitch) {

    float T = t+deltaT;
    float gD = 9.81;

    ControlStruct controls;
    //Process ThrottlePosition input
    //Assume throttle is used to control rear rotor speed
    float REAR_ROTOR_MAX_RPM = 3000;
    controls.RearRotorSpeed = REAR_ROTOR_MAX_RPM*(ThrottlePosition/100);

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
    float WING_ROTOR_MAX_RPM = 3000;
    controls.LeftRotorSpeed = WING_ROTOR_MAX_RPM*(WingRotorControl/100);
    controls.RightRotorSpeed = WING_ROTOR_MAX_RPM*(WingRotorControl/100);

    //Process WingRotor Pitch
    controls.LeftRotorPitch = LeftRotorPitch;
    controls.RightRotorPitch = RightRotorPitch;

    VehicleParamStruct VehicleParam = SetVehicleProps();
    float Jx = VehicleParam.I(0,0);
    float Jy = VehicleParam.I(1,1);
    float Jz = VehicleParam.I(2,2);
    float Jxz = VehicleParam.I(0,2);

    ////Decompose state in respective variables
	float p_N = prevState(0,0);  //North-ward position of the craft
	float p_E = prevState(1,0);  //East-ward position of the craft
	float p_B = prevState(2,0);  //Vertical position of the craft, measured positive in a downwards direction
	float pitch = prevState(3,0); //Pitch of the craft
	float roll = prevState(4,0); //Roll of the craft
	float yaw = prevState(5,0);  //Yaw of the craft
	float U = prevState(6,0); //X velocity of the craft in the body frame
	float V = prevState(7,0); //Y velocity of the craft in the body frame
	float W = prevState(8,0); //Z velocity of the craft in the body frame
	float P = prevState(9,0); //Body rotation rate along body X axis
	float Q = prevState(10,0); //Body rotation rate along body Y axis
	float R = prevState(11,0); //Body rotation rate along body Z axis
    
    VEC_12X1 StateDer;

    StateDer(0,0) =  U*std::cos(pitch)*std::cos(yaw) + 
    V*(-std::cos(roll)*sin(yaw)+std::sin(roll)*std::cos(pitch)*std::cos(yaw)) +
    W*(std::sin(roll)*std::sin(yaw)+std::cos(roll)*std::sin(pitch)*std::cos(yaw));

    StateDer(1,0) = U*(std::cos(pitch)*std::sin(yaw)) + 
    V*(std::cos(roll)*std::cos(yaw) + std::sin(roll)*std::sin(pitch)*std::sin(yaw)) + 
    W*(-std::sin(roll)*std::cos(yaw)+std::cos(roll)*std::sin(pitch)*std::sin(yaw));

    StateDer(2,0) = U*std::sin(pitch)-V*std::sin(roll)*std::cos(pitch)-W*std::cos(roll)*std::cos(pitch);

    StateDer(3,0) = P + std::tan(pitch)*(Q*std::sin(roll)+R*std::cos(roll));

    StateDer(4,0) = Q*std::cos(roll) - R*std::sin(roll);

    StateDer(5,0) = (Q*std::sin(roll)+R*std::cos(roll))/std::cos(pitch);


    float lambda = (1.0/(Jx*Jz-std::pow(Jxz,2)));
    VEC_3X1 AeroMoment = Moments::AerodynamicMoment(T,prevState,VehicleParam,controls);
    VEC_3X1 RearRotorMoment = Moments::RearRotorMoment(T,prevState,VehicleParam,controls);
    VEC_3X1 LeftRotorMoment = Moments::LeftRotorMoment(T,prevState,VehicleParam,controls);
    VEC_3X1 RightRotorMoment = Moments::RightRotorMoment(T,prevState,VehicleParam,controls);


    StateDer(6,0) = lambda*(Jxz*(Jx-Jy+Jz)*P*Q-(Jz*(Jz-Jy)+std::pow(Jxz,2))*Q*R + 
    AeroMoment(0,0) + RearRotorMoment(0,0) + LeftRotorMoment(0,0) + RightRotorMoment(0,0));

    StateDer(7,0) = (1.0/Jy)*((Jz-Jx)*P*R - Jxz*(std::pow(P,2)-std::pow(R,2)) +
    AeroMoment(1,0) + RearRotorMoment(1,0) + LeftRotorMoment(1,0) + RightRotorMoment(1,0));

    StateDer(8,0) = lambda*( ((Jx-Jy)*Jx + std::pow(Jxz,2))*P*Q - Jxz*(Jx-Jy+Jz)*Q*R +
    AeroMoment(2,0) + RearRotorMoment(2,0) + LeftRotorMoment(2,0) + RightRotorMoment(2,0)); 

    VEC_3X1 AeroForce = Forces::AerodynamicForce(T,prevState,VehicleParam,controls);
    VEC_3X1 RearRotorForce = Forces::RearRotorForce(T,prevState,VehicleParam,controls);
    VEC_3X1 LeftRotorForce = Forces::LeftRotorForce(T,prevState,VehicleParam,controls);
    VEC_3X1 RightRotorForce = Forces::RightRotorForce(T,prevState,VehicleParam,controls);

    StateDer(9,0) = R*V-Q*W-gD*std::sin(pitch) + (AeroForce(0,0)+RearRotorForce(0,0)+LeftRotorForce(0,0) + RightRotorForce(0,0))/VehicleParam.m;
    StateDer(10,0) = -R*U + P*W + gD*std::sin(roll)*std::cos(pitch) + (AeroForce(1,0)+RearRotorForce(1,0)+LeftRotorForce(1,0) + RightRotorForce(1,0))/VehicleParam.m;
    StateDer(11,0) = Q*U - P*V + gD*std::cos(roll)*std::cos(pitch) + (AeroForce(2,0)+RearRotorForce(2,0)+LeftRotorForce(2,0) + RightRotorForce(2,0))/VehicleParam.m;

    return prevState + StateDer*deltaT;
    
    };

    VEC_12X1 Initialize() {
    float p_N = 0;  //North-ward position of the craft
	float p_E = 0;  //East-ward position of the craft
	float p_B = 0;  //Vertical position of the craft, measured positive in a downwards direction
	float pitch = 0; //Pitch of the craft
	float roll = 0; //Roll of the craft
	float yaw = 0;  //Yaw of the craft
	float U = 0; //X velocity of the craft in the body frame
	float V = 0; //Y velocity of the craft in the body frame
	float W = 0; //Z velocity of the craft in the body frame
	float P = 0; //Body rotation rate along body X axis
	float Q = 0; //Body rotation rate along body Y axis
	float R = 0; //Body rotation rate along body Z axis

    VEC_12X1 InitialState;
    InitialState << p_N, p_E,p_B,pitch,roll,yaw,U,V,W,P,Q,R;
    }
}
