#pragma once

#include "./include/MatrixDef.h"
#include "./include/VehicleParam.h"
#include "./include/ControlStruct.h"
#include "./wings/WingProperties.h"

#include "./wings/LiftCoefficient.h"
#include "./wings/DragCoefficient.h"

#include "./rotors/RearRotorThrustCoefficient.h"
#include "./rotors/LeftRotorThrustCoefficient.h"
#include "./rotors/RightRotorThrustCoefficient.h"

#include "./controls/Elevator.h"
#include "./controls/Rudder.h"
#include "./controls/Ailerons.h"
#include "./controls/Flaps.h"

#include "./wings/PitchMomentCoefficient.h"
#include "./wings/RollMomentCoefficient.h"
#include "./wings/YawMomentCoefficient.h"

typedef Eigen::Matrix<double,3,3> MAT_3X3; // Redefines the type to MAT for easy use. Most matrices used are 3x3
typedef Eigen::Matrix<double,3,1> VEC_3X1; // Redefines the type to VEC for easy use. Most vectors used are 3x1
typedef Eigen::Matrix<double,12,1> VEC_12X1; //State vector for dynamics is 12 x 1

namespace Moments {
    VEC_3X1 AerodynamicMoment(double t,VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
    float p_N = State(0,0);  //North-ward position of the craft
	float p_E = State(1,0);  //East-ward position of the craft
	float p_B = State(2,0);  //Vertical position of the craft, measured positive in a downwards direction
	float pitch = State(3,0); //Pitch of the craft
	float roll = State(4,0); //Roll of the craft
	float yaw = State(5,0);  //Yaw of the craft
	float U = State(6,0); //X velocity of the craft in the body frame
	float V = State(7,0); //Y velocity of the craft in the body frame
	float W = State(8,0); //Z velocity of the craft in the body frame
	float P = State(9,0); //Body rotation rate along body X axis
	float Q = State(10,0); //Body rotation rate along body Y axis
	float R = State(11,0); //Body rotation rate along body Z axis

    float V_T = std::sqrt(std::pow(U,2)+std::pow(V,2)+std::pow(W,2));
	float AoA;
	float SideSlip;
    if ((V_T > 1E-3) & (std::abs(U) > 1E-3)) {
		AoA = std::atan2(W,U);
		SideSlip = std::asin(V/V_T);
	}
	else {
		AoA = 0;
		SideSlip = 0;
	}
	float MachNumber = 0; //Would have to have temperature as part of the state equations. Did not adjust for additional complexity of doing this
    
    MAT_3X3 ROT_MAT_WINDFRAME_TO_BODY;
	ROT_MAT_WINDFRAME_TO_BODY(0,0) = std::cos(AoA)*std::cos(SideSlip);
	ROT_MAT_WINDFRAME_TO_BODY(1,0) = -std::cos(AoA)*std::sin(SideSlip);
	ROT_MAT_WINDFRAME_TO_BODY(2,0) = -std::sin(AoA);
	ROT_MAT_WINDFRAME_TO_BODY(0,1) = std::sin(SideSlip);
	ROT_MAT_WINDFRAME_TO_BODY(1,1) = std::cos(SideSlip);
	ROT_MAT_WINDFRAME_TO_BODY(2,1) = 0;
	ROT_MAT_WINDFRAME_TO_BODY(0,2) = std::sin(AoA)*std::cos(SideSlip);
	ROT_MAT_WINDFRAME_TO_BODY(1,2) = -std::sin(AoA)*std::sin(SideSlip);
	ROT_MAT_WINDFRAME_TO_BODY(2,2) = std::cos(AoA);


    float AileronDeflection = controls.AileronDeflection;
	float RudderDeflection = controls.RudderDeflection;
	float ElevatorDeflection = controls.ElevatorDeflection;
	float FlapDeflection = controls.FlapDeflection;

    float rho = 1.225;
    float dynamicPressure = 0.5*rho*(pow(V_T,2));

    float pitchMoment = PitchMomentCoefficient::Value(AoA,SideSlip,MachNumber)*dynamicPressure*((std::pow(WingProps::WingSpan,2)/WingProps::AR))*WingProps::MAC;
    float rollMoment = RollMomentCoefficient::Value(AoA,SideSlip,MachNumber)*dynamicPressure*((std::pow(WingProps::WingSpan,2)/WingProps::AR))*WingProps::MAC;
    float yawMoment = YawMomentCoefficient::Value(AoA,SideSlip,MachNumber)*dynamicPressure*((std::pow(WingProps::WingSpan,2)/WingProps::AR))*WingProps::MAC;

    float LiftElevator = (Elevator::LiftValue(ElevatorDeflection))*dynamicPressure*(Elevator::ElevatorPlanformArea);
	float DragElevator = (Elevator::DragValue(ElevatorDeflection))*dynamicPressure*(Elevator::ElevatorPlanformArea);
    VEC_3X1 ElevatorForce;
    ElevatorForce << -DragElevator,0,-LiftElevator;
    ElevatorForce = ROT_MAT_WINDFRAME_TO_BODY*ElevatorForce;
    VEC_3X1 ElevatorMoment = VehicleParam.ElevatorPos.cross(ElevatorForce);


    float SideForceRudder = (Rudder::SideForceValue(RudderDeflection))*dynamicPressure*(Rudder::RudderPlanformArea);
	float DragRudder = (Rudder::DragValue(RudderDeflection))*dynamicPressure*(Rudder::RudderPlanformArea);
    VEC_3X1 RudderForce;
    RudderForce << -DragRudder,SideForceRudder,0;
    VEC_3X1 RudderMoment = VehicleParam.RudderPos.cross(RudderForce);

	float aileronRollMoment = Ailerons::Value(AileronDeflection)*dynamicPressure*((std::pow(WingProps::WingSpan,2)/WingProps::AR))*WingProps::MAC;
	float flapPitchMoment = Flaps::PitchMomentValue(FlapDeflection)*dynamicPressure*((std::pow(WingProps::WingSpan,2)/WingProps::AR))*WingProps::MAC;

    VEC_3X1 TotalMoment;
    TotalMoment << rollMoment + aileronRollMoment,pitchMoment+flapPitchMoment,yawMoment;
    TotalMoment = TotalMoment + ElevatorMoment + RudderMoment;

    return TotalMoment;

    };
    VEC_3X1 RearRotorMoment(double t, VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
        float p_N = State(0,0);  //North-ward position of the craft
        float p_E = State(1,0);  //East-ward position of the craft
        float p_B = State(2,0);  //Vertical position of the craft, measured positive in a downwards direction
        float pitch = State(3,0); //Pitch of the craft
        float roll = State(4,0); //Roll of the craft
        float yaw = State(5,0);  //Yaw of the craft
        float U = State(6,0); //X velocity of the craft in the body frame
        float V = State(7,0); //Y velocity of the craft in the body frame
        float W = State(8,0); //Z velocity of the craft in the body frame
        float P = State(9,0); //Body rotation rate along body X axis
        float Q = State(10,0); //Body rotation rate along body Y axis
        float R = State(11,0); //Body rotation rate along body Z axis

        float AngRate = controls.RearRotorSpeed;
	    float rho = 1.225;
        float dynamicPressure = 0.5*rho*pow(RearRotorThrustCoefficient::BladeLen*AngRate,2);
        VEC_3X1 RotorForce;
	    RotorForce << RearRotorThrustCoefficient::C_t*dynamicPressure*(PI*pow(RearRotorThrustCoefficient::BladeLen,2)),0,0;
        VEC_3X1 RotorMoment = VehicleParam.RearRotorPos.cross(RotorForce);

        return RotorMoment;
    };
    VEC_3X1 LeftRotorMoment(double t, VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
        float p_N = State(0,0);  //North-ward position of the craft
	float p_E = State(1,0);  //East-ward position of the craft
	float p_B = State(2,0);  //Vertical position of the craft, measured positive in a downwards direction
	float pitch = State(3,0); //Pitch of the craft
	float roll = State(4,0); //Roll of the craft
	float yaw = State(5,0);  //Yaw of the craft
	float U = State(6,0); //X velocity of the craft in the body frame
	float V = State(7,0); //Y velocity of the craft in the body frame
	float W = State(8,0); //Z velocity of the craft in the body frame
	float P = State(9,0); //Body rotation rate along body X axis
	float Q = State(10,0); //Body rotation rate along body Y axis
	float R = State(11,0); //Body rotation rate along body Z axis

	float AngRate = controls.LeftRotorSpeed;
	float rho = 1.225;
	float dynamicPressure = 0.5*rho*pow(LeftRotorThrustCoefficient::BladeLen*AngRate,2);

	VEC_3X1 RotorForce;
	RotorForce << 0,0,-LeftRotorThrustCoefficient::C_t*dynamicPressure*(PI*pow(LeftRotorThrustCoefficient::BladeLen,2));
    VEC_3X1 RotorMoment = VehicleParam.LeftRotorPos.cross(RotorForce);

    return RotorMoment;
    };
    VEC_3X1 RightRotorMoment(double t, VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
        	float p_N = State(0,0);  //North-ward position of the craft
	float p_E = State(1,0);  //East-ward position of the craft
	float p_B = State(2,0);  //Vertical position of the craft, measured positive in a downwards direction
	float pitch = State(3,0); //Pitch of the craft
	float roll = State(4,0); //Roll of the craft
	float yaw = State(5,0);  //Yaw of the craft
	float U = State(6,0); //X velocity of the craft in the body frame
	float V = State(7,0); //Y velocity of the craft in the body frame
	float W = State(8,0); //Z velocity of the craft in the body frame
	float P = State(9,0); //Body rotation rate along body X axis
	float Q = State(10,0); //Body rotation rate along body Y axis
	float R = State(11,0); //Body rotation rate along body Z axis

	float AngRate = controls.LeftRotorSpeed;
	float rho = 1.225;
	float dynamicPressure = 0.5*rho*pow(RightRotorThrustCoefficient::BladeLen*AngRate,2);

	VEC_3X1 RotorForce;
	RotorForce << 0,0,-RightRotorThrustCoefficient::C_t*dynamicPressure*(PI*pow(RightRotorThrustCoefficient::BladeLen,2));
    VEC_3X1 RotorMoment = VehicleParam.RightRotorPos.cross(RotorForce);
    return RotorMoment;
    };
}