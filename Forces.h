#pragma once
#include "./include/MatrixDef.h"
#include "./include/VehicleParam.h"
#include "./include/ControlStruct.h"

#include "./wings/LiftCoefficient.h"
#include "./wings/DragCoefficient.h"

#include "./rotors/RearRotorThrustCoefficient.h"
#include "./rotors/LeftRotorThrustCoefficient.h"
#include "./rotors/RightRotorThrustCoefficient.h"

#include "./controls/Elevator.h"
#include "./controls/Rudder.h"
#include "./controls/Ailerons.h"
#include "./controls/Flaps.h"


namespace Forces {
//Each of these functions return a force vector in the body frame
VEC_3X1 AerodynamicForce(double t,VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
	//Decompose state in respective variables
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

	float AileronDeflection = controls.AileronDeflection;
	float RudderDeflection = controls.RudderDeflection;
	float ElevatorDeflection = controls.ElevatorDeflection;
	float FlapDeflection = controls.FlapDeflection;


	float V_T = std::sqrt(std::pow(U,2) + std::pow(V,2)+ std::pow(W,2));
	float AoA = std::atan2(U,W);
	float SideSlip = std::asin(V/V_T);
	float MachNumber = 0; //Would have to have temperature as part of the state equations. Did not adjust for additional complexity of doing this


	float rho = 1.225;
	float dynamicPressure = 0.5*rho*(pow(V_T,2));


	//Lift and Drag values in the wind frame. Must be rotated into body frame
	float LiftLeftWing = (LiftCoefficient::Value(AoA,SideSlip,MachNumber))*dynamicPressure*(pow(WingProps::WingSpan,2)/WingProps::AR);
	float LiftRightWing = (LiftCoefficient::Value(AoA,SideSlip,MachNumber))*dynamicPressure*(pow(WingProps::WingSpan,2)/WingProps::AR);
	float DragLeftWing = (DragCoefficient::Value(AoA,SideSlip,MachNumber))*dynamicPressure*(pow(WingProps::WingSpan,2)/WingProps::AR);
	float DragRightWing = (DragCoefficient::Value(AoA,SideSlip,MachNumber))*dynamicPressure*(pow(WingProps::WingSpan,2)/WingProps::AR);

	float LiftFlaps = (Flaps::LiftValue(FlapDeflection))*dynamicPressure*(pow(WingProps::WingSpan,2)/WingProps::AR);
	float DragFlaps = (Flaps::DragValue(FlapDeflection))*dynamicPressure*(pow(WingProps::WingSpan,2)/WingProps::AR);

	float LiftElevator = (Elevator::LiftValue(ElevatorDeflection))*dynamicPressure*(Elevator::ElevatorPlanformArea);
	float DragElevator = (Elevator::DragValue(ElevatorDeflection))*dynamicPressure*(Elevator::ElevatorPlanformArea);
	
	float SideForceRudder = (Rudder::SideForceValue(RudderDeflection))*dynamicPressure*(Rudder::RudderPlanformArea);
	float DragRudder = (Rudder::DragValue(RudderDeflection))*dynamicPressure*(Rudder::RudderPlanformArea);


	VEC_3X1 AerodynamicForceVal;
	AerodynamicForceVal(0,0) = -1*(DragLeftWing+DragRightWing+DragElevator+DragFlaps+DragRudder);
	AerodynamicForceVal(1,0) = SideForceRudder;
	AerodynamicForceVal(2,0) = -1*(LiftLeftWing+LiftRightWing+LiftElevator+LiftFlaps);

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

	AerodynamicForceVal = ROT_MAT_WINDFRAME_TO_BODY*AerodynamicForceVal;
	return AerodynamicForceVal;
	
}
VEC_3X1 RearRotorForce(double t, VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
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
	RotorForce(0,0) = RearRotorThrustCoefficient::C_t*dynamicPressure*(PI*pow(RearRotorThrustCoefficient::BladeLen,2));
	RotorForce(1,0) = 0;
	RotorForce(2,0) = 0;
	return RotorForce;
};
VEC_3X1 LeftRotorForce(double t, VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
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
	RotorForce(0,0) = 0;
	RotorForce(1,0) = 0;
	RotorForce(2,0) = -LeftRotorThrustCoefficient::C_t*dynamicPressure*(PI*pow(LeftRotorThrustCoefficient::BladeLen,2));
	float LeftRotorPitch = controls.LeftRotorPitch; //Sense of rotation is along + body y-axis
	MAT_3X3 RotMat;
	RotMat << std::cos(LeftRotorPitch), 0, -std::sin(LeftRotorPitch),0,1,0,std::sin(LeftRotorPitch),0,std::cos(LeftRotorPitch);
	return RotMat*RotorForce;

};
VEC_3X1	RightRotorForce(double t, VEC_12X1 State, VehicleParamStruct VehicleParam, ControlStruct controls) {
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
	RotorForce(0,0) = 0;
	RotorForce(1,0) = 0;
	RotorForce(2,0) = -RightRotorThrustCoefficient::C_t*dynamicPressure*(PI*pow(RightRotorThrustCoefficient::BladeLen,2));
	float RightRotorPitch = controls.RightRotorPitch; //Sense of rotation is along + body y-axis
	MAT_3X3 RotMat;
	RotMat << std::cos(RightRotorPitch), 0, -std::sin(RightRotorPitch),0,1,0,std::sin(RightRotorPitch),0,std::cos(RightRotorPitch);
	return RotMat*RotorForce;
	return RotorForce;
};
}
