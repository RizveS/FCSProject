#pragma once
namespace Elevator {
	float ElevatorLiftSensitivity = 0.25;
    float ElevatorDragSensitivity = 0.005;
    float ElevatorPlanformArea = 2.0;

    float LiftValue(float ElevatorAngle) {
        //Assumes that all angles are expressed in radians 
        //Mach number independence is assumed
        float ElevatorAngleDeg = ElevatorAngle*(180.0/PI);
        return (ElevatorLiftSensitivity*ElevatorAngleDeg);
    	};

    float DragValue(float ElevatorAngle) {
        float ElevatorAngleDeg = ElevatorAngle*(180.0/PI);
        return (ElevatorDragSensitivity*ElevatorAngleDeg);
    };
}