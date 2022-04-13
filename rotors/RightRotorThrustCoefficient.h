#pragma once

namespace RightRotorThrustCoefficient {
	float C_t = 0.1; //Thrust coefficient for the rotor
	float BladeLen = 1; //Length of rotor blades 
	float Value() {
			return C_t;
	};
}