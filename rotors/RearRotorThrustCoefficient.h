#pragma once

namespace RearRotorThrustCoefficient {
	float C_t = 0.1; //Thrust coefficient for the rotor
	float BladeLen = 3; //Length of rotor blades 
	float Value() {
			return C_t;
	};
}