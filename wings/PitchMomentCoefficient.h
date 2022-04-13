#include "../include/MatrixDef.h"
#ifndef PI
#define PI 3.14159
#endif

#pragma once

namespace PitchMomentCoefficient {
    float C_0 = 0;
    float C_Slope = -0.02; //Slope of the pitching moment (Estimated)
    float Value(float AoA, float SideSlip, float MachNumber) {
        float AoADeg = AoA*(180.0/PI);
        return C_0 + C_Slope*AoADeg;
    }
}