#pragma once
#include "../include/MatrixDef.h"
#ifndef PI
#define PI 3.14159
#endif

#pragma once

namespace Ailerons {
    float C_0 = 0;
    float C_Slope = 0.05; //Slope of the pitching moment (Estimated)
    float Value(float AileronDeflection) {
        float AileronDeflectionDeg = AileronDeflection*(180.0/PI);
        return C_0 + C_Slope*AileronDeflectionDeg;
    }
}