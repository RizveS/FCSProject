#pragma once

namespace Flaps {
    float FlapLiftSensitivity = 0.15;
    float FlapDragSensitivity = 0.05;
    float FlapPitchSensitivity = 0.003;

    float LiftValue(float FlapDeflection) {
        FlapDeflection = FlapDeflection*(180.0/PI);
        return FlapLiftSensitivity*FlapDeflection;
    	};

    float DragValue(float FlapDeflection) {
        float FlapDeflection = FlapDeflection*(180.0/PI);
        return (FlapDragSensitivity*FlapDeflection);
    };

    float PitchMomentValue(float FlapDeflection) {
        return FlapPitchSensitivity*FlapDeflection;
    }
}