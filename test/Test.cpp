#include "../Dynamics.h"
#include "../include/ControlStruct.h"
#include<vector>
#include<iostream>


int main(int argc, char* argv[]) {
        float ThrottlePosition;
        float YokeTwist;
        float YokeDepression;
        float PedalDepression;
        float WingRotorControl;
        float LeftRotorPitch;
        float RightRotorPitch;
        int NumTimeSteps;
        float deltaT;
        float t = 0;

        std::cout << "Enter desired throttle position (Range: [0,100]): ";
        std::cin >> ThrottlePosition;
        std::cout << "Desired Throttle Position: " << ThrottlePosition;
        std::cout << std::endl;

        std::cout << "Enter desired yoke twist (Range: [-90,90] deg): ";
        std::cin >> YokeTwist;
        std::cout << "Desired Yoke Twist: " << YokeTwist;
        std::cout << std::endl;

        std::cout << "Enter desired yoke depression (Range: [-50,50]): ";
        std::cin >> YokeDepression;
        std::cout << "Desired Yoke Depression: " << YokeDepression;
        std::cout << std::endl;

        std::cout << "Enter desired rotor spin rate: (Range: [0,100]): ";
        std::cin >> WingRotorControl;
        std::cout << "Desired rotor spin rate: " << WingRotorControl;
        std::cout << std::endl;

        std::cout << "Enter left rotor pitch: (Range: [0,100]): ";
        std::cin >> LeftRotorPitch;
        std::cout << "Desired left rotor pitch: " << LeftRotorPitch;
        std::cout << std::endl;

        std::cout << "Enter right rotor pitch (Range: [0,100]): ";
        std::cin >> RightRotorPitch;
        std::cout << "Desired right rotor pitch: " << RightRotorPitch;
        std::cout << std::endl;

        std::cout << "Enter total number of time steps: ";
        std::cin >> NumTimeSteps;
        std::cout << "Total number of time steps: " << NumTimeSteps;
        std::cout << std::endl;

        std::cout << "Enter time increment per time step: ";
        std::cin >> deltaT;
        std::cout << "Total number of time steps: " << deltaT;
        std::cout << std::endl;

        //State Vector Initialization
        VEC_12X1 InitialState = Dynamics::Initialize();
        std::vector<VEC_12X1> StateVector; //Stores the state vector for each time step
        StateVector.push_back(InitialState);
        VEC_12X1 NextState;


        //Main loop for looping through dynamics
        for (int iteration = 1; iteration < NumTimeSteps +1; iteration++) {
            t = t + deltaT;
            NextState = Dynamics::MarchForward(t,deltaT,StateVector[iteration],ThrottlePosition,YokeTwist,YokeDepression,PedalDepression,WingRotorControl,LeftRotorPitch,RightRotorPitch);
            std::cout << "-----------------------------------------------------" << std::endl;
            std::cout << "Displacement about Body X+: " << NextState[0] << std::endl;
            std::cout << "Displacement about Body Y+: " << NextState[1] << std::endl;
            std::cout << "Displacement about Body Z+: " << NextState[2] << std::endl;
            std::cout << "Pitch: " << NextState[3] << std::endl;
            std::cout << "Roll: " << NextState[4] << std::endl;
            std::cout << "Yaw: " << NextState[5] << std::endl;
            std::cout << "Speed about Body X+: " << NextState[6] << std::endl;
            std::cout << "Speed about Body Y+: " << NextState[7] << std::endl;
            std::cout << "Speed about Body Z+: " << NextState[8] << std::endl;
            std::cout << "Rotation Rate about Body X+: " << NextState[9] << std::endl;
            std::cout << "Rotation Rate about Body Y+: " << NextState[10] << std::endl;
            std::cout << "Rotation Rate about Body Z+: " << NextState[11] << std::endl;
            std::cout <<"Time Elapsed in Simulation: " << t;
            StateVector.push_back(NextState);

        }
}