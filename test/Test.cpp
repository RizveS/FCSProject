#include "../Dynamics.h"
#include "../include/ControlStruct.h"
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

        std::cout << "Enter desired throttle position (Range: [0,100]): ";
        std::cin >> ThrottlePosition;
        std::cout << std::endl;
        std::cout << "Desired Throttle Position: " << ThrottlePosition;
        std::cout << std::endl;

        std::cout << "Enter desired yoke twist (Range: [-90,90] deg): ";
        std::cin >> YokeTwist;
        std::cout << std::endl;
        std::cout << "Desired Yoke Twist: " << YokeTwist;
        std::cout << std::endl;

        std::cout << "Enter desired yoke depression (Range: [-50,50]): ";
        std::cin >> YokeDepression;
        std::cout << std::endl;
        std::cout << "Desired Yoke Depression: " << YokeDepression;
        std::cout << std::endl;

        std::cout << "Enter desired rotor spin rate: (Range: [0,100]): ";
        std::cin >> WingRotorControl;
        std::cout << std::endl;
        std::cout << "Desired rotor spin rate: " << WingRotorControl;
        std::cout << std::endl;

        std::cout << "Enter left rotor pitch: (Range: [0,100]): ";
        std::cin >> LeftRotorPitch;
        std::cout << std::endl;
        std::cout << "Desired left rotor pitch: " << LeftRotorPitch;
        std::cout << std::endl;

        std::cout << "Enter right rotor pitch (Range: [0,100]): ";
        std::cin >> RightRotorPitch;
        std::cout << std::endl;
        std::cout << "Desired right rotor pitch: " << RightRotorPitch;
        std::cout << std::endl;

        std::cout << "Enter total number of time steps: ";
        std::cin >> NumTimeSteps;
        std::cout << std::endl;
        std::cout << "Total number of time steps: " << NumTimeSteps;
        std::cout << std::endl;

        return 0;
}