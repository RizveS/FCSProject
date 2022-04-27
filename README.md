# FCSProject
Implementation of the Terrafugia's Flight Dynamics

# Description
The Terrafugia is a VTOL aircraft capable of normal flight as a fixed wing aircraft. Dynamics of the Terrafugia have been implemented here with estimated values put in for the geometry and aerodynamic characteristics of the vehicle. The intended end-use of this implementation is as part of the Unreal Game Engine, where the values for the vehicle can be changed to provide a realistic feel for the handling of the vehicle

# Installation
The code base is written entirely in terms of header files, with modularity in mind. For the Terrafugia, there are two distinct sets of actuators: wings and rotors.
Each was characterized using a set of coefficients that describe the forces and moments generated when the actuators are in use. 
Please see Test.cpp in the tests folder for an example of how the dynamics can be implemented in plain C++. Implementations as Unreal components will be added as the project progresses.
