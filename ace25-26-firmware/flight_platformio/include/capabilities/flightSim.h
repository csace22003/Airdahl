# pragma once

#include <array>
#define STATE_VEC_SIZE 2


class flightSim{
    private:
        float g;
        float Cd;
        float A;
        float mass;
        float dt;
        std::array<float,STATE_VEC_SIZE> simState;
        float curDrag(float V, float alt);
        std::array<float,STATE_VEC_SIZE> rocketDynamics(std::array<float,STATE_VEC_SIZE> state, float drag);
        static std::array<float,STATE_VEC_SIZE> addVectorsElementwise(std::array<float,STATE_VEC_SIZE> v1, std::array<float,STATE_VEC_SIZE> vSTATE_VEC_SIZE);
        static std::array<float,STATE_VEC_SIZE> multiplyVectorScalar(std::array<float,STATE_VEC_SIZE> v, float k); 
    public:
        flightSim(float g, float Cd, float A, float mass, float dt);
        void initializeSim(std::array<float,STATE_VEC_SIZE> startState);
        float runSim();
};