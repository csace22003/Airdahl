#include <flightSim.h>
#include <cmath>

flightSim::flightSim(float g, float Cd, float A, float mass, float dt):
       g(g),
       Cd(Cd),
       A(A),
       mass(mass),
       dt(dt),
       simState({0,0})
{}

float flightSim::curDrag(float V, float alt){
    float rho0 = 1.225; // Sea level density
    int h_scale = 8400; // Scale height (m)
    float rho = rho0 * exp(-1*alt / h_scale);
    return 0.5 * rho * Cd * A * V * V;
}

std::array<float,STATE_VEC_SIZE> flightSim::rocketDynamics(std::array<float,STATE_VEC_SIZE> state, float drag){
    float vel = state[1];
    float drag_force;
    // Drag force (opposes velocity)
    if(vel > 0.1){
        drag_force = -drag;
    }
    else{
        drag_force = 0;
    }
    // Gravity force
    float gravity_force = -1 * g * mass;
    // Total force
    float total_force = drag_force + gravity_force;
    // Acceleration
    float accel = total_force / mass;
    
    // Return derivatives
    std::array<float,STATE_VEC_SIZE> dsdt = {vel,accel};
    return dsdt;
}

void flightSim::initializeSim(std::array<float,STATE_VEC_SIZE> startState){
    for(size_t x = 0; x < startState.size(); ++x){
        simState[x] = startState[x]; // Reset simState to new initial conditions
    }
}

// Helper methods for matrix operations in C++ vector class
std::array<float,STATE_VEC_SIZE> flightSim::addVectorsElementwise(std::array<float,STATE_VEC_SIZE> v1, std::array<float,STATE_VEC_SIZE> v2){
    if(v1.size()!=v2.size()){ // Size mismatch
        return {};
    }
    else{
        std::array<float,STATE_VEC_SIZE> res = v1;
        for(size_t x = 0; x < v1.size(); ++x){
            res[x] = v1[x] + v2[x];
        }
        return res;
    }
}

std::array<float,STATE_VEC_SIZE> flightSim::multiplyVectorScalar(std::array<float,STATE_VEC_SIZE> v, float k){
    std::array<float,STATE_VEC_SIZE> res=v;
    for(size_t x = 0; x < v.size(); ++x){
        res[x] = v[x] * k;
    }
    return res;
}

float flightSim::runSim(){
    float apogeeAlt = 0;
    while(simState[1]>=0){
        // Velocity magnitude for drag calculation
        float vmag = simState[1];
        float drag = curDrag(vmag,simState[0]);

        // Angle of attack (Assume rocket straight into wind)
        float angle_of_attack = 0;

        // Runge-Kutta 4
        std::array<float,STATE_VEC_SIZE> k1 = rocketDynamics(simState, drag);

        // k2 and k3 - recalculate at midpoint
        std::array<float,STATE_VEC_SIZE> S_mid = addVectorsElementwise(simState,multiplyVectorScalar(k1,dt/2));
        
        float drag_mid = curDrag(S_mid[1],S_mid[0]);

        std::array<float,STATE_VEC_SIZE> k2 = rocketDynamics(S_mid, drag_mid);

        S_mid = addVectorsElementwise(simState,multiplyVectorScalar(k2,dt/2));
        std::array<float,STATE_VEC_SIZE> k3 = rocketDynamics(S_mid,drag_mid);

        // k4 - recalculate at end point
        std::array<float,STATE_VEC_SIZE> S_end = addVectorsElementwise(simState,multiplyVectorScalar(k3,dt));
        float drag_end = curDrag(S_end[1], S_end[0]);

        std::array<float,STATE_VEC_SIZE> k4 = rocketDynamics(S_end, drag_end);
        
        // Update state
        std::array<float,STATE_VEC_SIZE> newState = addVectorsElementwise(simState, multiplyVectorScalar(addVectorsElementwise(k1,addVectorsElementwise(multiplyVectorScalar(k2,2),addVectorsElementwise(multiplyVectorScalar(k3,2),k4))),dt/6));
        simState = newState;

        // Check max altitude
        if(simState[0]>apogeeAlt){
            apogeeAlt = simState[0];
        }
    }
    return apogeeAlt;
}