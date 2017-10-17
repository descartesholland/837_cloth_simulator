#include "timestepper.h"

#include <cstdio>
#include <iostream>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
    std::cerr << "ForwardEuler::takeStep()...\n";
    std::vector<Vector3f> currentState = particleSystem->getState();
    std::vector<Vector3f> currentStateEval = particleSystem->evalF(currentState);
    
    std::cerr << "\tInitial state (" << currentState.size() << ") : \n";
    for (Vector3f i: currentState)
        std::cout << "\t" << i.x() << " " << i.y() << " " << i.z() << "\n";
    
    std::vector<Vector3f> newState;
    for(int i = 0; i < currentState.size(); ++i) {
        newState.push_back(currentState[i] + stepSize * currentStateEval[i] );
    }
    particleSystem->setState(newState);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
//    std::cerr << "Trapezoidal::takeStep()...\n";
    std::vector<Vector3f> currentState = particleSystem->getState();
    std::vector<Vector3f> f0 = particleSystem->evalF(currentState);
    
    std::vector<Vector3f> f1pre; // X(t) + h * f0
    for(int i = 0; i < currentState.size(); ++i) {
        f1pre.push_back( currentState[i] + stepSize * f0[i] );
    }
    
    std::vector<Vector3f> f1 = particleSystem->evalF(f1pre);
    
    std::vector<Vector3f> newState;
    for(int i = 0; i < currentState.size(); ++i) {
        newState.push_back(currentState[i] + stepSize/2.0 * (f0[i] + f1[i]));
    }
    particleSystem->setState(newState);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
}

