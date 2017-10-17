#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

const int NUM_PARTICLES = 4;
const float GRAVITY = -9.8f;
const Vector3f GRAVITY_VECTOR = Vector3f(0, -9.8f, 0);
const float PARTICLE_MASS = 1.0f;
const float DRAG_COEFFICIENT = 0.3f;
const float SPRING_STIFNESS = 32.0f;
const float REST_LENGTH = 0.15f;
const float epsilon = 0.00001;

PendulumSystem::PendulumSystem()
{
    // Add particles for simple pendulum
    
    m_vVecState.push_back(Vector3f(-0.5, 1, 0));
    m_vVecState.push_back(Vector3f(0,0,0));
    for(int i = 0; i < NUM_PARTICLES-1; ++i) {
        m_vVecState.push_back(Vector3f(-0.5, rand_uniform(-0.5f, 0.5f), 0));
        m_vVecState.push_back(Vector3f(0,0,0));
    }
    
    // Springs represented by a tuple ( particleIndex1, particleIndex2, stiffness, restLength )
    springs.push_back(Vector4f(0, 2, SPRING_STIFNESS, REST_LENGTH));
    springs.push_back(Vector4f(2, 4, SPRING_STIFNESS, REST_LENGTH));
    springs.push_back(Vector4f(4, 6, SPRING_STIFNESS, REST_LENGTH));

    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
}

std::vector<Vector4f> PendulumSystem::getSpringsForParticle(int particleIndex) {
    std::vector<Vector4f> ans;
    for(int i = 0; i < springs.size(); ++i) {
        if(springs[i].x() == particleIndex || springs[i].y() == particleIndex)
            ans.push_back(springs[i]);
    }
//    std::cout << "getStringsForParticle(" << particleIndex << ") returning " << ans.size() << " items. \n";
    return ans;
}

void printVecVec(std::vector<Vector3f> vec) {
    for(Vector3f v : vec) {
        std::cout << "\t(" << v.x() << ", " << v.y() << ", " << v.z() << ")\n";
    }
}

std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;

    //  - gravity
    std::vector<Vector3f> forceGrav;
    for(int i = 0; i < state.size(); i+=2)
        forceGrav.push_back(PARTICLE_MASS * GRAVITY_VECTOR);
    
    //  - viscous drag
    std::vector<Vector3f> forceViscous;
    for(int i = 0; i < state.size(); i+=2) {
        float _tempX = DRAG_COEFFICIENT * state[i+1].x();
        float _tempY = DRAG_COEFFICIENT * state[i+1].y();
        float _tempZ = DRAG_COEFFICIENT * state[i+1].z();

        forceViscous.push_back( Vector3f(
                                std::abs(_tempX) < epsilon ? 0 : -_tempX,
                                std::abs(_tempY) < epsilon ? 0 : -_tempY,
                                std::abs(_tempZ) < epsilon ? 0 : -_tempZ));
    }
    
    //  - springs
    std::vector<Vector3f> forceSpring;
    for(int i = 0; i < state.size(); i+=2) {
        // get springs that are connected to particle i
        std::vector<Vector4f> iSprings = getSpringsForParticle(i);
        
        // sum forces from each spring
        Vector3f aggSpringForce = Vector3f(0,0,0);
        
        for(int ii = 0; ii < iSprings.size(); ++ii) {
            
            // -k * ( ||d|| - r ) / ||d|| * d
            // where d = x_1 - x_2
            Vector3f d;
            if( i == iSprings[ii].x() ) // current particle is 'source' of spring
                d = state[iSprings[ii].x()] - state[iSprings[ii].y()];
            else  // current particle is the 'destination' of spring
                d = state[iSprings[ii].y()] - state[iSprings[ii].x()];
            
            float _temp = -iSprings[ii].z() * (d.abs() - iSprings[ii].w()); // .w() returns restLength
            aggSpringForce+= ( std::abs(_temp) < epsilon ? 0 : _temp ) * d.normalized();
        }
        
        forceSpring.push_back(aggSpringForce);
    }
    
    std::vector<Vector3f> netForce;
    for(int i = 0; i < forceGrav.size(); ++i) {
        // Fix 0th particle (clear forces)
        if( i == 0)
            netForce.push_back(Vector3f(0,0,0));
        else
            netForce.push_back(forceGrav[i] + forceViscous[i] + forceSpring[i]);
    }
    
    for(int i = 0; i < netForce.size(); ++i) {	        
        f.push_back(state[2*i + 1]); // v
        f.push_back(netForce[i] / PARTICLE_MASS); // force / mass = accel.
    }

    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    for(int i = 0; i < m_vVecState.size(); i+=2) {
        Vector3f pos = m_vVecState[i];
        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(0.075f, 10, 10);
    }
}
