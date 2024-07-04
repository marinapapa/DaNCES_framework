#ifndef FLIGHT_CONTROL_HPP_INCLUDED
#define FLIGHT_CONTROL_HPP_INCLUDED

#include <iostream>
#include <math.h>
#include <model/json.hpp>


namespace model {
  namespace flight_control {

    template <typename Agent>
    void integrate_motion(Agent* self)
    {
      const float hdt = 0.5f * Simulation::dt(); // [tick]

	    // Cruise speed control as Drag
	    const float dv_c = (self->sa.cruiseSpeed - self->speed);    // change in speed for cruise speed control [m / tick]
	    const float lF = self->sa.w * dv_c * self->ai.bodyMass;     // linear force to cruise speed on linear direction (magnitude)
 	    self->steering += lF * self->dir;

      // calculate forces
      vec3 vel(self->speed * self->dir); // velocity vector
  	  auto force = self->steering;
	   
      // modified Euler method (a.k.a. midpoint method)
      vel += self->accel * hdt;                   // v(t + dt/2) = v(t) + a(t) dt/2
      self->pos += vel * Simulation::dt();        // r(t + dt) = r(t) + v(t + dt/2) * dt
      self->accel = force / self->ai.bodyMass;    // a(t + dt) = F(t + dt)/m
      vel += self->accel * hdt;                   // v(t) = v(t + dt/2) + a(t + dt) dt/2

      // clip speed & integrate
      self->speed = glm::length(vel);
      self->dir = math::save_normalize(vel, self->dir);
      self->speed = glm::clamp(self->speed, self->ai.minSpeed, self->ai.maxSpeed);
    }

  }
}

#endif