// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <iostream>
void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    // refernce variables
    double K_d, K_p;
    double Feq, target_altitude,h;
//    double del, e, Pout, h;
//    Add target_altitude to variable list for 3e info paper


//    // proportional control
    target_altitude = 500;
    h = position.abs() - MARS_RADIUS;
    
    // initialise variables
    
    K_p = 0.091;
    K_d = 2*K_p;
    Feq = 747.1;
    throttle = (Feq/1121)+ K_p*(target_altitude-h) + K_d*((target_altitude-h)/delta_t);
//    throttle = Feq/MAX_THRUST;
//    del = 0.8;
//
//    //error
//    e = -( 0.5 + K_h*h + velocity*position.norm());
//    Pout = K_p*e;
//
//    //autopilot
//    if (Pout <= -del){
//        throttle = 0;
//    }
//    if (-del < Pout < 1-del){
//        throttle = del + Pout;
//    }
//    if (Pout>= 1 - del){
//        throttle = 1;
//    }

    
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
    vector3d thr, drag, acceleration_lander, new_position, gravity, new_velocity;
   // va_list t_list, v_list, p_list;
    static vector3d previous_position;
    double density, land_mass, land_area, chute_area;
   
    //Lander
    density = atmospheric_density(position);
    land_area = M_PI*pow(2.0, LANDER_SIZE);
    chute_area = 20*pow(2.0, LANDER_SIZE);
    land_mass = UNLOADED_LANDER_MASS +fuel*FUEL_CAPACITY*FUEL_DENSITY;
    
    
    //FORCES
  
    gravity = -(GRAVITY*MARS_MASS*land_mass)/(position.abs2())*position.norm();
    drag = -0.5*density*DRAG_COEF_LANDER*land_area*velocity.abs2()*velocity.norm();
    if (parachute_status == DEPLOYED){
        drag = (-0.5 * density * DRAG_COEF_CHUTE * (velocity.abs2()) * chute_area*velocity.norm());
    }
    //thrust
    thr = thrust_wrt_world() + 100.0*cos(0.1*simulation_time)*position.norm();
    
    
    
    
    
  
    //acceleration
    acceleration_lander = (gravity+drag+thr)/land_mass;
    
    //Verlet
        
    if (simulation_time == 0.0){
        new_position = position + velocity*delta_t ;
        new_velocity = velocity + acceleration_lander*delta_t;
            }
    else {
        new_position = 2*position-previous_position+ (delta_t*delta_t)*acceleration_lander;
        new_velocity = (1/delta_t)*(new_position-position);
            }
        
    previous_position = position;
    position = new_position;
    velocity = new_velocity;
    
//    double speed = new_velocity.abs();
    
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();
    
  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}





void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    position = vector3d(0.0, -(MARS_RADIUS + 500), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 7:
    position = vector3d(0.0, -(MARS_RADIUS + 510), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 8:
    position = vector3d(0.0, -(MARS_RADIUS + 700), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 9:
    break;

  }
}
