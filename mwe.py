# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 18:31:08 2020

@author: Toma
"""

from pyfly import PyFly
from pid_controller import *
from math import *
import matplotlib.pyplot as plt


sim = PyFly("pyfly_config.json", "x8_param.mat")


sim.seed(0)

# Initial state
sim.reset(state={"roll": 0, "pitch": 0, "yaw": 0,
                 "omega_p": 0, "omega_q": 0, "omega_r": 0,
                 "velocity_u": 22, "velocity_v": 0, "velocity_w": 0,
                 "position_n": 0, "position_e": 0, "position_d": -100,
                 "aileron": 0, "elevator": 0, "throttle": 0.4})

"""
- way to set all states to zero
- modifying actuation to have pulling on lines
  - will use the aileron, elevator inputs
  - the difference will be in aerodynamics
- adding Guidance PID loop
"""

# Setting up controllers

sampleFactor = 100 # dtG = 10dtC

pidGuidance = PIDGuidance(sim.dt*sampleFactor)
pidControl = PIDControl(sim.dt)


for step_o in range(40):
    
    # GUIDANCE LOOP
    
    homeHead = atan((2500-sim.state["position_e"].value)/(2500-sim.state["position_n"].value))
    

    heading = atan(sim.state["yaw"].value)
    
    
    
    pidGuidance.set_reference(homeHead)
    
    if step_o != 0:
        delphi, e_head_prev = pidGuidance.get_action(sim.state["yaw"].value, e_head_prev)
    else:
        delphi, e_head_prev = pidGuidance.get_action(sim.state["yaw"].value)
    
    phiSetpoint = sim.state["roll"].value + delphi
    
    phi_max = np.radians(30)
    
    if phiSetpoint > phi_max:
        phiSetpoint = phi_max
    elif phiSetpoint < -phi_max:
        phiSetpoint = -phi_max
    
    
    pidControl.set_reference(phiSetpoint, theta=0, va=22)
    
                             
    # CONTROL LOOP
    for step_i in range(int(sampleFactor)):
    
        phi = sim.state["roll"].value
        theta = sim.state["pitch"].value
        Va = sim.state["Va"].value
        omega = [sim.state["omega_p"].value, sim.state["omega_q"].value, sim.state["omega_r"].value]
    
        action = pidControl.get_action(phi, theta, Va, omega)
        success, step_info = sim.step(action)
    
        if not success:
            print('Simulation fail at step {}, {}'.format(step_o, step_i))
            break

sim.render(block=True)

# Plot trajectory
plt.figure()
plt.plot(sim.state["position_e"].history, sim.state["position_n"].history)
plt.axis('equal')
plt.show()