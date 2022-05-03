import argparse
from os import stat
import time
import numpy as np
import time
import random

import zmq

import state_representation as sr
from controllers import create_joint_controller, CONTROLLER_TYPE
from robot_model import Model

import roboticstoolbox as rtb

from network_interfaces.zmq import network
import robot_model


def main(state_uri, command_uri):
    panda_model = robot_model.Model("panda", "panda-model/panda_arm.urdf")

    context = zmq.Context(1)
    subscriber = network.configure_subscriber(context, state_uri, False)
    publisher = network.configure_publisher(context, command_uri, False)

    command = network.CommandMessage()
    command.control_type = [4]

    while 1:
        state = network.receive_state(subscriber)
        if state:
            command.joint_state = sr.JointState().Zero(state.joint_state.get_name(), state.joint_state.get_names())            
            command.joint_state = state.joint_state
            break
    
    initFlag = 1
    
    nIter = 700
    time_period = 2e-3

    nb_joints = 7

    ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, nb_joints)
    desired_state = sr.JointState("test", nb_joints)
    feedback_state = sr.JointState("test", nb_joints)



    tf = 3
    nTraj = 1
    # data = np.zeros((22, len(tTraj)*nTraj))
    dataset = []

    for iTraj in range(nTraj):

        # -------------- Going to the initial pose -------------
        ctrl.set_parameter(sr.Parameter("stiffness", 6, sr.StateType.PARAMETER_DOUBLE))
        initPose = np.array([-0.12596654, 0.57328627, -0.89052628, -1.99565171,  0.45342126,  2.38429479, 0.42424838])
        desired_state.set_positions(initPose)
        error = np.abs(np.linalg.norm(state.joint_state.get_positions() - initPose))

        while error > 0.5:
            state = network.receive_state(subscriber)
            if state:

                feedback_state.set_positions(state.joint_state.get_positions())
                feedback_state.set_velocities(state.joint_state.get_velocities())

                commanded_torque = ctrl.compute_command(desired_state, feedback_state)

                error = np.abs(np.linalg.norm(state.joint_state.get_positions() - initPose))               


                command.joint_state.set_torques(commanded_torque.get_torques())
                network.send_command(command, publisher)
            time.sleep(time_period)


        # # -------------- torque sinus -------------
        time.sleep(2)

        timeZero = time.time() 
        velocity = state.joint_state.get_velocities()

        # while abs(velocity[0]) > 1e-6 or error <= 0.3:
        t = 0.0
        prev_command = 0.0
        friction = 0.0
        alpha = 0.02
        while t < 5:
            state = network.receive_state(subscriber)
            if state:

                t = time.time()- timeZero
                velocity = state.joint_state.get_velocities()
                error = np.abs(np.linalg.norm(state.joint_state.get_positions() - initPose))

                torque = np.zeros(7)
                if t > 0.0 and t < tf:
                    torque[0] = (1 - np.cos(2*np.pi/tf *t))*1

                # Computing gravity torque and friction
                current_joint = sr.JointPositions().Zero(state.joint_state.get_name(), state.joint_state.get_names())
                current_joint.set_positions(state.joint_state.get_positions())
                gravity_torque = panda_model.compute_gravity_torques(current_joint)

                new_friction = prev_command - state.joint_state.get_torques() + gravity_torque.get_torques()
                friction = alpha*new_friction + (1-alpha)*friction

                torque += 0.5*friction

                
                print(friction)

                
                prev_command = torque
                

                data = np.zeros(30)

                data[0] = t
                data[1:8] = state.joint_state.get_positions()
                data[8:15] = velocity
                data[15:22] = torque
                data[22:29] = state.joint_state.get_torques()
                data[29] = state.mass.get_value()[0,0]
                
                dataset.append(data)

                command.joint_state.set_torques(torque)
                network.send_command(command, publisher)
            time.sleep(time_period)





    np.savetxt("data_friction.txt", np.array(dataset).T)

    exit(0)


if __name__ == "__main__":


    main("128.178.145.79:1701", "128.178.145.79:1702")
