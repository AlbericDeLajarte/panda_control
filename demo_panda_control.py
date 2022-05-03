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
    
    initFlag = 1
    
    nIter = 700
    data = np.zeros((15, nIter))
    time_period = 10e-3

    nb_joints = 7

    initPose = np.array([-0.12596654, 0.57328627, -0.89052628, -1.99565171,  0.45342126,  2.38429479, 0.42424838])
    finalPose = np.array([1.57880807,  0.01870928, -0.80292824, -1.50262885,  0.28996286,  1.78826753, 0.81954111])

    ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, nb_joints)
    desired_state = sr.JointState("test", nb_joints)
    feedback_state = sr.JointState("test", nb_joints)

    # -------------- Going to the initial pose -------------
    ctrl.set_parameter(sr.Parameter("stiffness", 6, sr.StateType.PARAMETER_DOUBLE))

    desired_state.set_positions(initPose)

    for n in range(nIter):
        state = network.receive_state(subscriber)
        if state:
            
            if initFlag:     
                command.joint_state = sr.JointState().Zero(state.joint_state.get_name(), state.joint_state.get_names())            
                command.joint_state = state.joint_state
                
                initFlag = 0

            feedback_state.set_positions(state.joint_state.get_positions())
            feedback_state.set_velocities(state.joint_state.get_velocities())

            commanded_torque = ctrl.compute_command(desired_state, feedback_state)
            # print(error)
            # print(commanded_torque.get_torques())
            #print("--------------")
            
            # Sinus loop
            # fControl = 0.3 #Hz
            # t = time.time() - timeZero
            # torque1 = 10*np.sin(2*np.pi*fControl*t)
            # torque = np.array([torque1, 0, 0, 0, 0, 0, 0])
            # command.joint_state.set_torques(torque)


            command.joint_state.set_torques(commanded_torque.get_torques())
            network.send_command(command, publisher)
        time.sleep(time_period)


    initPose = state.joint_state.get_positions()
    # trajectory = np.linspace(initPose, finalPose, nIter).T
    trajectory = rtb.tools.trajectory.jtraj(initPose, finalPose, np.linspace(0, time_period*nIter, nIter))

    # -------------- Following trajectory -------------
    ctrl.set_parameter(sr.Parameter("stiffness", 190, sr.StateType.PARAMETER_DOUBLE))
    ctrl.set_parameter(sr.Parameter("damping", 10, sr.StateType.PARAMETER_DOUBLE))
    timeZero = time.time()

    prev_command = 0.0
    friction = 0.0
    alpha = 0.02

    for n in range(nIter):
        state = network.receive_state(subscriber)
        if state:

            ctrl.set_parameter(state.mass)
            
            feedback_state.set_positions(state.joint_state.get_positions())
            feedback_state.set_velocities(state.joint_state.get_velocities())

            desired_state.set_positions(trajectory.q.T[:, n])
            desired_state.set_velocities(trajectory.qd.T[:, n])
            desired_state.set_accelerations(trajectory.qdd.T[:, n])

            # commanded_torque = ctrl.compute_command(desired_state, feedback_state)
            # torque = commanded_torque.get_torques()
            cartesianError = (state.ee_state.get_position() - np.array([0.466, -0.1, 0.17]))

            torque = -5*np.array([0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.2]) * (np.linalg.pinv(state.jacobian.data())@np.concatenate((cartesianError, np.zeros(3))))

            # Computing gravity torque and friction
            current_joint = sr.JointPositions().Zero(state.joint_state.get_name(), state.joint_state.get_names())
            current_joint.set_positions(state.joint_state.get_positions())
            gravity_torque = panda_model.compute_gravity_torques(current_joint)

            new_friction = prev_command - state.joint_state.get_torques() + gravity_torque.get_torques()
            friction = alpha*new_friction + (1-alpha)*friction

            torque += 0*friction
            print(np.linalg.norm(cartesianError))

            prev_command = torque

            data[0, n] = time.time()- timeZero
            data[1:8, n] = trajectory.q.T[:, n]
            data[8:15, n] = state.joint_state.get_positions()

            command.joint_state.set_torques(torque)
            network.send_command(command, publisher)
        time.sleep(time_period)

    np.savetxt("data.txt", data)

    exit(0)


if __name__ == "__main__":


    main("128.178.145.79:1701", "128.178.145.79:1702")
