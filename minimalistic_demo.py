import time
import numpy as np
import time

import zmq

import state_representation as sr
from controllers import create_joint_controller, CONTROLLER_TYPE

from network_interfaces.zmq import network

def main(state_uri, command_uri):

    # Create ZMQ socket. False means we should connect to the IP -> NEEDS TO USE TRUE IN FRANKA-LWI TO BIND IP ON THE OTEHR SIDE
    context = zmq.Context(1)
    subscriber = network.configure_subscriber(context, state_uri, False)
    publisher = network.configure_publisher(context, command_uri, False)

    # Create controller and command object
    command = network.CommandMessage()
    command.control_type = [4]

    nb_joints = 7
    ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, nb_joints)
    desired_state = sr.JointState("test", nb_joints)
    current_state = sr.JointState("test", nb_joints)

    ctrl.set_parameter(sr.Parameter("stiffness", 6, sr.StateType.PARAMETER_DOUBLE))

    # Create target object
    targetPose = np.array([-0.12596654, 0.57328627, -0.89052628, -1.99565171,  0.45342126,  2.38429479, 0.42424838])
    desired_state.set_positions(targetPose)

    # Move to target with joint impedance controller
    initFlag = 1
    time_period = 10e-3
    while True:
        state = network.receive_state(subscriber)
        if state:
            # This happens once to setup the command object with the right parameters from the received robot state
            if initFlag:     
                command.joint_state = sr.JointState().Zero(state.joint_state.get_name(), state.joint_state.get_names())            
                command.joint_state = state.joint_state
                
                initFlag = 0

            # Set current state and compute control torques
            current_state.set_positions(state.joint_state.get_positions())
            current_state.set_velocities(state.joint_state.get_velocities())

            commanded_torque = ctrl.compute_command(desired_state, current_state)

            command.joint_state.set_torques(commanded_torque.get_torques())
            network.send_command(command, publisher)
        time.sleep(time_period)


if __name__ == "__main__":

    # Put here the IP of the machine running franka-lightweight-interface
    main("128.178.145.79:1601", "128.178.145.79:1602")
