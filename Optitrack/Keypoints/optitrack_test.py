import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = (rotx, roty, rotz)


if __name__ == "__main__":
    clientAddress = "192.168.0.23"
    optitrackServerAddress = "192.168.0.4"
    ball_id = 311
    cam_id = 311

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    start = time.time()
    end = time.time()
    # file = open("balloon.txt", "a+")
    while is_running and int(end - start) < 6000:
        if 310 in positions:
            # last position
            end = time.time()
            time_elasped = int(end - start)
            # if time_elasped%5 == 0:
            # file.write(str(time_elasped) + " " + str(positions[robot_id]) + " " + str(rotations[robot_id]) + "\n")
            print('Time elapsed', time_elasped)
            # print('Ballon position', positions[ball_id])
            print('Camera position', positions[310])
            time.sleep(2)
        
    # file.close()
