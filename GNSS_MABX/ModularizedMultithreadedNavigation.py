#Function Descriptions and Comments
#Importing the required modules
import socket
import threading
import select
import time, math
import numpy as np
import pandas as pd
from collections import deque

gnss_IP = '192.168.50.11'  # replace with the IP address or hostname of the server you're connecting to
gnss_PORT = 3006  # replace with the port number you're connecting to

mabx_IP = "192.168.50.1"  # Mabx IP for sending from Pegasus
mabx_PORT = 30000  # Mabx port for sending from Pegasus

BUFFER_SIZE = 4096  # packet size
local_interface = 'enx806d970aca5b' #'enx806d970ac9f9' #enx806d970aca5b

mabx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mabx_addr = (mabx_IP, mabx_PORT)
mabx_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, local_interface.encode())

gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
gnss_socket.connect((gnss_IP, gnss_PORT))

#Reading the waypoints from CSV file, converting them to a nested list and defining a threshold distance for the waypoint
waypoints = pd.read_csv('waypoints.csv', dtype='str')
waypoints = [[np.float64(val) for val in row] for row in waypoints.values.tolist()]
wp_threshold = 1.111395e5

traversed_waypoints = str("traversed_waypoints" + ".csv")

#Function to set the angle of the vehicle
def set_angle(m_nAngle, deltaAngle):
	"""
	Set the angle of the vehicle by adding the delta angle to the current angle.
	Parameters:
	m_nAngle (int): Current angle of the vehicle in degrees.
	deltaAngle (int): Angle to be added to the current angle.

	Returns:
	tuple: A tuple containing two values H_Angle and L_Angle which are the higher and lower bytes of the angle value
	       respectively.
	"""
	# Adding delta angle to the current angle and setting a gear ratio
	m_nAngle = m_nAngle + deltaAngle
	gearRatio = 17.75
	# Limiting the angle value to within -40 to 40 degrees
	if (40 * gearRatio < m_nAngle):
	    m_nAngle = 40 * gearRatio
	elif (-40 * gearRatio > m_nAngle):
	    m_nAngle = -40 * gearRatio
	# Converting the angle value to a servo pulse width and getting higher and lower bytes of the value
	l_usAngle = (m_nAngle / gearRatio - (-65.536)) / 0.002
	H_Angle = (int)(l_usAngle) >> 8
	L_Angle = (int)(l_usAngle) & 0xff
	return H_Angle, L_Angle

#Function to calculate the checksum of the message
def calc_checksum(msg):
	"""
	Calculate the checksum of the message.

	vbnet
	Copy code
	Parameters:
	msg (bytearray): The message to be used for calculating the checksum.

	Returns:
	int: The checksum value calculated for the message.
	"""
	# Initializing the checksum value to zero and adding all the message bytes to it
	cs = 0
	for m in msg:
	    cs += m
	# Taking the two's complement of the checksum value and taking the lower 8 bits of the result as the checksum
	cs = (0x00 - cs) & 0x000000FF
	cs = cs & 0xFF
	return cs

def set_speed(speed):
    """
    This function takes a speed value as input and returns two values H_speed and L_speed.
    """
    speed = speed * 128
    H_speed = (int)(speed) >> 8
    L_speed = (int)(speed) & 0xff
    return H_speed, L_speed

def get_msg_to_mabx(speed, m_nAngle, angle, flasher, counter):
    """
    This function takes five inputs:
        - speed: a speed value
        - m_nAngle: an angle value
        - angle: an angle value
        - flasher: a flasher value
        - counter: a counter value
    It then calculates and sets H_Angle, L_Angle, H_Speed, and L_Speed, and returns a message to MABX.
    """
    H_Angle, L_Angle = set_angle(m_nAngle, -1*angle)
    H_Speed, L_Speed = set_speed(speed)

    msg_list = [1, counter, 0, 1, 52, 136, 215, 1, H_Speed, L_Speed, H_Angle, L_Angle, 0, flasher, 0, 0, 0, 0]

    msg_list[2] = calc_checksum(msg_list)
    message = bytearray(msg_list)
    print("Speed: ", message[8], message[9])
    print("Angle: ", message[10], message[11])
    print("===============================================================================")
    return message

def get_flasher(angle):
    """
    This function takes an angle value as input and returns a flasher value based on the angle.
    """
    return 1 if angle > 90 else 2 if angle < -100 else 0

def calculate_steer_output(currentLocation, wp, heading):
    """
    This function takes three inputs:
        - currentLocation: a list of two float values representing the current location
        - wp: a waypoint value
        - heading: a heading value
    It then calculates the steer output based on the current location, waypoint, and heading, and returns the steer output value.
    """
    off_y = - currentLocation[0] + waypoints[wp][0]
    off_x = - currentLocation[1] + waypoints[wp][1]

    # calculate bearing based on position error
    bearing_ppc = 90.00 + math.atan2(-off_y, off_x) * 57.2957795 

    # convert negative bearings to positive by adding 360 degrees
    if bearing_ppc < 0:
        bearing_ppc += 360.00

    # calculate the difference between heading and bearing
    bearing_diff = heading - bearing_ppc

    # normalize bearing difference to range between -180 and 180 degrees
    if bearing_diff < -180:
        bearing_diff = bearing_diff + 360

    if bearing_diff > 180:
        bearing_diff = bearing_diff - 360

    steer_output = 750 * np.arctan(-1 * 2 * 3.5 * np.sin(np.pi * bearing_diff / 180) / 8)
    
    return steer_output

# Function to handle TCP socket connection and update UDP stack with GNSS data
def tcp_client(lock, udp_stack):
    '''
    Description: This function runs in an infinite loop and receives GNSS data through a TCP socket connection.
                 If GNSS data is received, the function decodes the received message, extracts latitude, longitude,
                 and heading information, creates a tuple with this information, acquires the lock, and appends the
                 tuple to the UDP stack.
    
    Input:
    lock - A threading.Lock() object to synchronize thread access to shared resources
    udp_stack - A deque object to store GNSS data tuples
    
    Output: None
    '''
    while True:       
        data = gnss_socket.recv(BUFFER_SIZE)
        if data.startswith(b'#INSPVAA,ICOM6'):
            data_str = hex_data.decode('utf-8')
            data_list = data_str.split(';', 1)
            data_list = [chunk for chunk in data_list[1].split(',')]
            lat = float(data_list[2])
            lng = float(data_list[3])
            heading = float(data_list[10])
            data_tuple = (lat, lng, heading)

            lock.acquire()
            print(f"GNSS thread running and inside loop with lock acquired") 
            udp_stack.append(data_tuple)
            lock.release()
        
        time.sleep(0.05)

# Function to handle UDP socket connection and send data to the MABX device
def udp_client(lock, udp_stack):
    '''
    Description: This function runs in an infinite loop and sends data to the MABX device through a UDP socket connection.
                 If the UDP stack has GNSS data, the function acquires the lock, pops the GNSS data tuple from the stack,
                 releases the lock, calculates the steering output, updates the waypoint, and creates a message for the MABX
                 device with the steering output and constant speed. If the UDP stack is empty, the function creates a message
                 with the constant speed and zero steering output.
    
    Input:
    lock - A threading.Lock() object to synchronize thread access to shared resources
    udp_stack - A deque object to store GNSS data tuples
    
    Output: None
    '''
    const_speed = 10
    wp = 0
    steer_output = 0
    counter = 0
    while True:
        time.sleep(0.06)
        counter = (counter + 1) % 256
        while not udp_stack:
            message = get_msg_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)

        if udp_stack:
            lock.acquire()
            print(f"MABX thread running inside infinite loop with lock acquired and Stack has data")
            lat, lng, heading = udp_stack.pop()
            lock.release()
            print(f"\n\n\n##################################################################\n")
            print(f"Latitude: {lat}")
            print(f"Longitude: {lng}")
            print(f"Azimuth: {heading}")

            currentLocation = [lat, lng]

            if ((np.linalg.norm(np.array(currentLocation) - waypoints[len(waypoints) - 1]) * wp_threshold) > 1):
                steer_output = calculate_steer_output(currentLocation, wp, heading)
                steer_output = steer_output * -1.0
                print(f"steer_output: {steer_output}")

                if (wp < len(waypoints) and 
                    ((np.linalg.norm(np.array(currentLocation) - waypoints[wp]) * wp_threshold) < 6)):
                    wp = wp + 1
                    traversed_waypoints.write(str(currentLocation[0])+","+str(currentLocation[1])+"\n")
					traversed_waypoints.flush()
           
            else:
                steer_output = 0
        
        message = get_msg_to_mabx(const_speed, steer_output, 0, 0, counter)
        mabx_socket.sendto(message, mabx_addr)


''' 
The code sets up two threads - tcp_thread and udp_thread - which execute the tcp_client and udp_client functions respectively. 
The lock is used to synchronize access to the udp_stack deque object shared between the threads.

The tcp_thread and udp_thread are started with tcp_thread.start() and udp_thread.start(), and 
then the main thread enters a loop where it sleeps for 1 second at a time until the program is interrupted by the 
user with a keyboard interrupt. Once the program is interrupted, the tcp_thread and 
udp_thread are joined to the main thread with tcp_thread.join() and udp_thread.join() respectively to wait for the threads to complete.
The code is the main entry point for running a program that uses two threads, tcp_thread and udp_thread, 
to handle communication with a server using TCP and UDP protocols. 
The lock object is used to synchronize access to a shared resource, a deque object named udp_stack.

The tcp_thread and udp_thread are created using threading.
Thread class, with target parameter set to the respective client functions and args parameter set to (lock, udp_stack) tuple, 
to pass the shared resources to the threads.

The main thread will run an infinite loop, with a sleep of 1 second on each iteration, 
until a keyboard interrupt (Ctrl+C) is received. Once a keyboard interrupt is detected, 
both the threads will be joined using join method of the Thread class to wait for them to complete before exiting the program.

Therefore, the purpose of this code is to create two threads, tcp_thread and udp_thread, 
to handle communication with a server using TCP and UDP protocols, respectively, and 
to synchronize access to a shared resource using a lock.
'''
if __name__ == '__main__':
    lock = threading.Lock()
    udp_stack = deque()

    tcp_thread = threading.Thread(target=tcp_client, args=(lock, udp_stack))
    udp_thread = threading.Thread(target=udp_client, args=(lock, udp_stack))

    tcp_thread.start()
    udp_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        tcp_thread.join()
        udp_thread.join()

