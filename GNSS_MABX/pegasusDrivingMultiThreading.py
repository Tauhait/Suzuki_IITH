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

waypoints = pd.read_csv('waypoints.csv', dtype='str')
waypoints = [[np.float64(val) for val in row] for row in waypoints.values.tolist()]

wp_threshold = 1.111395e5

def set_angle(m_nAngle, deltaAngle):
    m_nAngle = m_nAngle + deltaAngle
    gearRatio = 17.75
    if (40 * gearRatio < m_nAngle):
        m_nAngle = 40 * gearRatio
    elif (-40 * gearRatio > m_nAngle):
        m_nAngle = -40 * gearRatio
    l_usAngle = (m_nAngle / gearRatio - (-65.536)) / 0.002

    H_Angle = (int)(l_usAngle) >> 8
    L_Angle = (int)(l_usAngle) & 0xff
    return H_Angle, L_Angle

def calc_checksum(msg):
    cs = 0
    for m in msg:
        cs += m
    cs = (0x00 - cs) & 0x000000FF
    cs = cs & 0xFF
    return cs

def set_speed(speed):
    speed = speed * 128
    H_speed = (int)(speed) >> 8
    L_speed = (int)(speed) & 0xff
    return H_speed, L_speed

def get_msg_to_mabx(speed, m_nAngle, angle, flasher, counter):
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
    return 1 if angle > 90 else 2 if angle < -100 else 0

def calculate_steer_output(currentLocation, wp, heading):
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


def tcp_client(lock, udp_stack):
    while True:        
        print(f"GNSS thread running and inside loop with lock acquired")        
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
            udp_stack.append(data_tuple)
            lock.release()
        
        time.sleep(0.05)


def udp_client(lock, udp_stack):
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
            else:
                steer_output = 0
        
        message = get_msg_to_mabx(const_speed, steer_output, 0, 0, counter)
        mabx_socket.sendto(message, mabx_addr)


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
        
        
