import socket
import time

#Byte order - Big Endian
#0th byte= id
#1st byte = rolling counter
#2nd byte = checksum(optional)
###############################20220119=01 34 88 D7 layout
#3rd byte = layout(optional) 
#4th byte = layout(optional)
#5th byte = layout(optional)
#6th byte = layout(optional)
#7th byte = Mode(Optional)
#8th byte = speed
#9th byte = speed
#10 th byte = tire angle
#11th byte = tire angle
#12th byte = shift
#13th byte = flasher
#14th byte = sequence number
#15th byte = ExeTime
#16th byte = ExeTime
#17th byte = Emergency Flag


#Steps for AV
#doors should be close
#if some errors are showing restart the vehicle two three times

#1. VCAN to PCAN change
#2. Release leftmost brake
#3. Change gear from Parking to Drive
#4. Press ACC standby button
#5. Set ACC button by togging it down


#test_ip = "127.0.0.1"
#test_port = 20001

UDPsend_IP = "192.168.50.1" # Mabx IP for sending from Pegasus
UDPsend_PORT = 30000 # Mabx port for sending from Pegasus 

sockSend = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
counter = 0
m_nAngle = 0

def setAngle(m_nAngle, deltaAngle):
    m_nAngle = m_nAngle + deltaAngle
    gearRatio = 17.75
    if(40*gearRatio < m_nAngle):
        m_nAngle = 40*gearRatio
    elif (-40*gearRatio > m_nAngle):
        m_nAngle = -40*gearRatio
    l_usAngle = (m_nAngle / gearRatio - (-65.536)) / 0.002
    H_Angle = (int)(l_usAngle) >> 8
    L_Angle = (int)(l_usAngle) & 0xff
    return H_Angle, L_Angle

def getCheckSum(msg):
    cs = 0
    for m in msg:
        cs += m
    cs = (0x00 - cs) & 0x000000FF
    cs = cs & 0xFF
    return cs

def setSpeed(speed):
    speed = speed * 128
    H_speed = (int)(speed) >> 8
    L_speed = (int)(speed) & 0xff
    return H_speed, L_speed


def sendToMabx(speed, m_nAngle, angle, flasher, counter): 
    speed = max(speed, 30)
    H_Angle, L_Angle = setAngle(m_nAngle, -1 * angle)    
    H_Speed, L_Speed = setSpeed(speed)    
    
    msg_list = [1, counter, 0, 1, 52, 136, 215, 1, H_Speed, L_Speed, H_Angle, L_Angle, 0, flasher, 0, 0, 0, 0]
    
    msg_list[2] = getCheckSum(msg_list)
    
    message = bytearray(msg_list)
    
    print("RC: ", message[1])
    print("Mode: ", message[7])
    print("Speed: ", message[8], message[9])
    print("Angle: ", message[10], message[11])
    print("Checksum: ", message[2])
    print("Flasher: ", message[13])
    print(" ".join(hex(m) for m in message))
    print("===============================================================================")
    addr = (UDPsend_IP, UDPsend_PORT)
    sockSend.sendto(message, addr)
init_angle = 5
delta_angle = 10
s = 5
while(True):
    sendToMabx(s, init_angle, delta_angle, 1, counter)
    init_angle = delta_angle
    delta_angle += 25
    s += 1
    counter = (counter + 1) % 256
    time.sleep(0.02)#20ms


sockSend.close()
