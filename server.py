import socket
import time
#03 4f 00 00 00 00 80 00 0a 00 00 00 00 00
UDP_IP = "192.168.50.2" # Pegasus IP for receiving
UDP_PORT = 51001 # Pegasus port for receiving
#0th byte= id
#1st byte = rolling counter
#2nd byte = checksum
#3rd byte = mode
#4th byte = 
#5th byte = 
#6th byte = 
#7th byte = 
#8th byte = shift position
#9th byte = flasher
#10 th byte = 
#11th byte = 
#12th byte = 
#13th byte = 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

numBytes =  [1, 1, 1, 1, 2, -1, 2, -1, 1, 1]
shift_dict = {0: "Shift in progress", 1: "L", 8: "R", 9: "N", 10: "P", 11: "D", 13: "S", 14: "M"}
flasher_dict = {0: "None", 1: "Left", 2: "Right", 3: "Both"}

while True:
    print("Ready to receive..")
    bytestr, addr = sock.recvfrom(1024)
    print("Receiving..")
    byte_array = bytearray(bytestr)
    hex_str = byte_array.hex()
    if(len(hex_str) != 28):
        continue
    i = 0
    j = 0
    decoded_val = []
    while(i < len(hex_str)):
        nBytes = numBytes[j]
        byteval = hex_str[i : i + 2*nBytes]
        if(nBytes == 2):
            byteval = byteval[2:4] + byteval[0:2]
        print("Byteval:",byteval)
        decimal = int(("0x" + byteval), 16)
        decoded_val.append(decimal)
        i = i + 2*nBytes
        j = j + nBytes

    print(bytestr)
    print("ID: ", decoded_val[0])
    print("Rolling counter: ", decoded_val[1])
    print("Checksum: ", decoded_val[2])
    print("Current Mode: ", decoded_val[3])
    print("Current Speed: ", decoded_val[4])
    print("Current Tire Angle: ", decoded_val[5])
    print("Current Shift: ", shift_dict[decoded_val[6]])
    print("Current Flasher: ", flasher_dict[decoded_val[7]])
    print("==================================================")
    time.sleep(0.5)
sock.close()
