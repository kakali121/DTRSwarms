import pygame
import socket
import time
import sys
import paho.mqtt.client as mqtt

IP_ADDRESS = '192.168.0.212'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')
 
# initialising pygame
pygame.init()
 
# creating display
display = pygame.display.set_mode((300, 300))

def Forward():
    command = 'CMD_MOTOR#1300#1300#1300#1300\n'
    s.send(command.encode('utf-8'))
    
def Backward():
    command = 'CMD_MOTOR#-1000#-1000#-1000#-1000\n'
    s.send(command.encode('utf-8'))
    
def Left_Turn():
    command = 'CMD_MOTOR#-1300#-1300#1500#1500\n'
    s.send(command.encode('utf-8'))
    
def Right_Turn():
    command = 'CMD_MOTOR#1500#1500#-1300#-1300\n'
    s.send(command.encode('utf-8'))
    
def Terminate():
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))


try:
    start = time.time()
    end = time.time()
    elapsed = int(end - start)

    # creating a running loop
    while elapsed < 600:
        temp = int(end - start)
        if not temp == elapsed:
            elapsed = temp
            print("Time elapsed", elapsed)
        # creating a loop to check events that are occurring
        for event in pygame.event.get():
            # checking if keydown event happened or not
            if event.type == pygame.KEYDOWN:
                # Forward
                if event.key == pygame.K_w:
                    Terminate()
                    print("Moving Forward...")
                    Forward()
                # Backward
                elif event.key == pygame.K_s:
                    Terminate()
                    print("Moving Backward...")
                    Backward()
                # Left Turn
                elif event.key == pygame.K_a:
                    Terminate()
                    print("Turning Left...")
                    Left_Turn()
                # Right Turn
                elif event.key == pygame.K_d:
                    Terminate()
                    print("Turning Right...")
                    Right_Turn()
                elif event.key == pygame.K_t:
                    print("Terminating...")
                    Terminate()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        end = time.time()
    Terminate()
    
except KeyboardInterrupt:
    Terminate()
    
Terminate()
# Close the connection
s.shutdown(2)
s.close()
