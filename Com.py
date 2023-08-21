import serial
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23,GPIO.OUT)
GPIO.output(23,GPIO.HIGH)


krs = serial.Serial('/dev/ttyAMA0', baudrate=115200, parity=serial.PARITY_EVEN, timeout=0.5)

tx_commands = [
    [0x84, 0x3A, 0x4C],  # 4
    [0x85, 0x3A, 0x4C],  # 5
    [0x88, 0x3A, 0x4C],  # 8
    [0x89, 0x3A, 0x4C],  # 9
    [0x8C, 0x3A, 0x4C],  # 12
    [0x8D, 0x3A, 0x4C],  # 13
    [0x8E, 0x3A, 0x4C],  # 14
    [0x8F, 0x3A, 0x4C],  # 15
    [0x90, 0x3A, 0x4C],  # 16
    [0x91, 0x3A, 0x4C],  # 17
    [0x92, 0x3A, 0x4C],  # 18
    [0x93, 0x3A, 0x4C],  # 19
    [0x94, 0x3A, 0x4C],   # 20
    [0x95, 0x3A, 0x4C]   # 21
]



#for cmd in tx_commands:  
 #   redata =  krs.write(cmd)
  #  print("Done")
   # time.sleep(2)

#Read the current position of the servo Because ICS3.5 does not implement a command to get the current position Use the angle that can be obtained in reply when sending a position command
def krs_getPos_CMD(servo_id):
   
    #To get the current position with the return value of the position command, get the return value from the servo at position 0.
    Value = krs_setPos_CMD(servo_id, 0)
        
    #Send the same position as the obtained position to the servo in the free state and return it to the torque-on state
    Value = krs_setPos_CMD(servo_id, Value)
    
    return Value
    
#It operates by specifying the angle of the servo specified by the number.
def krs_setPos_CMD(joint):
	
	
	for cmd in joint:
		time_st = time.time()
		servo_id,pos = cmd
		txCmd = [0x80 | servo_id,   #Add the ID number to the position command 0x80 to form a header.
             pos >> 7 & 0x7f,   #Split position data into 2 bytes
             pos & 0x7f]
     
		krs.write(txCmd)
		time.sleep(0.1)
		#rxCmd = krs.read()
		#print(rxCmd)
		#r = ''.join(format(byte,'08b') for byte in rxCmd)
		#print(r)
		time_t = time.time()
		print((time_t - time_st))
	
	
	
#    if len(rxCmd) == 0:
#    return 0

        
#    else:
#        return (rxCmd[4] << 7) + rxCmd[5]
        
     
joint_cmd = [(9,SECOND) for SECOND in range(4000,10000,30)]
krs_setPos_CMD(joint_cmd)	
