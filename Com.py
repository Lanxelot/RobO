import serial
import time
import RPi.GPIO as GPIO

class ServoController:
    
     def __init__(self):
        self.krs = serial.Serial(serial_port='/dev/ttyAMA0', baudrate=115200, parity=serial.PARITY_EVEN, timeout=0.5)
        self.Arduino =  serial.Serial(serial_port='/dev/ttyACM0', baudrate=9600, parity=serial.PARITY_EVEN, timeout=0.5)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.OUT)
        GPIO.output(23, GPIO.HIGH)

    def __del__(self):
        self.krs.close()
        GPIO.cleanup()


    def execute_function(self,degrees):

        self.servo_value = []
        for y in degrees:
            self.input_min = -135
            self.input_max = 135

            # Output range: 3500 to 11500
            self.output_min = 3500
            self.output_max = 11500

            # Map the input degrees to output values
            self.output_range = self.output_max - self.output_min
            self.input_range = self.input_max - self.input_min

            self.scaled_value = (y - self.input_min) / self.input_range
            self.mapped_value = int(self.output_min + (self.scaled_value * self.output_range))
            if y == 0:
                self.mapped_value = 7500  
            self.servo_value.append(self.mapped_value)
        # print(self.servo_value)
            # print("{0}:{1}".format(y,self.mapped_value))

        self.target_servos = [4, 5, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
        self.joint_cmd = []
        # Map input values to output servo IDs
        x = 0
        for value in self.servo_value:
            if x <= 13:
                self.joint_cmd.append(tuple((self.target_servos[x],value)))
                x = x+1
            elif x > 13:
                x = 0
        
        self.krs_setPos_CMD(self.joint_cmd)

        
    def krs_setPos_CMD(self,joint_cmd):
        for tup in joint_cmd:
            servo_id,pos = tup
            print("{0},{1}".format(servo_id,pos))
            # txCmd = [0x80 | servo_id,
            #      pos >> 7 & 0x7f,
            #      pos & 0x7f]
            # self.krs.write(txCmd)
            # time.sleep(0.1)
            # rxCmd = self.krs.read()
            # if len(rxCmd) == 0:
            #     return 0
            # else:
            #     return (rxCmd[4] << 7) + rxCmd[5]

        

    def krs_getPos_CMD(self, servo_id):
        value = self.krs_setPos_CMD(servo_id, 0)
        value = self.krs_setPos_CMD(servo_id, value)
        return value

    def Home_Pose(self):
        Home_Pos_cmd = [(4,7500),(5,7500),(8,7500),(9,7500),(12,7500),(13,7500),(14,7500),(15,7500),(16,7500),(17,7500),(18,7500),(19,7500),(20,7500),(21,7500)]
        self.krs_setPos_CMD(Home_Pos_cmd)


    def IMU_sensor(self):
        s = [0,1]
        while True:
            read_serial=ser.readline()
            s[0] = str(int (ser.readline(),16))
            print (s[0])
            print (read_serial)

    def read_current(self,servo_id):
        txCmd = [0xA0 | servo_id,
                 0x03]
        self.krs.write(txCmd)
        time.sleep(0.1)
        rxCmd = self.krs.read(5)
        if len(rxCmd) == 0:
            return 0
        else:
            return rxCmd[4]

    def read_Temp(self,servo_id):
        txCmd = [0xA0 | servo_id,
                 0x04]
        self.krs.write(txCmd)
        time.sleep(0.1)
        rxCmd = self.krs.read(5)
        if len(rxCmd) == 0:
            return 0
        else:
            return rxCmd[4]

    def read_Speed(self,servo_id):
        txCmd = [0xA0 | servo_id,
                 0x02]
        self.krs.write(txCmd)
        time.sleep(0.1)
        rxCmd = self.krs.read(5)
        if len(rxCmd) == 0:
            return 0
        else:
            return rxCmd[4]
    
    def read_Stretch(self,servo_id):
        txCmd = [0xA0 | servo_id,
                 0x01]
        self.krs.write(txCmd)
        time.sleep(0.1)
        rxCmd = self.krs.read(5)
        if len(rxCmd) == 0:
            return 0
        else:
            return rxCmd[4]
            
    def Foot_sensor(self):
        pass



if __name__ == "__main__":
    controller = ServoController()
    number_list = list(range(-135, 136))
    c = controller.execute_function(number_list)
    print(c)
    
