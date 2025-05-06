import enum
import threading
import serial
import random
from time import sleep, time
from collections import deque
import pkg_resources

from RobokitRS.RobokitRS_config import *

MAXRATE = 0.395
ALPHARATIO = 0.39215
ANALOGTOANGLERATIO = 0.3515625

LED_PIN        = 2
BUZZER_PIN     = 3
SONAR_PIN      = 12
FRONT_IR_1_PIN = 13
FRONT_IR_2_PIN = 14
FLOOR_IR_1_PIN = 15
FLOOR_IR_2_PIN = 16
FLOOR_IR_3_PIN = 17
IMU_1_PIN      = 18
IMU_2_PIN      = 19
MELODY_1_PIN   = 18
MELODY_2_PIN   = 19
FLOOR_IR_4_PIN = 20
FLOOR_IR_5_PIN = 21

class Modes(enum.Enum):
    INPUT = 0x00
    OUTPUT = 0x01
    ANALOG = 0x02
    PWM = 0x03
    SERVO = 0x04
    I2C = 0x06
    TONE = 0x0A
    SONAR = 0x0B
    RGBLED = 0x0E

class SensorType(enum.Enum):
    TEMPERATURE = 0,
    JOYSTICK_X = 1,
    JOYSTICK_Y = 2,
    LIGHT = 3,
    DIAL = 4,
    A_KEYPAD = 5,
    ROTARYPOSITION = 6,
    MAGNETIC = 7,
    ULTRASONIC = 8,
    SONAR = 9

class GyroDataType(enum.Enum):
    ANGLE_X = 0,
    ANGLE_Y = 1,
    ANGLE_Z = 2,
    GYRO_X = 3,
    GYRO_Y = 4,
    GYRO_Z = 5,
    SHAKE = 6

class RotaryPositionDataType(enum.Enum):
    ROTATION = 0,
    POSITION = 1,
    ANGLE = 2

class TwoWheelsDirectionType(enum.Enum):
    FORWARD = 0,
    BACKWARD = 1,
    LEFTWARD = 2,
    RIGHTWARD = 3,
    STOP = 4

class RobokitRS():
    def __init__(self):
        self.__START_SYSEX = 0xF0
        self.__END_SYSEX = 0xF7

        self.serialPort = None

        self.__eventHandlerDic = {}

        self.__pin_data_dict = {}
        self.__serial_buf = deque()
        self.__protocol_buf = deque()

        self.__connect_checked = False
        self.__version_checked = False

        self.__sonar_read_data = dict.fromkeys(range(2,14),0)
        self.__analog_read_data = dict.fromkeys(range(14,20),0)
        self.__digital_read_data = dict.fromkeys(range(16),0)
        self.__gyro_data = GyroData()
        self.__rotary_data = dict.fromkeys(range(14,20),RotaryPostionData())
        self.__dot_data = [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ]

        self.__led_data_dict = {}
        self.__servo_data_dict = {}
        self.__melody_volume = 100
        self.__gyro_position = -1
        
        self.kill = False

        conf = RobokitRS_config()
        conf.readConfigFile(pkg_resources.resource_filename(__package__,"/pins"), self.__pin_data_dict)

        protocolWriterTH = threading.Thread(target=self.__protocolWriter__)
        protocolWriterTH.daemon = True
        protocolWriterTH.start()

    def __thread_init__(self):
        serialRecvTH = threading.Thread(target=self.__serialReceiver__)
        serialRecvTH.daemon = True
        serialRecvTH.start()

        serialParserTH = threading.Thread(target=self.__serialParser__)
        serialParserTH.daemon = True
        serialParserTH.start()

    def __protocolWriter__(self):
        while True:
            if self.kill: 
                # print("killed protocol writer")
                return
            if len(self.__protocol_buf) == 0 :
                sleep(0)
                continue
            lock = threading.Lock()
            lock.acquire()
            protocolData = self.__protocol_buf.popleft()
            lock.release()
            self.serialPort.write(bytearray(protocolData.data))
            sleep(protocolData.delay)

    def __serialReceiver__(self):
        while True:
            if self.kill: 
                # print("killed serial Receiver")
                return
            if self.serialPort.isOpen() is False:
                sleep(0)
                continue

            if self.serialPort.readable() is False:
                sleep(0)
                continue

            self.__serial_buf.extend(self.serialPort.read_all())
            sleep(0.05)
              
    def __serialParser__(self):
            parseChar = 0x00
            ins = 0x00
            leng = 0x00
            data = []

            while self.__connect_checked is False or self.__version_checked is False:
                if len(self.__serial_buf) <= 5:
                    sleep(0)
                    continue
                parseChar = self.__serial_buf.popleft()
                if parseChar != 0xF0:
                    sleep(0)
                    continue

                parseChar = self.__serial_buf.popleft()
                if parseChar != 0x01:
                    sleep(0)
                    continue

                ins = self.__serial_buf.popleft()
                leng = self.__serial_buf.popleft()

                while True:
                    if len(self.__serial_buf) >= leng:
                        break
                    sleep(0)

                for i in range(leng):
                    data.append(self.__serial_buf.popleft())

                parseChar = self.__serial_buf.popleft()
                if parseChar != 0xF7:
                    sleep(0)
                    continue

                if ins == 0x7A:
                    self.__connect_checked = True
                elif ins == 0x7F:
                    self.__version_checked = True

                ins = 0x00
                leng = 0x00
                del data[:]

            while True:
                if self.kill:
                    print("killed Serial Parser")
                    return
                if len(self.__serial_buf) <= 3:
                    sleep(0)
                    continue

                parseChar = self.__serial_buf.popleft()
                if parseChar == 0xF0:
                    while True:
                        if len(self.__serial_buf) <= 0:
                            sleep(0)
                            continue
                        d = self.__serial_buf.popleft()
                        data.append(d)
                        if d == 0xF7:
                            break

                    if (data[0] == 0x01) and (data[3] == 0x09):
                        self.__gyro_data.angleX = (data[4] + ((data[5] & 0x01) << 7)) * (-1 if ((data[5] >> 4 & 0x01) == 1) else 1)
                        self.__gyro_data.angleY = (data[6] + ((data[7] & 0x01) << 7)) * (-1 if ((data[7] >> 4 & 0x01) == 1) else 1)
                        self.__gyro_data.angleZ = (data[8] + ((data[9] & 0x01) << 7)) * (-1 if ((data[9] >> 4 & 0x01) == 1) else 1)

                        self.__gyro_data.gyroX = (data[10] + ((data[11] & 0x01) << 7)) * (-1 if ((data[11] >> 4 & 0x01) == 1) else 1)
                        self.__gyro_data.gyroY = (data[12] + ((data[13] & 0x01) << 7)) * (-1 if ((data[13] >> 4 & 0x01) == 1) else 1)
                        self.__gyro_data.gyroZ = (data[14] + ((data[15] & 0x01) << 7)) * (-1 if ((data[15] >> 4 & 0x01) == 1) else 1)

                        self.__gyro_data.shake = (data[19] & 0x01)
                    elif (data[0] == 0x63):
                        pin = data[1]
                        sonarLSB = data[2]
                        sonarMSB = data[3]
                        self.__sonar_read_data[pin] = sonarMSB * 127 + sonarLSB
                        if SensorType.SONAR in self.__eventHandlerDic and self.__sonar_read_data[pin] != 508:
                            self.__eventHandlerDic[SensorType.SONAR](self.__sonar_read_data[pin])
                    del data[:]
                elif parseChar == 0x90:
                    digitalData = self.__serial_buf.popleft()
                    digitalData2 = self.__serial_buf.popleft()
                    for i in range(7):
                        self.__digital_read_data[i] = digitalData >> i & 0x01
                    self.__digital_read_data[7] = digitalData2 & 0x01
                elif parseChar == 0x91:
                    digitalData = self.__serial_buf.popleft()
                    dummy = self.__serial_buf.popleft()
                    for i in range(8):
                        self.__digital_read_data[i+8] = digitalData >> i & 0x01
                elif parseChar & 0x0F > -1 and parseChar & 0x0F < 16:
                    pin = parseChar & 0x0F
                    lsb = self.__serial_buf.popleft()
                    msb = self.__serial_buf.popleft()
                    value = (msb << 7) + lsb

                    if pin > 5:
                        continue
                    self.__analog_read_data[pin + 14] = value
                    rotaryObj = self.__rotary_data[pin+14]
                    if rotaryObj.enable == True:
                        if rotaryObj.firstValue == None:
                            rotaryObj.firstValue = value
                            rotaryObj.points = []

                            angle = self.__measureRotaryPositionSensorAngle__(pin+14)
                            if angle < 0:
                                angle += 360
                            rotaryObj.calibration = (rotaryObj.calibration % 360) - angle

                        length = len(rotaryObj.points)
                        if length == 0 or (int(rotaryObj.points[length -1]) != int(value)):
                            rotaryObj.points.append(value)
                            self.__measureRotaryPositionSensorPosition__(pin+14)

    def port_open(self, portname:str):
        """
        Serialport open func
        
        Parameters
        --------
        portName
            Name of serialport
            (example : 'COM1')
        """
        # macos 사용을 위한 해당 항목 제거, 추후 os 확인 전처리기 추가
        # portname = portname.upper()
        # if "COM" not in portname:
        #     print("Portname value error. Portname should be a \'COM\' with number. Example : COM1")
        #     return

        self.serialPort = serial.Serial( portname, baudrate=115200)
        if self.serialPort.isOpen() == True:
            self.serialPort.close()
        self.serialPort.open()
        print("RSBoard serial port opend")

        self.__thread_init__()
        print("Thread init done")

        self.__init_robokitRS_th__()
        print("RSBoard init done")

        return True
    
    def end(self):
        print("End this Program")
        self.kill = True
        self.serialPort.write(bytearray([0xff, 0x00, 0x00]))
        sleep(0.5)
        self.serialPort.close()
        print("Serial Port Status:", self.serialPort.is_open)
        try:
            exit()
        except:
            pass 
        
    def __init_robokitRS_th__(self):
        if self.serialPort.isOpen() is False:
            print("Port is not connected")
            return

        while self.__connect_checked is False:
            protocol = ProtocolData()
            protocol.data = [self.__START_SYSEX, 0x01, 0x7A, 0x00, self.__END_SYSEX]
            self.__protocol_buf.append(protocol)
            sleep(1)

        while self.__version_checked is False:
            protocol = ProtocolData()
            protocol.data = [self.__START_SYSEX, 0x01, 0x7F, 0x00, self.__END_SYSEX]
            self.__protocol_buf.append(protocol)
            sleep(1)

        protocol = ProtocolData()
        protocol.data = [0xd0, 0x01, 0xd1, 0x01]
        protocol.delay = 10 *0.001
        self.__protocol_buf.append(protocol)

        if self.__pin_mode_init__() is False:
            print('Pin mode init failed')
            return False
        print('Pin mode init done')

        if self.__syntex_init__() is False:
            print('Sysex init failed')
            return False
        print('Sysex init done')

    def __pin_mode_init__(self):
        try:
            for number, data in self.__pin_data_dict.items():
                protocol = ProtocolData()
                protocol.data = [ 0xF4, number, data.mode]
                self.__protocol_buf.append(protocol)
        except:
            print('Pin mode init error : Pin map data error')
            return False
        return True
    
    def __syntex_init__(self):
        try:
            for number, data in self.__pin_data_dict.items():
                if data.mode == 14:
                    self.__pixel_led_init(number, 1)
        except:
            print('Syntex init error : Syntex map data error')
            return False
        return True
    
    def SetEventHandler(self, event:SensorType, func):
        if event in self.__eventHandlerDic:
            print("Event ", event, " already exist lisntener")
        else:
            self.__eventHandlerDic[event] = func

    def RemoveEventHandler(self, event:SensorType):
        if event in self.__eventHandlerDic:
            del self.__eventHandlerDic[event]
            print("Event ", event, " is removed")
        else:
            print("Event ", event, " not exist lisntener")

#  =================== CONTROL FUNC =================== #
    def delay(self, waitingtime:float = 1):
        """
        Keeps the robot wating for a specified time
        
        Parameters
        --------
        waitingTime
            Wating time of robot 
            (Unit : sec)
        """
        
        protocol = ProtocolData()
        protocol.data.append(0x00)
        protocol.delay = waitingtime
        self.__protocol_buf.append(protocol)


#  =================== ANALOG FUNC =================== #
    def analog_read(self,pin:int)->int:
        """
        Analog sensor read function
        
        Parameters
        --------
        pin
            Pin number of analog sensors
            (min : 0, max : 5)
        """

        if pin < 0 or pin > 5:
            print('pin value error. pin value must be between 0 and 5')
            return

        return self.__analog_read_data[pin+14]

    def analog_reads(self, pins:list):
        """
        Analog sensor read function
        
        Parameters
        --------
        pins
            Pin number list of analog sensors
        """

        if type(pins) is not list:
            print('__pin_data_dict type error. pin value type must be list')
            return

        ret = []
        for i in pins:
            if i < 0 or i > 5:
                print('pin value error. pin value must be between 0 and 5')
                return
            ret.append( self.__analog_read_data[i+14])

        return ret

    def analog_write(self, pin:int, value:int):
        """
        Analog sensor write function
        
        Parameters
        --------
        pin
            Pin number of analog sensor
            (min : 0, max : 5)
        value
            Value to input to analog sensor
            (min : 0, max : 255)
        """

        if pin < 0 or pin > 5:
            print('pin value error. pin value must be between 0 and 5')
            return
        if value < 0 or value > 5:
            print('value error. pin value must be between 0 and 5')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x01)
        protocol.data.append(0x03)
        protocol.data.append(0x00)
        protocol.data.append(int( value % 128 ))
        protocol.data.append( (pin+14) << 1 | int(value / 128) )
        protocol.data.append(self.__END_SYSEX)
        
        self.__protocol_buf.append(protocol)

#  =================== DIGITAL FUNC =================== #
    def digital_read(self, pin:int)->int:
        """
        Digital sensor read function
        
        Parameters
        --------
        pin
            Pin number of digital sensor
            (min : 2, max : 13)
        """

        if pin > 13 or pin < 2:
            print('pin value error. pin value must be between 2 and 13')
            return

        return self.__digital_read_data[pin]

    def digital_reads(self, pins:list)->list:
        """
        Digital sensor read function
        
        Parameters
        --------
        pins
            Pin number list of digital sensors
            (min : 2, max : 13)
        """

        if type(pins) is not list:
            print('__pin_data_dict type error. __pin_data_dict type must be list')
            return

        ret = []
        for i in pins:
            if i > 13 or i < 2:
                print('pin value error. pin value must be between 2 and 13')
                return
            ret.append( self.__digital_read_data[i])

        return ret
    
    def digital_write(self, pin:int, value:int):
        """
        Digital sensor write function
        
        Parameters
        --------
        pin
            Pin number of digital sensors
            (min : 2, max : 13)
        value
            Value to input to digital sensor
            (min : 0, max : 1)
        """

        if pin > 13 or pin < 2:
            print('pin value error. pin value must be between 2 and 13')
            return
        if value > 1 or value < 0:
            print('value error. pin value must be 0 or 1')
            return

        protocol = ProtocolData()
        if pin <= 7:
            protocol.data.append(0x90)
        else:
            protocol.data.append(0x91)

        if value == 1:
            if pin == 2:
                protocol.data.append(0x04)
                protocol.data.append(0x00)
            elif pin == 3:
                protocol.data.append(0x08)
                protocol.data.append(0x00)
            elif pin == 4:
                protocol.data.append(0x10)
                protocol.data.append(0x00)
            elif pin == 5:
                protocol.data.append(0x20)
                protocol.data.append(0x00)
            elif pin == 6:
                protocol.data.append(0x40)
                protocol.data.append(0x00)
            elif pin == 7:
                protocol.data.append(0x00)
                protocol.data.append(0x01)
            elif pin == 8:
                protocol.data.append(0x01)
                protocol.data.append(0x00)
            elif pin == 9:
                protocol.data.append(0x02)
                protocol.data.append(0x00)
            elif pin == 10:
                protocol.data.append(0x04)
                protocol.data.append(0x00)
            elif pin == 11:
                protocol.data.append(0x08)
                protocol.data.append(0x00)
            elif pin == 12:
                protocol.data.append(0x10)
                protocol.data.append(0x00)
            elif pin == 13:
                protocol.data.append(0x20)
                protocol.data.append(0x00)
        else:
            protocol.data.append(0x00)
            protocol.data.append(0x00)
        self.__protocol_buf.append(protocol)


#  =================== SENSOR FUNC =================== #
    def sonar_begin(self, pin):
        """
        Sonar sensor begin function
        
        Parameters
        --------
        pin
            Pin number of digital sensors
            (min : 2, max : 13)
        """

        if pin > 13 or pin < 2:
            print('pin value error. pin value must be between 2 and 13')
            return

        self.set_pin_mode(pin, Modes.SONAR)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x62)
        protocol.data.append(pin)
        protocol.data.append(pin)
        protocol.data.append(0xff)
        protocol.data.append(0x10)
        protocol.data.append(0x03)
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def sonar_read(self, pin):
        """
        Sonar sensor read function
        
        Parameters
        --------
        pin
            Pin number of digital sensors
            (min : 2, max : 13)
        """
        
        if pin > 13 or pin < 2:
            print('pin value error. pin value must be between 2 and 13')
            return
    

        return self.__sonar_read_data[pin]

    def sensor_read(self, pin:int, type:SensorType=SensorType.TEMPERATURE):
        """
        Analog sensor read function
        
        Parameters
        --------
        pin
            Pin number of analog sensors
            (min : 0, max : 5)
        """

        if pin < 0 or pin > 5:
            print('pin value error. pin value must be between 0 and 5')
            return
        pin += 14
        if type == SensorType.TEMPERATURE:
            return self.__analog_read_data[pin]
        elif type == SensorType.JOYSTICK_X or type == SensorType.JOYSTICK_Y:
            # default value == 512
            value = self.__analog_read_data[pin] >> 6
            if (value > 14): return 2
            elif (value > 9): return 1
            elif (value > 5): return 0
            elif (value > 1): return -1
            else: return -2
        elif type == SensorType.LIGHT:
            return int(round(self.__analog_read_data[pin] * 0.1))
        elif type == SensorType.DIAL:
            return int(round(self.__analog_read_data[pin] * 0.1))
        elif type == SensorType.A_KEYPAD:
            value = self.__analog_read_data[pin]
            if (value >= 450) : return 1
            elif (value >= 390) : return 2
            elif (value >= 310) : return 3
            elif (value >= 200) : return 4
            elif (value >= 100) : return 5
            else : return 0
        elif type == SensorType.MAGNETIC:
            analogValue = self.__analog_read_data[pin]
            zero = 512
            zeroGap = 32
            startS = zero + zeroGap
            startN = zero - zeroGap
            maxArea = 64

            value = 0
            if analogValue > startS:
                value = (analogValue > 1024 - maxArea) if 1024 - maxArea else analogValue
                value = min(10, int(round((value - startS) / 32)))
            elif analogValue < startN :
                value = (analogValue < maxArea) if maxArea else analogValue
                value = min(10, int(round((startN - value) / 32))) * -1
            return value
        else:
            return 0


#  =================== MOTOR FUNC =================== #
    def motor_stop(self, motorid:int):
        """DCMotor write func.

        Parameters
        --------
        motorID
            DCMotor's ID
            Motor1 : 1
            Motor2 : 2
            Motor3 : 3
            Motor4 : 4
        """
        motorid -= 1  # Adjust motorid to match the expected range (0-3)
        if motorid > 3 or motorid < 0:
            print('motorid value error. motorid value must be between 0 and 3')
            return
        
        self.motor_write(motorid,0,0)


    def set_twowheels_direction(self, first_id:int, second_id:int, first_speed:int=15, second_speed:int=15, direction:TwoWheelsDirectionType=TwoWheelsDirectionType.STOP):
        """TwoWheels robot driving in the direction func.
        
        Parameters
        --------
        motorID
            DCMotor's ID
            Motor1 : 0
            Motor2 : 1
            Motor3 : 2
            Motor4 : 3
        speed
            DCMotor's speed
            (min : 0, max : 15)
        direction
            The direction of drive
            FOWARD : 0
            BACKWARD : 1
            LEFTWARD : 2
            RIGHTWARD : 3
            """
        
        if first_id == second_id:
            print('motorids value error. motorids value must be different')
            return
        if first_id > 3 or first_id < 0:
            print('motorid value error. motorid value must be between 0 and 3')
            return
        if second_id > 3 or second_id < 0:
            print('motorid value error. motorid value must be between 0 and 3')
            return
        if first_speed > 15 or first_speed < 0:
            print('speed value error. speed value must be between 0 and 15')
            return
        if second_speed > 15 or second_speed < 0:
            print('speed value error. speed value must be between 0 and 15')
            return
        if type(direction) is not TwoWheelsDirectionType:
            print('direction type error. direction type must be TwoWheelsDirectionType')
            return

        if first_id == 0:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
        elif first_id == 1:
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
        elif first_id == 2:
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
        elif first_id == 3:
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

        if second_id == 0:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
        elif second_id == 1:
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
        elif second_id == 2:
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
        elif second_id == 3:
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

        dir1 = 0
        dir2 = 0
        speed1 = first_speed * 17
        speed2 = second_speed * 17

        if first_id == 0 or first_id == 2:
            if direction == TwoWheelsDirectionType.FORWARD or direction == TwoWheelsDirectionType.RIGHTWARD:
                dir1 = 1
            elif direction == TwoWheelsDirectionType.BACKWARD or direction == TwoWheelsDirectionType.LEFTWARD:
                dir1 = 0
            else:
                speed1 = 0
        else:
            if direction == TwoWheelsDirectionType.FORWARD or direction == TwoWheelsDirectionType.RIGHTWARD:
                dir1 = 0
            elif direction == TwoWheelsDirectionType.BACKWARD or direction == TwoWheelsDirectionType.LEFTWARD:
                dir1 = 1
            else:
                speed1 = 0

        if second_id == 1 or second_id == 3:
            if direction == TwoWheelsDirectionType.FORWARD or direction == TwoWheelsDirectionType.LEFTWARD:
                dir2 = 0
            elif direction == TwoWheelsDirectionType.BACKWARD or direction == TwoWheelsDirectionType.RIGHTWARD:
                dir2 = 1
            else:
                speed2 = 0
        else:
            if direction == TwoWheelsDirectionType.FORWARD or direction == TwoWheelsDirectionType.LEFTWARD:
                dir2 = 1
            elif direction == TwoWheelsDirectionType.BACKWARD or direction == TwoWheelsDirectionType.RIGHTWARD:
                dir2 = 0
            else:
                speed2 = 0

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x02)
        protocol.data.append(0x04)
        protocol.data.extend(self.__make_motor_data(first_id,dir1,speed1))
        protocol.data.extend(self.__make_motor_data(second_id,dir2,speed2))
        protocol.data.append(self.__END_SYSEX)
        
        self.__protocol_buf.append(protocol)
        

    def motor_write(self, motorid:int, direction:int=0, speed:int=15):
        """DCMotor write func.

        Parameters
        --------
        motorID
            DCMotor's ID
            Motor1 : 0
            Motor2 : 1
            Motor3 : 2
            Motor4 : 3
        direction
            The direction of rotation
            CW : 0
            CCW : 1
        speed
            DCMotor's speed
            (min : 0, max : 15)
            """

        if motorid > 3 or motorid < 0:
            print('motorid value error. motorid value must be between 0 and 3')
            return
        if direction > 1 or direction < 0:
            print('direction value error. direction value must be 0 or 1')
            return
        if speed > 15 or speed < 0:
            print('speed value error. speed value must be between 0 and 15')
            return

        speed = speed * 17

        if motorid == 0:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
        elif motorid == 1:
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
        elif motorid == 2:
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
        elif motorid == 3:
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x02)
        protocol.data.append(0x02)
        protocol.data.append( int(speed % 128) )
        protocol.data.append( int(motorid) << 4 | int(direction) << 3 | int(speed / 128))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)








#asdfafa;kdjfalkfjkldsfjalkjf






    def motor_write2(self, motorid:int, direction:int=0, speed:int=15):
        """DCMotor write func.

        Parameters
        ----------
        motorid : int
            DCMotor's ID (1 to 4)
        direction : int
            The direction of rotation (CW: clockwise, CCW: counter-cockwise)
        speed : int
            DCMotor's speed (min=0, max=15)
        """
        motorid -= 1  # Adjust motorid to match the expected range (0-3)
        if not (0 <= motorid <= 3):
            print('motorid value error. motorid value must be between 0 and 3')
            return
        if direction not in (0, 1):
            print('direction value error. direction value must be 0 or 1')
            return
        if not (0 <= speed <= 15):
            print('speed value error. speed value must be between 0 and 15')
            return

        if motorid == 0:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
        elif motorid == 1:
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
        elif motorid == 2:
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
        elif motorid == 3:
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

        protocol = ProtocolData()
        protocol.data.extend([
        self.__START_SYSEX,
        0x00,
        0x02,
        0x02,
        int(speed % 128),
        (motorid << 4) | (direction << 3) | int(speed / 128),
        self.__END_SYSEX
        ])
        self.__protocol_buf.append(protocol)
        

    def __make_motor_data(self, channel, dir, speed):
        data = []

        data.append( int(speed % 128) )
        data.append( int(channel) << 4 | int(dir) <<3 | int(speed / 128))

        return data

    def servo_write(self, pin:int, position:int):
        """servoMotor write func.

        Parameters
        --------
        pin
            Pin number of servoMotor
            (min : 2, max : 13 )
        position
            Angle value of servoMotor
            (min : -120, max : 120)
            """

        if pin > 13 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 13')
            return
        if position > 120 or position < -120:
            print('Position value error. Position value must be between -120 and 120')
            return

        if pin in self.__servo_data_dict:
            pass
        else:
            self.set_pin_mode(pin, Modes.SERVO)

        if self.__servo_data_dict[pin].prevAngle == position:
            return

        self.__servo_data_dict[pin].prevAngle = position
        position = -min(120, max(-120, position)) + 120
        
        protocol = ProtocolData()
        protocol.data.append(0xE0 | pin)
        protocol.data.append( int(position % 128) )
        protocol.data.append( int(position / 128) )
        self.__protocol_buf.append(protocol)

        # protocol = ProtocolData()
        # protocol.data.append(self.__START_SYSEX)
        # protocol.data.append(0x00)
        # protocol.data.append(0x03)
        # protocol.data.append(0x03)
        # protocol.data.append(0x00)
        # protocol.data.append( int(position % 128) )
        # protocol.data.append( int(pin) << 1 | int(position / 128) )
        # protocol.data.append(self.__END_SYSEX)
        # self.__protocol_buf.append(protocol)

    # def step_motor_Write(self, channel, step, dir, speed, actionmode):
    #     protocol = ProtocolData()
    #     protocol.data.append(self.__START_SYSEX)
    #     protocol.data.append(0x00)
    #     protocol.data.append(0x05)
    #     protocol.data.append(0x02)
    #     protocol.data.append(int(step % 128))
    #     protocol.data.append( int(actionmode) << 6 | int(speed) << 5 | int(dir) << 4 | int(channel) << 3 | int(step/128) << 1 )
    #     protocol.data.append(self.__END_SYSEX)
    #     self.__protocol_buf.append(protocol)

#  =================== SMARTMOTOR FUNC  =================== #
#   Motor 1 = foward left
#   Motor 2 = foward right
#   Motor 3 = backward left
#   Motor 4 = backward right
    def smartmotor_write(self, motorid, wheelflag, control, brakeflag, torque, red, green, blue, position, sign, speed, dir):
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x04)
        protocol.data.append(0x03)
        protocol.data.append( int(red) << 6 | int(green) << 5 | int(blue) << 4 | int(torque) << 3 )

        if wheelflag is True:
            protocol.data[4] = (protocol.data[4]) | int(brakeflag) << 1 | int(wheelflag)
            protocol.data.append(int(speed % 128))
            protocol.data.append( int(motorid) << 6 | int(dir) << 1 | int(speed / 128))
        else:
            protocol.data[4] = (protocol.data[4]) | int(control) << 1 | int(wheelflag)
            protocol.data.append( int(position % 128) )
            protocol.data.append( int(motorid) << 6 | int(sign) << 1 | int(position / 128))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def smart_motor_write(self, motorid:int, red:bool, green:bool, blue:bool, direction:int, speed:int):

        r = red if 1 else 0
        g = green if 1 else 0
        b = blue if 1 else 0

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x04)
        protocol.data.append(0x03)
        protocol.data.append( int(r) << 6 | int(g) << 5 | int(b) << 4 | int(3) << 3 )
        protocol.data[4] = (protocol.data[4]) | int(1) << 1 | int(1)
        protocol.data.append(int(speed % 128))
        protocol.data.append( int(motorid) << 6 | int(direction) << 1 | int(speed / 128))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_smartmotor_color(self,motorIds:list=[0], red:bool=0,green:bool=0,blue:bool=0):
        """
        Robot set led color func

        Parameters
        --------
        motorId
            Smart Motor's ID
            Motor1 : 1
            Motor2 : 2
            Motor3 : 3
            Motor4 : 4
            All motor : 0
        r
            Red LED on-off switch
            (on : true, off : false)
        g
            Green LED on-off switch
            (on : true, off : false)
        b
            Blue LED on-off switch
            (on : true, off : false)
        """

        if type(motorIds) is not list:
            print('MotorIds type error. MotorIds type must be list')
            return
        
        if len(motorIds) > 4:
            print('MotorIdsl length error. The maximum number of motor ID is four')
            return

        r = red if 1 else 0
        g = green if 1 else 0
        b = blue if 1 else 0

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x04)
        if 0 in motorIds:
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(r,g,b,1,1,0))
            protocol.data.extend(self.__MakeSmartMotorData(r,g,b,0,2,0))
            protocol.data.extend(self.__MakeSmartMotorData(r,g,b,0,3,0))
            protocol.data.extend(self.__MakeSmartMotorData(r,g,b,1,4,0))
        else:
            protocol.data.append(len(motorIds) * 3)
            for i in range(len(motorIds)):
                if motorIds[i] < 0 or motorIds[i] > 4:
                    print('MotorIds value error. MotorIds value must be between 0 and 5')
                    return
                protocol.data.extend(self.__MakeSmartMotorData(r,g,b,1,motorIds[i],0))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_smartmotor_red(self):
        self.set_smartmotor_color(red=1,green=0,blue=0)
    def set_smartmotor_green(self):
        self.set_smartmotor_color(red=0,green=1,blue=0)
    def set_smartmotor_blue(self):
        self.set_smartmotor_color(red=0,green=0,blue=1)
    def set_smartmotor_yellow(self):
        self.set_smartmotor_color(red=1,green=1,blue=0)
    def set_smartmotor_magenta(self):
        self.set_smartmotor_color(red=1,green=0,blue=1)
    def set_stmartmotor_cyan(self):
        self.set_smartmotor_color(red=0,green=1,blue=1)
    def set_smartmotor_white(self):
        self.set_smartmotor_color(red=1,green=1,blue=1)
    def set_smartmotor_off(self):
        self.set_smartmotor_color(red=0,green=0,blue=0)

    def __MakeSmartMotorData(self, r, g, b, dir, id, speed)->list:
        data =[]

        wheelMode = 1
        brakeMode = 1
        torque = 3

        mode = 0
        sp = 0

        mode = wheelMode
        mode = (brakeMode << 1) | mode
        mode = (torque << 2) | mode
        mode = (b << 4) | mode
        mode = (g << 5) | mode
        mode = (r << 6) | mode

        # protocol.data.append()
        # protocol.data.append( (pin+14) << 1 | int(value / 128) )

        sp = int(speed / 128)
        sp = (dir << 1) | sp
        sp = (id << 2) | sp

        data.append(mode)
        data.append(int( speed % 128 ))
        data.append(sp)

        return data

#  =================== DRIVE FUNC  =================== #
    def set_mecanumwheels_drive_front(self,speed:int = 15, motortype:int=1):
        """
        Robot forward driving func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """

        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must be between 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return
        
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,1,speed))
            protocol.data.extend(self.__make_motor_data(1,0,speed))
            protocol.data.extend(self.__make_motor_data(2,1,speed))
            protocol.data.extend(self.__make_motor_data(3,0,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,4,smartmotor_speed))

        protocol.data.append(self.__END_SYSEX)
        
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_back(self, speed:int = 15, motortype:int=1):
        """
        Robot backward driving func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """

        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must be between 0 and 15')
            return
        
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,0,speed))
            protocol.data.extend(self.__make_motor_data(1,1,speed))
            protocol.data.extend(self.__make_motor_data(2,0,speed))
            protocol.data.extend(self.__make_motor_data(3,1,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,4,smartmotor_speed))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_right(self, speed:int = 15, motortype:int=1):
        """
        Robot rightward driving func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """

        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must be between 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,1,speed))
            protocol.data.extend(self.__make_motor_data(1,1,speed))
            protocol.data.extend(self.__make_motor_data(2,0,speed))
            protocol.data.extend(self.__make_motor_data(3,0,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,4,smartmotor_speed))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_left(self, speed:int = 15, motortype:int=1):
        """
        Robot leftward driving func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,0,speed))
            protocol.data.extend(self.__make_motor_data(1,0,speed))
            protocol.data.extend(self.__make_motor_data(2,1,speed))
            protocol.data.extend(self.__make_motor_data(3,1,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,4,smartmotor_speed))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_rotate_left(self, speed:int = 15, motortype:int=1):
        """
        Robot rotate leftward func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,0,speed))
            protocol.data.extend(self.__make_motor_data(1,0,speed))
            protocol.data.extend(self.__make_motor_data(2,0,speed))
            protocol.data.extend(self.__make_motor_data(3,0,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,4,smartmotor_speed))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_rotate_right(self, speed:int = 15, motortype:int=1):
        """
        Robot rotate rightward func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,1,speed))
            protocol.data.extend(self.__make_motor_data(1,1,speed))
            protocol.data.extend(self.__make_motor_data(2,1,speed))
            protocol.data.extend(self.__make_motor_data(3,1,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,4,smartmotor_speed))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_frontleft(self, speed:int = 15, motortype:int=1):
        """
        Robot drive front and left func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,1,0))
            protocol.data.extend(self.__make_motor_data(1,0,speed))
            protocol.data.extend(self.__make_motor_data(2,1,speed))
            protocol.data.extend(self.__make_motor_data(3,1,0))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,1,0))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,3,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,4,0))
        
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_frontright(self,speed:int = 15, motortype:int=1):
        """
        Robot drive front and right func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)

        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,1,speed))
            protocol.data.extend(self.__make_motor_data(1,0,0))
            protocol.data.extend(self.__make_motor_data(2,1,0))
            protocol.data.extend(self.__make_motor_data(3,0,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,2,0))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,3,0))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,4,smartmotor_speed))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_backleft(self, speed:int = 15, motortype:int=1):
        """
        Robot drive back and left func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)
            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,0,speed))
            protocol.data.extend(self.__make_motor_data(1,0,0))
            protocol.data.extend(self.__make_motor_data(2,1,0))
            protocol.data.extend(self.__make_motor_data(3,1,speed))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(6)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,1,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,4,smartmotor_speed))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_backright(self, speed:int = 15, motortype:int=1):
        """
        Robot drive right and back func

        Parameters
        --------
        Speed
            Robot's driving speed
            (min : 0, max : 15)

        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        if speed > 15 or speed < 0:
            print('Speed value error. Speed value must between be 0 and 15')
            return
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)

        if motortype == 1:
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)

            speed = speed * 17
            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(0,0,0))
            protocol.data.extend(self.__make_motor_data(1,1,speed))
            protocol.data.extend(self.__make_motor_data(2,0,speed))
            protocol.data.extend(self.__make_motor_data(3,1,0))
        elif motortype == 2:
            smartmotor_speed = speed * 8
            protocol.data.append(0x04)
            protocol.data.append(6)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,1,2,smartmotor_speed))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,3,smartmotor_speed))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_mecanumwheels_drive_stop(self, motortype:int=1):
        """
        Robot stop driving func

        Parameters
        --------
        Motor type
            Robot's motor type
            motor : 1
            smart motor : 2
        """
        
        if motortype < 0 or motortype > 2:
            print('Motor type value error. Motor type value must be 1 or 2')
            return

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)

        if motortype == 1:
            self.set_pin_mode(4,Modes.OUTPUT)
            self.set_pin_mode(5,Modes.OUTPUT)
            self.set_pin_mode(6,Modes.OUTPUT)
            self.set_pin_mode(7,Modes.OUTPUT)
            self.set_pin_mode(8,Modes.OUTPUT)
            self.set_pin_mode(9,Modes.OUTPUT)
            self.set_pin_mode(10,Modes.OUTPUT)
            self.set_pin_mode(11,Modes.OUTPUT)

            protocol.data.append(0x02)
            protocol.data.append(0x08)
            protocol.data.extend(self.__make_motor_data(1,1,0))
            protocol.data.extend(self.__make_motor_data(2,1,0))
            protocol.data.extend(self.__make_motor_data(3,1,0))
            protocol.data.extend(self.__make_motor_data(4,1,0))
        elif motortype == 2:
            protocol.data.append(0x04)
            protocol.data.append(12)
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,1,0))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,2,0))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,3,0))
            protocol.data.extend(self.__MakeSmartMotorData(0,0,0,0,4,0))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

 #  =================== Sensor Init =================== #
    def set_pin_mode(self, pin:int, mode:Modes):
        """
        Pin mode setting func
        
        Parameters
        --------
        pin
            The pin number to be set
            (min : 2, max : 19)
        mode
            INPUT\n
            OUTPUT\n
            ANALOG\n
            PWM\n
            SERVO\n
            I2C\n
            TONE\n
            SONAR\n
            RGBLED\n
            """

        if pin > 19 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 19')
            return

        if self.__pin_data_dict[pin].mode == mode:
            return

        if mode == Modes.RGBLED:
            self.__pixel_led_init(pin, 1)
            ledData = RGBLedData()
            ledData.pinNumber = pin
            ledData.red = 0
            ledData.prevred = 0
            ledData.green = 0
            ledData.prevgreen = 0
            ledData.blue = 0
            ledData.prevblue = 0
            ledData.brightness = 100
            ledData.prevBrightness = 0
            ledData.isOn = False
            self.__led_data_dict[pin] = ledData
        elif mode == Modes.SERVO:
            servoData = ServoData()
            servoData.prevAngle = 0
            self.__servo_data_dict[pin] = servoData

        self.__pin_data_dict[pin].mode = mode
        protocol = ProtocolData()
        protocol.data = [ 0xF4, pin, mode.value ]
        self.__protocol_buf.append(protocol)

    def __servo_motor_init(self, pinNumber, min, max):
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x70)
        protocol.data.append(pinNumber)
        protocol.data.append(min % 128)
        protocol.data.append(min / 128)
        protocol.data.append(max % 128)
        protocol.data.append(max / 128)
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def __pixel_led_init(self, pinNumber, moduleCount) :
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x07)
        protocol.data.append(2)
        protocol.data.append( pinNumber << 1 )
        protocol.data.append(moduleCount)
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

#  =================== LED FUNC  =================== #
    def set_rgb_led_red(self, pin = LED_PIN):
        """
        RGB LED set red color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 100, 0, 0)

    def set_rgb_led_orange(self, pin = LED_PIN):
        """
        RGB LED set orange color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 100, 67, 0)

    def set_rgb_led_yellow(self, pin = LED_PIN):
        """
        RGB LED set yellow color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 100, 100, 0)

    def set_rgb_led_green(self, pin = LED_PIN):
        """
        RGB LED set green color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 0, 100, 0)

    def set_rgb_led_sky(self, pin = LED_PIN):
        """
        RGB LED set sky color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 0, 67, 100)

    def set_rgb_led_navy(self, pin = LED_PIN):
        """
        RGB LED set navy color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 33, 0, 100)

    def set_rgb_led_purple(self, pin = LED_PIN):
        """
        RGB LED set purple color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 67, 0, 100)
    
    def set_rgb_led_pink(self, pin = LED_PIN):
        """
        RGB LED set pink color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 100, 0, 100)

    def set_rgb_led_white(self, pin = LED_PIN):
        """
        RGB LED set white color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 100, 100, 100)
    
    def set_rgb_led_off(self, pin = LED_PIN):
        """
        RGB LED set off color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        self.set_rgb_led_color( pin, 0, 0, 0)

    def set_rgb_led_random(self, pin = LED_PIN):
        """
        RGB LED set random color function
        
        Parameters
        pin
            The pin number to be set
            (min : 2, max : 13)
        """
        r = random.randrange(1,100)
        g = random.randrange(1,100)
        b = random.randrange(1,100)

        self.set_rgb_led_color( pin , r, g, b)

    def set_rgb_led_color(self, pin:int=LED_PIN, red:int=0, green:int=0, blue:int=0):
        """
        RGB LED set color function.

        Parameters
        --------
        pin
            Pin number of RGB LED
            (min : 2, max : 13)
        red
            R value of RGB LED
            (min : 0, max : 100)
        green
            G value of RGB LED
            (min : 0, max : 100)
        blue
            B value of RGB LED
            (min : 0, max : 100)
        """
        if pin > 13 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 13')
            return
        if red > 100 or red < 0:
            print('R value error. R value must be between 0 and 100')
            return
        if green > 100 or green < 0:
            print('G value error. G value must be between 0 and 100')
            return
        if blue > 100 or blue < 0:
            print('B value error. B value must be between 0 and 100')
            return

        if pin in self.__led_data_dict:
            pass
        else:
            self.set_pin_mode(pin, Modes.RGBLED)

        self.__led_data_dict[pin].red = red 
        self.__led_data_dict[pin].green = green
        self.__led_data_dict[pin].blue = blue

        if self.__led_data_dict[pin].isOn is True:
            self.set_rgb_led_on(pin)

    def set_rgb_led_brightness(self, pin:int=LED_PIN, brightness:int=100):
        """
        RGB LED set brightness function.

        Parameters
        --------
        pin
            Pin number of RGB LED
            (min : 2, max : 13)
        brightness
            Brightness value of RGB LED
            (min : 0, max : 100)
        """

        if pin > 13 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 13')
            return
        if brightness > 100 or brightness < 0:
            print('Brightness value error. Brightness value must be between 0 and 100')
            return

        if pin in self.__led_data_dict:
            if brightness == self.__led_data_dict[pin].prevBrightness:
                return
        else:
            self.set_pin_mode(pin, Modes.RGBLED)

        self.__led_data_dict[pin].brightness = brightness

        if self.__led_data_dict[pin].isOn is True:
            self.set_rgb_led_on(pin)

    def change_rgb_led_brightness(self, pin:int=LED_PIN, brightness:int=10):
        """
        RGB LED change brightness function.

        Parameters
        --------
        pin
            Pin number of RGB LED
            (min : 2, max : 13)
        brightness
            The relative brightness value of RGB LED
        """

        if pin > 13 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 13')
            return

        if pin in self.__led_data_dict:
            pass
        else:
            self.set_pin_mode(pin, Modes.RGBLED)
        
        self.__led_data_dict[pin].brightness += brightness

        if self.__led_data_dict[pin].brightness < 0:
            self.__led_data_dict[pin].brightness = 0
        if self.__led_data_dict[pin].brightness > 100:
            self.__led_data_dict[pin].brightness = 100

        if self.__led_data_dict[pin].isOn is True:
            self.set_rgb_led_on(pin)

    def set_rgb_led_on(self,pin:int=LED_PIN):
        """
        RGB LED set color function.

        Parameters
        --------
        pin
            Pin number of RGB LED
            (min : 2, max : 13)
        """

        if pin in self.__led_data_dict:
            pass
        else:
            self.set_pin_mode(pin, Modes.RGBLED)

        if (self.__led_data_dict[pin].red == self.__led_data_dict[pin].prevred) and (self.__led_data_dict[pin].green == self.__led_data_dict[pin].prevgreen) and (self.__led_data_dict[pin].blue == self.__led_data_dict[pin].prevblue) and (self.__led_data_dict[pin].brightness == self.__led_data_dict[pin].prevBrightness):
            return
        
        self.__led_data_dict[pin].prevred = self.__led_data_dict[pin].red
        self.__led_data_dict[pin].prevgreen = self.__led_data_dict[pin].green
        self.__led_data_dict[pin].prevblue = self.__led_data_dict[pin].blue
        self.__led_data_dict[pin].prevBrightness = self.__led_data_dict[pin].brightness
        
        # colorRate = min( MAXRATE, max(0, MAXRATE * (self.__led_data_dict[pin].brightness * ALPHARATIO * 0.01)))
        # r = round(self.__led_data_dict[pin].red * colorRate)
        # g = round(self.__led_data_dict[pin].green * colorRate)
        # b = round(self.__led_data_dict[pin].blue * colorRate)

        r = min( 100, max( 0, round(self.__led_data_dict[pin].red * MAXRATE* (self.__led_data_dict[pin].brightness * ALPHARATIO * 0.01))))
        g = min( 100, max( 0, round(self.__led_data_dict[pin].green * MAXRATE* (self.__led_data_dict[pin].brightness * ALPHARATIO * 0.01))))
        b = min( 100, max( 0, round(self.__led_data_dict[pin].blue * MAXRATE* (self.__led_data_dict[pin].brightness * ALPHARATIO * 0.01))))

        if (r == 0) and (g == 0) and (b == 0):
            self.__led_data_dict[pin].isOn = False
        else:
            self.__led_data_dict[pin].isOn = True

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x07)
        protocol.data.append(0x06)
        protocol.data.append( int(pin << 1 | 1) )
        protocol.data.append(0x01)
        protocol.data.append(int(r % 128))
        protocol.data.append(int(g % 128))
        protocol.data.append(int(b % 128))
        protocol.data.append( int(b / 128) << 2 | int(g / 128) << 1 | int(r / 128) )
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)
        
#  =================== MELODY FUNC  =================== #    
    def tone(self, pin:int = BUZZER_PIN, note:int=0, duration:int = 500):
        """
        Piezo buzzer write func

        Parameters
        --------
        pin
            Pin number of piezo buzzer
            (min : 2, max : 13)
        note
            Sound note number
            (min : 0, max : 95)
        duration
            The sound playing time
            (default : 500)
        """

        if pin > 13 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 13')
            return
        if note > 95 or note < 0:
            print('Note value error. Note value must be between 0 and 95')
            return
        
        frequency = [ freq.value for freq in Notes][note]

        self.set_pin_mode(pin, Modes.TONE)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)    
        protocol.data.append(0x5F)
        protocol.data.append(0x00)
        protocol.data.append(int(pin))
        protocol.data.append(int(frequency % 128))
        protocol.data.append(int(frequency / 128))
        protocol.data.append(int(duration % 128))
        protocol.data.append(int(duration / 128))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def tone_with_delay(self, pin:int=BUZZER_PIN, note:int=0, duration:int = 500):
        if pin > 13 or pin < 2:
            print('Pin value error. Pin value must be between 2 and 13')
            return
        if note > 95 or note < 0:
            print('Note value error. Note value must be between 0 and 95')
            return
        
        frequency = [ freq.value for freq in Notes][note]

        self.set_pin_mode(pin, Modes.TONE)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x5F)
        protocol.data.append(0x00)
        protocol.data.append(int(pin))
        protocol.data.append(int(frequency % 128))
        protocol.data.append(int(frequency / 128))
        protocol.data.append(int(duration % 128))
        protocol.data.append(int(duration / 128))
        protocol.data.append(self.__END_SYSEX)
        protocol.delay = duration * 0.001
        self.__protocol_buf.append(protocol)
    
    def tone_do(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 36, duration)
    def tone_re(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 38, duration)
    def tone_mi(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 40, duration)
    def tone_fa(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 41, duration)
    def tone_sol(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 43, duration)
    def tone_la(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 45, duration)
    def tone_si(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 47, duration)
    def tone_highdo(self, pin:int = BUZZER_PIN, duration = 1000):
        self.tone(pin, 48, duration)
    
    def melody_poweron(self):
        self.tone_with_delay(BUZZER_PIN,75,170)
        self.tone_with_delay(BUZZER_PIN,78,204)
        self.tone_with_delay(BUZZER_PIN,82,255)
        self.tone_with_delay(BUZZER_PIN,85,291)

    def melody_play(self, melodyid:int):
        """
        Uses a melody board to make sounds function

        Parameters
        --------
        melodyID
            Melody's ID in the sd card.
            (min : 1, max : 25245)
        """

        if melodyid < 1 or melodyid > 25245:
            print('MelodyID value error. MelodyID value must be between 1 and 25245')
            return

        folder = melodyid / 255
        number = melodyid - (folder * 255)

        self.set_pin_mode(18,Modes.I2C)
        self.set_pin_mode(19,Modes.I2C)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x0a)
        protocol.data.append(0x06)
        protocol.data.append(0x00)
        protocol.data.append(0x05)
        protocol.data.append(0x03)
        protocol.data.append(folder)
        protocol.data.append( int(number % 128))
        protocol.data.append( int(number / 128))
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)
    
    def change_melody_volume(self, volume:int=-10):
        """
        Melody board change volume function.

        Parameters
        --------
        Volume
            The relative volume value of melody board
        """
        self.__melody_volume += volume

        if self.__melody_volume < 0:
            self.__melody_volume = 0
        elif self.__melody_volume > 100:
            self.__melody_volume = 100

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x0a)
        protocol.data.append(0x04)
        protocol.data.append(0x00)
        protocol.data.append(0x05)
        protocol.data.append(0x06)
        protocol.data.append( round(self.__melody_volume * 0.3))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_melody_volume(self, volume:int=100):
        """
        Melody board set volume function.

        Parameters
        --------
        volume
            Volume value of melody board
            (min : 0, max : 100)
        """

        if volume > 100 or volume < 0:
            print('Volume value error. Volume value must be between 0 and 100')
            return

        if self.__melody_volume == volume:
            return

        self.__melody_volume = volume

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x0a)
        protocol.data.append(0x04)
        protocol.data.append(0x00)
        protocol.data.append(0x05)
        protocol.data.append(0x06)
        protocol.data.append( round(self.__melody_volume * 0.3))

        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def melody_stop(self):
        """
        Melody board stop function.
        """
        self.set_pin_mode(18,Modes.I2C)
        self.set_pin_mode(19,Modes.I2C)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x0a)
        protocol.data.append(0x03)
        protocol.data.append(0x00)
        protocol.data.append(0x05)
        protocol.data.append(0x16)
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

#  =================== GYRO FUNC  =================== #
    def gyro_read(self, type:GyroDataType)->int:
        """
        Gyro sensor data read function
        
        Parameters
        --------
        Type
            Gyro sensor data type.
        """

        if type == GyroDataType.ANGLE_X:
            return self.__gyro_data.angleX
        elif type == GyroDataType.ANGLE_Y:
            return self.__gyro_data.angleY
        elif type == GyroDataType.ANGLE_Z:
            return self.__gyro_data.angleZ
        elif type == GyroDataType.GYRO_X:
            return self.__gyro_data.gyroX
        elif type == GyroDataType.GYRO_Y:
            return self.__gyro_data.gyroY
        elif type == GyroDataType.GYRO_Z:
            return self.__gyro_data.gyroZ
        elif type == GyroDataType.SHAKE:
            return self.__gyro_data.shake

    def gyro_begin(self):
        """
        Start gyro sensor
        """
        self.set_pin_mode(18,Modes.I2C)
        self.set_pin_mode(19,Modes.I2C)

        self.set_gyro_default_position(0)

    def set_gyro_default_position(self, position:int=0):
        """
        Set gyro sensor position func

        Parameters
        --------
        position
            Gyro sensor's default position
            Up : 0 (default)
            Front : 1
            Right : 2
            Back : 3
            Left : 4
        """
        if position > 4 or position < 0:
            print('Position value error. Position value must be between 0 and 4')
            return

        if self.__gyro_position == position:
            return

        self.set_pin_mode(18,Modes.I2C)
        self.set_pin_mode(19,Modes.I2C)

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x7D)
        protocol.data.append(0x02)
        protocol.data.append(0x07)
        protocol.data.append(position)
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

#  =================== ROTARY FUNC  =================== #
    def rotary_position_read(self, pin:int, type:RotaryPositionDataType) -> int:
        ret = 0

        if pin > 5 or pin < 0:
            print('Pin value error. Pin value must be between 0 and 5')
            return ret
        
        pin += 14
        if self.__pin_data_dict[pin].mode is not Modes.ANALOG:
            self.set_pin_mode(pin, Modes.ANALOG)

        rotaryObj = self.__rotary_data[pin]
        if rotaryObj.enable is False:
            self.__enableRotaryPositionSensor(pin)
            self.__rotary_data[pin] = RotaryPostionData()
            self.__rotary_data[pin].enable = True

        if type == RotaryPositionDataType.ROTATION:
            ret = rotaryObj.rotation
        elif type == RotaryPositionDataType.POSITION:
            ret = rotaryObj.position
        elif type == RotaryPositionDataType.ANGLE:
            ret = self.__measureRotaryPositionSensorAngle__(pin,True)
        
        return int(ret)

    def reset_rotary_position(self, pin:int, type:RotaryPositionDataType, value:int):
        if pin > 5 or pin < 0:
            print('Pin value error. Pin value must be between 0 and 5')
            return

        pin += 14
        
        if self.__pin_data_dict[pin].mode is not Modes.ANALOG:
            self.set_pin_mode(pin, Modes.ANALOG)

        rotaryObj = self.__rotary_data[pin]
        if rotaryObj.enable is False:
            self.__enableRotaryPositionSensor(pin)
            self.__rotary_data[pin] = RotaryPostionData()
            self.__rotary_data[pin].enable = True

        if type == RotaryPositionDataType.ROTATION:
            if rotaryObj.firstValue is None:
                rotaryObj.points.append(self.__analog_read_data[pin])
            self.__rotary_data[pin].rotation = value
        elif type == RotaryPositionDataType.POSITION:
            if rotaryObj.firstValue is None:
                rotaryObj.points.append(self.__analog_read_data[pin])
            self.__rotary_data[pin].position = value
        elif type == RotaryPositionDataType.ANGLE:
            if rotaryObj.firstValue is None:
                self.__rotary_data[pin].calibration = value
            else:
                angle = self.__measureRotaryPositionSensorAngle__(pin)
                if angle < 0: 
                    angle += 360
                self.__rotary_data[pin].calibration = (value % 360) - angle

    def __enableRotaryPositionSensor(self, pin:int):

        gap = 64
        gaps = [ (gap&0xFF), (gap>>8)&0xFF ]
        gapMsb = (gaps[0] >> 7) & 0x01
        gaps[0] = gaps[0] & 0x7F
        gaps[1] = ((gaps[1]<<1)+gapMsb) & 0x7F

        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x50)
        protocol.data.append(0x04)
        protocol.data.append(0x05)
        protocol.data.append(pin)
        protocol.data.append(gaps[0])
        protocol.data.append(gaps[1])
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def __measureRotaryPositionSensorAngle__(self, pin:int, useCalibration:bool=False)->int:
        analogValue = self.__analog_read_data[pin]
        angle = analogValue * ANALOGTOANGLERATIO
        if angle > 180:
            angle -= 360
        
        if useCalibration == True:
            angle += self.__rotary_data[pin].calibration % 360
            if angle < -180:
                angle += 360
            elif angle > 180:
                angle -= 360

            angle = int(round(angle))
        else:
            angle = int(round(angle))

        return angle
        
    def __measureRotaryPositionSensorPosition__(self, pin:int):
        obj = self.__rotary_data[pin]
        
        points = obj.points
        length = len(obj.points)
        
        p1 = 0
        p2 = 0
        gap = 0
        dir = 0
        sum = 0

        if length > 1:
            for i in range(1,length):
                p1 = points[i - 1]
                p2 = points[i]
                gap = p2 - p1
                dir = 1 if (gap >= 0) else -1
                if abs(gap) > 512:
                    if dir == 1:
                        gap -= 1024
                    elif dir == -1:
                        gap += 1024
                else:
                    gap = p2 - p1
                sum += gap
            del points[:-1]

            sum = sum * ANALOGTOANGLERATIO
            obj.position += sum
            obj.rotation += (sum/360)
    # =================== Dot function  =================== #
    ###------202305-Dot-Matrix-Update=-------###
        '''
        self.__dot_data = [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ]
        '''
    ###--------------------------------------###


    def __dot_begin(self):
        """
        Start dot matrix
        """
        self.set_pin_mode(18,Modes.I2C)
        self.set_pin_mode(19,Modes.I2C)

    def __dot_bit_draw(self):
        bit_sum = sum(self.__dot_data)
        if bit_sum == 0:
            self.dot_clear()
        else:    
            self.__dot_begin()
            protocol = ProtocolData()
            protocol.data.append(self.__START_SYSEX)
            protocol.data.append(0x00)
            protocol.data.append(0x08)
            protocol.data.append(0x10)
            protocol.data.append(0x00)
            for i in self.__dot_data:
                protocol.data.append(i)
            protocol.data.append(self.__END_SYSEX)
            self.__protocol_buf.append(protocol)

    def clear_dotmatrix(self):
        """
        Clear all leds
        """
        self.__dot_begin()
        self.__dot_data = [ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ]
        protocol = ProtocolData()
        protocol.data.append(self.__START_SYSEX)
        protocol.data.append(0x00)
        protocol.data.append(0x08)
        protocol.data.append(0x01)
        protocol.data.append(0x07)
        protocol.data.append(self.__END_SYSEX)
        self.__protocol_buf.append(protocol)

    def set_dot_state_of_dotmatrix(self, row:int = 0, col:int = 0, fill:int = 1):
        """
        Set single LED on Dot Matrix

        Parameters
        --------
        row  : 0 ~ 6
        col  : 0 ~ 14
        fill : 1(on), 0(off)
        """

        if fill == 1:
            self.__dot_data[col] = self.__dot_data[col] | pow(2, row)
        else:
            self.__dot_data[col] = self.__dot_data[col] & 127 - pow(2, row)
        self.__dot_bit_draw()

    def set_dotmatrix_col(self, col:int = 0, fill:str = "0000000"):
        """
        Set single LED on Dot Matrix

        Parameters
        --------
        col  : 0 ~ 14
        fill : Higher number to the top
        """
        if(len(fill)>7):
            fill = fill[:7]
            fill = fill[::-1]
            self.__dot_data[col] = int(fill, 2)
        elif(len(fill)<7):
            tempStr = format(self.__dot_data[col], 'b').zfill(7)
            tempStr = tempStr[::-1]
            tempStr = tempStr[len(fill)::]
            tempStr = fill + tempStr
            tempStr = tempStr[::-1]
            self.__dot_data[col] = int(tempStr, 2)
        else:
            fill = fill[::-1]
            self.__dot_data[col] = int(fill, 2)

        self.__dot_bit_draw()
        
    def set_dotmatrix_row(self, row:int = 0, fill:str = "000000000000000"):
        """
        Set single LED on Dot Matrix

        Parameters
        --------
        row  : 0 ~ 6
        fill : Higher number to the top
        """
        for i in range(15):
            self.__dot_data[i] = bin(self.__dot_data[i])
            self.__dot_data[i] = self.__dot_data[i][2:9]
            if(len(self.__dot_data[i])<7):
                temp = "0"
                if 6-len(self.__dot_data[i]) > 0:
                    for j in range(6-len(self.__dot_data[i])):
                        temp = temp + "0"
                self.__dot_data[i] = temp + self.__dot_data[i]
        if len(fill) > 15:
            fill = fill[0:15]
        for i in range(len(fill)):
            temp = list(self.__dot_data[i])
            temp[6-row] = fill[i]
            self.__dot_data[i] = "".join(temp)

        for i in range(15):
            self.__dot_data[i] = int(self.__dot_data[i],2)
        self.__dot_bit_draw()

    def set_dotmatrix_col_all(self, fill):
        if len(fill) != 15:
            print("Invalid Input")
            return
        for i in range(len(fill)):
            if type(fill[i]) != str or len(fill[i]) != 7:
                print("Invalid Input")
                return

        for i in range(len(fill)):
            fill[i] = int(fill[i], 2)

        for i in range(len(fill)):
            self.__dot_data[i] = fill[i]
        self.__dot_bit_draw()

    def set_dotmatrix_row_all(self, fill):
        if len(fill) != 7:
            print("Invalid Input")
            return
        for i in range(len(fill)):
            if type(fill[i]) != str or len(fill[i]) != 15:
                print("Invalid Input")
                return

        for i in range(15):
            self.__dot_data[i] = 0
            for j in range(7):
                if fill[j][i] == "1":
                    self.__dot_data[i] += pow(2,j)

        self.__dot_bit_draw()
