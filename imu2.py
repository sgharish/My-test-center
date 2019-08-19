import smbus			#import SMBus module of I2C
from time import sleep
import math #import
import RPi.GPIO as GPIO

#some MPU6050 Registers and their Address
PWR_MGMT_1  = 0x6B
SMPLRT_DIV  = 0x19
CONFIG    = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE  = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
G_gain = 0.07 #full scale range is +\- 2000dps based on the datasheet value this value is used as the gain of the gyro
t = 0.01 #time interval of 10ms
#global def for the kalman filterglobal
global QAngle =0.001
global QBias = 0.003
global RMeasure = 0.03
global angle = 0.0
global bias = 0.0
global rate = 0.0
global P=[[0.0,0.0],[0.0,0.0]]

def getAngle(self, newAngle, newRate, dt):
    # step 1:
    rate = newRate - self.bias;  # new_rate is the latest Gyro measurement
    angle += dt * self.rate;

    # Step 2:
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + QAngle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += QBias * dt

    # Step 3: Innovation
    y = newAngle - angle

    # Step 4: Innovation covariance
    s =P[0][0] + RMeasure

    # Step 5:    Kalman Gain
    K = [0.0, 0.0]
    K[0] = P[0][0] / s
    K[1] = P[1][0] / s

    # Step 6: Update the Angle
    angle += K[0] * y
    bias += K[1] * y

    # Step 7: Calculate estimation error covariance - Update the error covariance
    P00Temp = P[0][0]
    P01Temp = P[0][1]

    P[0][0] -= K[0] * P00Temp;
    P[0][1] -= K[0] * P01Temp;
    P[1][0] -= K[1] * P00Temp;
    P[1][1] -= K[1] * P01Temp;

    return angle


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
  
    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value
  
  
def dist(a,b):
  return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
  degree_y = (math.atan2(x, dist(y,z))+3.141)*57.295
  return (degree_y)
 # convertes the accelerometer value to degree gives the radial accelaration commonly called as the pitch 
def get_x_rotation(x,y,z):
  degree_x = float ((math.atan2(y, dist(x,z))+3.141)*57.295)
  return (degree_x)
  # convertes the accelerometer value to degree gives the tangential accelaration commonly called as the roll
def CompFx(x,y,z,gyro):
    global q_angle
    q_angle = 0.03
    q_angle_0= (0.98*(q_angle + gyro)+(0.02)*get_x_rotation(x,y,z))
    return q_angle_0
def CompFy(x,y,z,gyro):
    global q_angle
    q_angle = 0.03
    q_angle_1= (0.98*(q_angle + gyro)+(0.02)*get_y_rotation(x,y,z))
    return q_angle_1
    

   
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)
	

	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0
	
	
	gyrox = (gyro_x*G_gain)
	gyroy = (gyro_y*G_gain)
	gyroz = (gyro_z*G_gain)
	
	
	Gx = (gyro_x*G_gain)*0.01
	Gy = (gyro_y*G_gain)*0.01
	Gz = (gyro_z*G_gain)*0.01
	

	print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
	print ("X Rotation: " , get_x_rotation(Ax,Ay,Az))
	print ("Y Rotation: " , get_y_rotation(Ax,Ay,Az))
	print ("compFX=", CompFx(Ax,Ay,Az,Gx))
	print ("compFY=", CompFy(Ax,Ay,Az,Gx))


# kalman filter is implemented to find combine the values of the acceleromenter and gyro.