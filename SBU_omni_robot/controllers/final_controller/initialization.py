from controller import Robot
from controller import GPS
from controller import Compass
from controller import DistanceSensor
from controller import PositionSensor
from controller import LidarPoint
from controller import Display
import math
import numpy as np

WHEEL_RADIUS = 0.029 
CHASSIS_AXLE_LENGTH = 0.22 

wheel_cirum = 2 * math.pi * WHEEL_RADIUS
encoder_unit = wheel_cirum / (2*math.pi)

gps = None
compass = None
motor_1 = None
motor_2 = None
motor_3 = None
sonar_1 = None
sonar_2 = None
sonar_3 = None
pos_1 = None
pos_2 = None
pos_3 = None
ir_1 = None
ir_2 = None
ir_3 = None
ir_4 = None
ir_5 = None
ir_6 = None
gps_values = None
compass_val = None
sonar_value = None
position_value = None
ir_value = None
robot_position = np.array([-14.28, -16.6, 0.0])
robot_omega = np.array([0.0, 0.0, 0.0])


def init_robot(time_step=32):

    global gps
    global compass
    global motor_1
    global motor_2
    global motor_3
    global sonar_1
    global sonar_2
    global sonar_3
    global pos_1
    global pos_2
    global pos_3
    global ir_1
    global ir_2
    global ir_3
    global ir_4
    global ir_5
    global ir_6

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    TIME_STEP = 32

    # define robot motors
    motor_1 = robot.getDevice("motor_1")
    motor_2 = robot.getDevice("motor_2")
    motor_3 = robot.getDevice("motor_3")  

    # set position for robot motors
    motor_1.setPosition(float('inf'))
    motor_2.setPosition(float('inf'))
    motor_3.setPosition(float('inf'))

    # set velocity for robot motors
    motor_1.setVelocity(0.0)
    motor_2.setVelocity(0.0)
    motor_3.setVelocity(0.0)

    # define distance sensors
    sonar_1 = robot.getDevice("sonar_1")
    sonar_2 = robot.getDevice("sonar_2")
    sonar_3 = robot.getDevice("sonar_3")

    # enable distance sensors
    sonar_1.enable(time_step)
    sonar_2.enable(time_step)
    sonar_3.enable(time_step)

    # define position sensors
    pos_1 = robot.getDevice("pos_1")
    pos_2 = robot.getDevice("pos_2")
    pos_3 = robot.getDevice("pos_3")  

    # enable position sensors
    pos_1.enable(time_step)
    pos_2.enable(time_step)
    pos_3.enable(time_step)

    # define infra red sensors
    ir_1 = robot.getDevice("distance_sensor1")
    ir_2 = robot.getDevice("distance_sensor2")
    ir_3 = robot.getDevice("distance_sensor3")
    ir_4 = robot.getDevice("distance_sensor4")
    ir_5 = robot.getDevice("distance_sensor5")
    ir_6 = robot.getDevice("distance_sensor6")

    # enable infra red sensors
    ir_1.enable(time_step)
    ir_2.enable(time_step)
    ir_3.enable(time_step)
    ir_4.enable(time_step)
    ir_5.enable(time_step)
    ir_6.enable(time_step)


    wheel_cirum = 2 * math.pi * WHEEL_RADIUS
    encoder_unit = wheel_cirum / (2*math.pi)

    # define and enable lidar sensors
    # lidar = robot.getDevice("head_hokuyo_sensor");
    # lidar.enable(time_step);

    # define and enable gps
    gps = robot.getDevice("gps")
    gps.enable(time_step)

    # define and enable compass
    compass = robot.getDevice("compass")
    compass.enable(time_step)

    return robot

def read_sensors_values():

    global gps_values
    global compass_val
    global sonar_value
    global position_value
    global ir_value

    # read GPS values
    gps_values = gps.getValues()

    # read compass and rotate arrow accordingly
    compass_val = compass.getValues()

    # read sonar sensors values
    sonar_value = np.array([sonar_3.getValue(),sonar_1.getValue(),sonar_2.getValue()])
    
    # read position sensors values
    position_value = encoder_unit*np.array([pos_1.getValue(),pos_2.getValue(),pos_3.getValue()])
    
    # read infra-red sensors values
    ir_value = np.array([ir_1.getValue(),ir_2.getValue(), ir_3.getValue(), ir_4.getValue(), ir_5.getValue(), ir_6.getValue()])

    return gps_values,compass_val,sonar_value,position_value,ir_value

def init_robot_state(in_pos=robot_position,in_omega=robot_omega):
    global robot_position
    global robot_omega
    # define robot state here
    robot_position = in_pos
    robot_omega    = in_omega

def update_robot_state():
    global robot_velocity
    global robot_position
    global robot_omega
    # updating the current theta
    robot_position[2] = math.atan2(compass_val[0], compass_val[1])
    
    # updating the currnet robot position
    robot_position[0] = gps_values[0]
    robot_position[1] = gps_values[1]

def update_motor_speed(input_omega=robot_omega):
    motor_1.setVelocity(input_omega[0])
    motor_2.setVelocity(input_omega[1])
    motor_3.setVelocity(input_omega[2])

def get_bearing_in_degrees(north):
  rad = math.atan2(north[0], north[1])
  bearing = (rad - 1.5708) / 3.14 * 180.0
  if (bearing < 0.0):
    bearing = bearing + 360.0
    
  return bearing