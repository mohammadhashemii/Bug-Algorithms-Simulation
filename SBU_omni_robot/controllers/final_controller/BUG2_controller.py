# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.


import numpy as np
from initialization import * 
import math

def is_on_M_line(x, y, threshold=0.1):
    goal_point_X = 1.3
    
    return abs(x - goal_point_X) < threshold
     
def align_to_M(heading, theta, turn_left=False):
    threshold = 1
    ts = 4  # turning speed
    if turn_left:
        ts *= -1  # turn left
    if abs(heading - theta) < threshold:
        return True
    else:
        
        update_motor_speed(input_omega=[ts, ts, ts])
        return False
        
def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
    
if __name__ == "__main__":
    
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0,0,0],in_omega=[0,0,0]) 
    prev = ""

    goal_postition = 1.3, 6.15
    
    # DEFINE STATES HERE!
    state = 'start'
    robot_speed = 5
    forward_left_speeds = [-1*robot_speed, -1*robot_speed, 2*robot_speed]
    far_from_wall_counter = 0
    close_to_wall_counter = 0
    hit_point = []  # x, y
    leave_point = []  # x, y
    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,encoder_value,ir_value = read_sensors_values()
        front_ir_values = ir_value[0], ir_value[3]
        right_ir_values = ir_value[2], ir_value[5]
        left_ir_values = ir_value[1], ir_value[4]
        left_sonar = sonar_value[2]
        right_sonar = sonar_value[0]
        front_sonar = sonar_value[1]
        
        update_robot_state()       
        # DEFINE STATE MACHINE HERE!
        
        if state == 'start':
            if(is_on_M_line(gps_values[0], gps_values[1])):
                prev = state
                state = 'align_robot_heading'
                
        elif state == 'align_robot_heading':
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=180)
            if is_aligned:
                prev = state
                state = 'move_to_goal'
                
        elif state == 'move_to_goal':
            update_motor_speed(input_omega=[-1*robot_speed, robot_speed, 0])
            difference = abs(front_ir_values[1] - front_ir_values[0])
            if (front_ir_values[0] + front_ir_values[1]) / 2 < 1000:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
                
            elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1])< 0.5):
                state = 'end'
                
        
        elif state == 'wall_following':
            # check if it must turn

            difference = front_ir_values[1] - front_ir_values[0]
            print(difference)
            if difference > 150:
                update_motor_speed(input_omega=[robot_speed, robot_speed, 0])
            elif difference < -150:
                update_motor_speed(input_omega=[-1*robot_speed, -1*robot_speed, 0])
            elif front_sonar > 0.7:
                far_from_wall_counter += 1
                if  far_from_wall_counter == 10:
                    prev = state
                    state = 'go_close'
                    far_from_wall_counter = 0
                else:
                    update_motor_speed(input_omega=[-1*robot_speed, -1*robot_speed, 2*robot_speed])
                    forward_left_speeds = [-1*robot_speed, -1*robot_speed, 2*robot_speed]
                    
            elif(is_on_M_line(gps_values[0], gps_values[1])) and gps_values[1]<9 and prev == 'wall_following':
                leave_point.append([gps_values[0], gps_values[1]])
             
                if calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_postition[0], goal_postition[1]):
                    prev = state
                    state = 'align_robot_heading' 
                else:
                    continue


            #if 50< difference < 200:
            #    forward_left_speeds[0] -= 1
            #elif -200 < difference < -50:
            #    forward_left_speeds[1] -= 1                   
      
            else:    
                update_motor_speed(input_omega=[-1*robot_speed, -1*robot_speed, 2*robot_speed])
                forward_left_speeds = [-1*robot_speed, -1*robot_speed, 2*robot_speed]
            
            if(is_on_M_line(gps_values[0], gps_values[1]) and prev =='wall_following' and gps_values[1]<9):
                leave_point.append([gps_values[0], gps_values[1]])
                if calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1]) < calculate_euclidean_distance(hit_point[-1][0], hit_point[-1][1], goal_postition[0], goal_postition[1]):
                    prev = state
                    state = 'align_robot_heading'
                else:
                    continue
                
            # check left and right sonar values
            if left_sonar < 200 or left_ir_values[0] < 300:
                prev = state
                state = 'turn'
                theta_0 = get_bearing_in_degrees(compass_val)
                
                
            #elif (front_ir_values[0] < 500 or front_ir_values[1] < 500) or front_sonar < 2:
            if front_sonar < 0.3:     
                close_to_wall_counter += 1                
                if close_to_wall_counter >= 20:
                    prev = state
                    state = 'go_reverse'
                            
        elif state == 'go_reverse':
            
            update_motor_speed(input_omega=[robot_speed, -1*robot_speed, 0])
            if front_sonar > 0.8:
                prev = state         
                state = 'wall_following'
                close_to_wall_counter = 0
                
        elif state == 'go_close':
            update_motor_speed(input_omega=[-1*robot_speed, robot_speed, 0])
            difference = abs(front_ir_values[1] - front_ir_values[0])
            if (front_ir_values[0] + front_ir_values[1]) / 2 < 900 or front_sonar < 1:
                prev = state
                state = 'wall_following'

        elif state == 'turn':          
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=theta_0+90, turn_left=True)
            if is_aligned:
                prev = state
                state = 'wall_following'
            
       
        elif state == 'end':
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=90)
            if is_aligned:
                update_motor_speed(input_omega=[0, 0, 0]) #end
                
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1])< 0.5):
            state = 'end'
            update_motor_speed(input_omega=[0, 0, 0])
        
    pass