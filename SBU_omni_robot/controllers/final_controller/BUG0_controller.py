import numpy as np
from initialization import * 
import math

def is_on_M_line(x_current, y_current, x0, y0,  threshold=0.05):
    goal_point_X = 1.3
    goal_point_Y = 6.15
    
    if x0 - goal_point_X != 0:
        dist = abs(y_current - ((x_current - x0) * (y0 - goal_point_Y) / (x0 - goal_point_X) + y0))
        return dist < 1
    else:
        dist = abs(x_current - goal_point_X)
        return dist < threshold
    
     
def align_to_M(heading, theta, turn_left=False):
    threshold = 5
    ts = 3  # turning speed
    if turn_left:
        ts *= -1  # turn left
    if abs(heading - theta%360) < threshold:
        return True
    else:
        update_motor_speed(input_omega=[ts, ts, ts])
        return False
        
def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def calculate_theta(x1, y1, x0, y0):
    if x1 == x0:
        return 180
    else:
        return math.degrees(math.atan((y1-y0) / (x1-x0))%360)+90      

def check_in_visited(gps_valuesX, gps_postitionY , visited_points):
    for point in visited_points:
        dist = math.sqrt((gps_valuesX-point[0])**2 + (gps_postitionY-point[1])**2)
        if dist < 0.5:
        #if(calculate_euclidean_distance(gps_valuesX, gps_postitionY, point[0], point[1])<0.5):
            return True
    return False
            
if __name__ == "__main__":
    
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0,0,0],in_omega=[0,0,0]) 
    prev = ""
    global goal_postition
    goal_postition = 1.3, 6.15
    start_position = 1.3, -9.74
    state = 'start'
    robot_speed = 8
    forward_left_speeds = [-1*robot_speed, -1*robot_speed, 2*robot_speed]
    far_from_wall_counter = 0
    close_to_wall_counter = 0
    x0, y0 = start_position
    step_back_counter = 0
    visited_points=[]
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
            distance_threshold = 0.0001
            if(is_on_M_line(gps_values[0], gps_values[1], x0, y0)):
                prev = state
                state = 'align_robot_heading'
                
        elif state == 'align_robot_heading':
            theta = calculate_theta(goal_postition[0], goal_postition[1], x0, y0)
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=theta)
            if is_aligned:
                prev = state
                state = 'move_to_goal'
                
        elif state == 'move_to_goal':
            update_motor_speed(input_omega=[-1*robot_speed, robot_speed, 0])
            difference = abs(front_ir_values[1] - front_ir_values[0])
            if (front_ir_values[0] + front_ir_values[1]) / 2 < 1000:
                prev = state
                state = 'wall_following'
                
            elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1])< 0.5):
                state = 'end'
                        
        elif state == 'wall_following':   
            # check if it must turn
            difference = front_ir_values[1] - front_ir_values[0]
            
            ###################
            # BUG 0:
            heading_front = get_bearing_in_degrees(compass_val)
            heading_left = (get_bearing_in_degrees(compass_val) + 120) % 360
            heading_right = (get_bearing_in_degrees(compass_val) - 120) % 360
            
            theta = calculate_theta(goal_postition[0], goal_postition[1], gps_values[0], gps_values[0])
            if (abs(theta - heading_left) < 5 or 175 < abs(theta - heading_left) < 185) and left_ir_values[0] == 1000 and left_ir_values[1] == 1000:
                state='one_step_back'
                if not check_in_visited(gps_values[0], gps_values[1],visited_points):
                    visited_points.append([gps_values[0], gps_values[1]])                  
                    state = 'one_step_back'
                else:
                    state='end'
            ###################
            
            if difference > 100:
                update_motor_speed(input_omega=[robot_speed, robot_speed, 0])
            elif difference < -100:
                update_motor_speed(input_omega=[-1*robot_speed, -1*robot_speed, 0])
            elif front_sonar > 0.8:
                far_from_wall_counter += 1
                if  far_from_wall_counter == 10:
                    prev = state
                    state = 'go_close'
                    far_from_wall_counter = 0
                else:
                    update_motor_speed(input_omega=[-1*robot_speed, -1*robot_speed, 2*robot_speed])
                    forward_left_speeds = [-1*robot_speed, -1*robot_speed, 2*robot_speed]
                    
            elif 100 < difference < 200:
                forward_left_speeds[0] -= 1
            elif -200 < difference < -100:
                forward_left_speeds[1] -= 1                   
      
            else:    
                update_motor_speed(input_omega=[-1*robot_speed, -1*robot_speed, 2*robot_speed])
                forward_left_speeds = [-1*robot_speed, -1*robot_speed, 2*robot_speed]
                
            if left_sonar < 120 or left_ir_values[0] < 300:
                prev = state
                state = 'turn'
                theta_0 = get_bearing_in_degrees(compass_val)
                
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
            if (front_ir_values[0] + front_ir_values[1]) / 2 < 1000 or front_sonar < 1:
                prev = state
                state = 'wall_following'

        elif state == 'turn': 
            distance_threshold = 1         
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=theta_0+90, turn_left=True)
            if is_aligned:
                prev = state
                state = 'wall_following'
        
        elif state == 'one_step_back':
            if step_back_counter == 20:
                x0, y0 = gps_values[0], gps_values[0] 
                state = 'align_robot_heading'

                step_back_counter = 0                
            else:
                step_back_counter +=1
                update_motor_speed(input_omega=[robot_speed, -1*robot_speed, 0])
       
        elif state == 'end':
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=90)
            if is_aligned:
                update_motor_speed(input_omega=[0, 0, 0]) #end
                          
        elif(calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1])< 0.5):
            state = 'end'
            update_motor_speed(input_omega=[0, 0, 0])
        
    pass