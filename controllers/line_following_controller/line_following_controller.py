"""line_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# define constants and states here.
# ----------------------------------------------------------------	
MAX_SPEED = 6.28			# testing more values would be good but not necessary.
OUT_OF_LINE_THRESHOLD = 0.1 # the value is not determined yet. trial and error should be applied.

# define functions here.
# ----------------------------------------------------------------
def forward():
	return [MAX_SPEED, MAX_SPEED, 'forward'] # left and right

def turn_right():
	return [MAX_SPEED, MAX_SPEED/4, 'turn_right'] # left and right

def rotate_right1():
	return [MAX_SPEED, 0.0, 'rotate_right1'] # left and right

def rotate_right2():
	return [MAX_SPEED, -MAX_SPEED, 'rotate_right2'] # left and right

def turn_left():
	return [MAX_SPEED/4, MAX_SPEED, 'turn_left'] # left and right

def rotate_left1():
	return [0.0, MAX_SPEED, 'rotate_left1'] # left and right

def rotate_left2():
	return [-MAX_SPEED, MAX_SPEED, 'rotate_left2']

def stop():
	return [0.0, 0.0, 'stop']

if __name__ == "__main__":

	# define variables here.
	# ------------------------------------------------------------
	left_wheel_speed = 0.0
	right_wheel_speed = 0.0
	bg_white = True
	state = 0
	action = 'stop'
	buff_size = 6
	lost_counter = 3
	actions_buffer = [0]*buff_size
	actions_index = 0
	finish_counter = 50

	# create the Robot instance.
	robot = Robot()

	# get the time step of the current world.
	timestep = int(robot.getBasicTimeStep())

	####################################################################
	# You should insert a getDevice-like function in order to get the
	# instance of a device of the robot. Something like:
	#  motor = robot.getDevice('motorname')
	#  ds = robot.getDevice('dsname')
	#  ds.enable(timestep)
	####################################################################
	left_motor = robot.getDevice('left wheel motor')
	right_motor = robot.getDevice('right wheel motor')
	left_motor.setPosition(float('inf'))
	right_motor.setPosition(float('inf'))
	left_motor.setVelocity(0.0)
	right_motor.setVelocity(0.0)

	# sensors
	ground_sensors_names = ["gs0", "gs1", "gs2", "gs3", "gs4"];
	gs = [0]*len(ground_sensors_names)
	
	print(len(ground_sensors_names))
	for i in range(len(ground_sensors_names)):
		gs[i] = robot.getDevice(ground_sensors_names[i])
		gs[i].enable(timestep)
	# enabling each sensor
	# ------------------------------------------------------------

	# Main loop:
	# - perform simulation steps until Webots is stopping the controller
	while robot.step(timestep) != -1:
	    
	    # Read the sensors:
	    # ------------------------------------------------------------
	    # Enter here functions to read sensor data, like:
	    # val = ds.getValue()
	    gs_values = np.array([0]*len(gs))
	    for i in range(len(gs)):
	    	gs_values[i] = gs[i].getValue()
 
	    isblack = gs_values<600
	    print(' ')   
	    print('gs0= {}\t'.format(isblack[0]*1), 
	    	'gs1= {}\t'.format(isblack[1]*1), 
	    	'gs2= {}\t'.format(isblack[2]*1), 
	    	'gs3= {}\t'.format(isblack[3]*1), 
	    	'gs4= {}'.format(isblack[4]*1))
	    # print('gs0 = {}'.format(gs_values[0]*1), 
	    # 	'gs1 = {}'.format(gs_values[1]*1), 
	    # 	'gs2 = {}'.format(gs_values[2]*1), 
	    # 	'gs3 = {}'.format(gs_values[3]*1), 
	    # 	'gs4 = {}'.format(gs_values[4]*1))
	    print(' ')   
	    
	    # this prints the ground sensor values, which does by replacing the {}s by the values inside the format.
	    # print(isblack)

	    ###################################### the state machine comes here. #####################################

	    ########### 1 stands for black line
	    ########### 0 stands for white line
		############################################ white background ############################################
	    if(bg_white): 

		    if(	 	 isblack[0] and 	isblack[1] and     isblack[2] and 	  isblack[3] and 	 isblack[4]): # 11111, 31, bg_white
		    	state = 31

		    elif(	 isblack[0] and 	isblack[1] and     isblack[2] and 	  isblack[3] and not isblack[4]): # 11110, 30, bg_white
		    	if(state == 31 or state == 15):
		    		left_wheel_speed, right_wheel_speed, action = forward()
		    	else:
		    		left_wheel_speed, right_wheel_speed, action = rotate_left1()
		    	state = 30

		    elif(	 isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and 	 isblack[4]): # 11101, 29, bg_white
		    	bg_white = False
		    	state = 29

		    elif(	 isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 11100, 28, bg_white
		    	if(state != 15): # 15 means there was a sharp turn
			    	if(state == 31):
			    		left_wheel_speed, right_wheel_speed, action = forward()
			    	else:
			    		left_wheel_speed, right_wheel_speed, action = rotate_left1()
		    	state = 28

		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and 	 isblack[4]): # 11011, 27, bg_white
		    	bg_white = False
		    	state = 27

		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 11010, 26, bg_white
		    	state = 26
		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 11001, 25, bg_white
		    	state = 25
		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 11000, 24, bg_white
		    	if(state != 15): # 15 means there was a sharp turn.
		    		left_wheel_speed, right_wheel_speed, action = turn_left()
		    	state = 24

		    elif(	 isblack[0] and not isblack[1] and 	   isblack[2] and 	  isblack[3] and  	 isblack[4]): # 10111, 23, bg_white
		    	bg_white = False
		    	state = 23
		    elif(	 isblack[0] and not	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 10110, 22, bg_white
		    	state = 22
		    elif(	 isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and  	 isblack[4]): # 10101, 21, bg_white
		    	state = 21
		    elif(	 isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 10100, 20, bg_white
		    	state = 20
		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and  	 isblack[4]): # 10011, 19, bg_white
		    	state = 19
		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 10010, 18, bg_white
		    	state = 18
		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 10001, 17, bg_white
		    	state = 17
		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 10000, 16, bg_white
		    	left_wheel_speed, right_wheel_speed, action = rotate_left1()
		    	state = 16

		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and 	  isblack[3] and  	 isblack[4]): # 01111, 15, bg_white
		    	if(state == 31 or state == 30):
		    		left_wheel_speed, right_wheel_speed, action = forward()
		    	else:
		    		left_wheel_speed, right_wheel_speed, action = rotate_right1()
		    	state = 15

		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 01110, 14, bg_white
		    	left_wheel_speed, right_wheel_speed, action = forward()
		    	state = 14
		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and  	 isblack[4]): # 01101, 13, bg_white
		    	state = 13
		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 01100, 12, bg_white
		    	if(state != 15): # 15 means there was a sharp turn.
		    		left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 12

		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and  	 isblack[4]): # 01011, 11, bg_white
		    	state = 11
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 01010, 10, bg_white
		    	state = 10
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 01001, 9, bg_white
		    	state = 9
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 01000, 8, bg_white
		    	bg_white = True
		    	left_wheel_speed, right_wheel_speed, action = turn_left()
		    	state = 8

		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and 	  isblack[3] and  	 isblack[4]): # 00111, 7, bg_white
		    	if(state != 30): # 30 means there was a sharp turn
			    	if(state == 31):
			    		left_wheel_speed, right_wheel_speed, action = forward()
			    	else:
			    		left_wheel_speed, right_wheel_speed, action = rotate_right1()
		    	state = 7

		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 00110, 6, bg_white
		    	if(state != 30): # 30 means there was a sharp turn
		    		left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 6

		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and  	 isblack[4]): # 00101, 5, bg_white
		    	bg_white = True
		    	state = 5

		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 00100, 4, bg_white
		    	bg_white = True
		    	left_wheel_speed, right_wheel_speed, action = forward()
		    	state = 4

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and  	 isblack[4]): # 00011, 3, bg_white
		    	if(state != 30): # 30 means there was a sharp turn.
		    		left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 3

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 00010, 2, bg_white
		    	bg_white = True
		    	left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 2

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 00001, 1, bg_white
		    	left_wheel_speed, right_wheel_speed, action = rotate_right1()
		    	state = 1
		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 00000, 0, bg_white
		    	state = 0
		    # else # dummy





		########### 1 stands for black line
	    ########### 0 stands for white line
		############################################ black background ############################################    
	    else: 
               
		    if(	 	 isblack[0] and 	isblack[1] and     isblack[2] and 	  isblack[3] and 	 isblack[4]): # 11111, 31, bg_black
		    	state = 31

		    elif (	 isblack[0] and 	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 11110, 30, bg_black
		    	left_wheel_speed, right_wheel_speed, action = rotate_right1()
		    	state = 30

		    elif(	 isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and 	 isblack[4]): # 11101, 29, bg_black
		    	bg_white = False
		    	left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 29

		    elif(	 isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 11100, 28, bg_black
		    	if(state != 1): # 1 means there was a sharp turn
		    		left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 28

		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and 	 isblack[4]): # 11011, 27, bg_black
		    	bg_white = False
		    	left_wheel_speed, right_wheel_speed, action = forward()
		    	state = 27

		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 11010, 26, bg_black
		    	state = 26
		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 11001, 25, bg_black
		    	if(state != 1): # 1 means there was a sharp turn
		    		left_wheel_speed, right_wheel_speed, action = turn_right()
		    	state = 25

		    elif(	 isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 11000, 24, bg_black
		    	if(state != 1): # 1 means it was a sharp turn
			    	if(state == 0):
			    		left_wheel_speed, right_wheel_speed, action = forward()
			    	else:
			    		left_wheel_speed, right_wheel_speed, action = rotate_right1()
		    	state = 24

		    elif(	 isblack[0] and not isblack[1] and 	   isblack[2] and 	  isblack[3] and  	 isblack[4]): # 10111, 23, bg_black
		    	bg_white = False
		    	left_wheel_speed, right_wheel_speed, action = turn_left()
		    	state = 23

		    elif(	 isblack[0] and not	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 10110, 22, bg_black
		    	state = 22
		    elif(	 isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and  	 isblack[4]): # 10101, 21, bg_black
		    	state = 21
		    elif(	 isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 10100, 20, bg_black
		    	bg_white = True
		    	state = 20

		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and  	 isblack[4]): # 10011, 19, bg_black
		    	if(state != 16): # 16 means it was a sharp turn
		    		left_wheel_speed, right_wheel_speed, action = turn_left()
		    	state = 19

		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 10010, 18, bg_black
		    	state = 18
		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 10001, 17, bg_black
		    	left_wheel_speed, right_wheel_speed, action = forward()
		    	state = 17
		    elif(	 isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 10000, 16, bg_black
		    	if(state == 0 or state == 1):
		    		left_wheel_speed, right_wheel_speed, action = forward()
		    	else:
		    		left_wheel_speed, right_wheel_speed, action = rotate_right1()
		    	state = 16

		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and 	  isblack[3] and  	 isblack[4]): # 01111, 15, bg_black
		    	left_wheel_speed, right_wheel_speed, action = rotate_left1()
		    	state = 15

		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 01110, 14, bg_black
		    	state = 14
		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and  	 isblack[4]): # 01101, 13, bg_black
		    	state = 13
		    elif(not isblack[0] and 	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 01100, 12, bg_black
		    	state = 12
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and  	 isblack[4]): # 01011, 11, bg_black
		    	state = 11
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 01010, 10, bg_black
		    	state = 10
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 01001, 9, bg_black
		    	state = 9
		    elif(not isblack[0] and 	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 01000, 8, bg_black
		    	bg_white = True
		    	state = 8

		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and 	  isblack[3] and  	 isblack[4]): # 00111, 7, bg_black
		    	if(state != 16): # 16 means it was a sharp turn
		    		left_wheel_speed, right_wheel_speed, action = turn_left()
		    	state = 7

		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and 	  isblack[3] and not isblack[4]): # 00110, 6, bg_black
		    	state = 6
		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and  	 isblack[4]): # 00101, 5, bg_black
		    	state = 5
		    elif(not isblack[0] and not	isblack[1] and 	   isblack[2] and not isblack[3] and not isblack[4]): # 00100, 4, bg_black
		    	bg_white = True
		    	state = 4

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and  	 isblack[4]): # 00011, 3, bg_black
		    	if(state != 16): # 16 means it was a sharp turn

			    	if(state == 0):
			    		left_wheel_speed, right_wheel_speed, action = forward()
			    	else:
			    		left_wheel_speed, right_wheel_speed, action = rotate_left1()
		    	state = 3

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and 	  isblack[3] and not isblack[4]): # 00010, 2, bg_black
		    	bg_white = True
		    	state = 2

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and  	 isblack[4]): # 00001, 1, bg_black
		    	if(state == 0 or state == 16):
		    		left_wheel_speed, right_wheel_speed, action = forward()
		    	else:
		    		left_wheel_speed, right_wheel_speed, action = rotate_left1()
		    	state = 1

		    elif(not isblack[0] and not	isblack[1] and not isblack[2] and not isblack[3] and not isblack[4]): # 00000, 0, bg_black
		    	state = 0

		    # else # dummy
              



	    # Process sensor data here.
	    # ------------------------------------------------------------
	    # if(actions_index == states_num):
	    # 	actions_index = 0
	    # actions_record[actions_index] = action
	    # for i in range(states_num):
	    # 	actions_record[actions_index]
	    if(actions_index>=buff_size):
	    	actions_index = 0
	    if(state==0 and bg_white): # meaning the robot is losing track
	    	lost_counter = lost_counter -  1 
	    elif(state==31 and not bg_white): # meaning the robot is losing track
	    	lost_counter = lost_counter - 1
	    else:
	    	lost_counter = 3
	    	actions_buffer[actions_index] = action

	    if(lost_counter<0):
	    	# should take a majority function for the actions.
	    	pass

	    if(finish_counter<0):
    		left_wheel_speed, right_wheel_speed, action = stop()
	    elif((state == 31 and bg_white) or (state == 0 and not bg_white)):
    		finish_counter = finish_counter - 1
    		# print(finish_counter)
    		left_wheel_speed, right_wheel_speed, action = forward()
	    else:
    		finish_counter = 50
	    

	    # Enter here functions to send actuator commands, like:
	    # ------------------------------------------------------------
	    #  motor.setPosition(10.0)

	    #left_wheel_speed, right_wheel_speed = forward() # for testing


	    print('action: {}'.format(action))
	    if(bg_white == True):
	    	print('background = white')
	    else:
	    	print('background = black')
	    
	    left_motor.setVelocity(left_wheel_speed)
	    right_motor.setVelocity(right_wheel_speed)
	    print('--------------------------------------------------------------------------------------------------------------------------')
	    pass

	# Enter here exit cleanup code.
