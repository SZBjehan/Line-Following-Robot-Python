"""line_follower_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

#we have assigned 32ms to our variable
TIME_STEP = 32

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
ir = []
irNames = ['ds_right', 'ds_mid', 'ds_left']
for i in range(3):
    ir.append(robot.getDevice(irNames[i]))
    ir[i].enable(TIME_STEP) #the sensor values are updated after every 32ms
    
wheel = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheel.append(robot.getDevice(wheelsNames[i]))
    wheel[i].setPosition(float('inf'))
    wheel[i].setVelocity(0.0) # we need our wheels to be stationary when we start the simulation
    

cm = robot.getDevice('cam')
cm.enable(TIME_STEP)
#cm.recognitionEnable(TIME_STEP)

# Main loop:
base_speed = 4 #not too low or high
kp = 1.5
ki = 0 #no past values
kd = 0.3
last_error = I = D = P = error = 0
#less than this on the track, more than this off the track
sensorTh = 700

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    
    #for each iteration, our error will be set to 0 for the next readings
    error = 0
    
    #sensor values
    ir_right_val = ir[0].getValue()
    ir_mid_val = ir[1].getValue()
    ir_left_val = ir[2].getValue()
    
    print("sensor readings left: {}  mid:{}  right:{}".format(ir_left_val, ir_mid_val, ir_right_val))
    left_speed = base_speed
    right_speed = base_speed
    
    #if all sensors are aligned on the track
    if ir_left_val<sensorTh and ir_right_val<sensorTh and ir_mid_val<sensorTh:
        error = 0
    #if only middle sensor is aligned on the track
    elif ir_left_val>=sensorTh and ir_right_val>=sensorTh and ir_mid_val<sensorTh:
        error = 0
    #if only the left sensor is on the track, extreme left turn
    elif ir_left_val<sensorTh and ir_right_val>=sensorTh and ir_mid_val>=sensorTh:
        error = 2
    #if only the right sensor is on the track, extreme right turn
    elif ir_left_val>=sensorTh and ir_right_val<sensorTh and ir_mid_val>=sensorTh:
        error = -2
    #if only the left sensor is off-track, normal right turn
    elif ir_left_val>=sensorTh and ir_right_val<sensorTh and ir_mid_val<sensorTh:
        error = -1
    #if only the right sensor is off-track, normal left turn
    elif ir_left_val<sensorTh and ir_right_val>=sensorTh and ir_mid_val<sensorTh:
        error = 1
    
    
    P = error
    I = error + I
    D = error - last_error
    balance = int(kp*P + ki*I + kd*D)
    last_error = error
    right_speed = base_speed - balance
    left_speed = base_speed + balance
    print("P: {}  I:{}  D:{} balance:{}".format(P, I, D,balance))
    print("left_speed: {}   right_speed:{}".format(left_speed, right_speed))
    
    #right turn
    if left_speed>base_speed or right_speed<0:
        wheel[1].setVelocity(left_speed)
        wheel[3].setVelocity(left_speed)
        wheel[0].setVelocity(0)
        wheel[2].setVelocity(0)
        
    #left turn
    elif right_speed>base_speed or left_speed<0:
        wheel[1].setVelocity(0)
        wheel[3].setVelocity(0)
        wheel[0].setVelocity(right_speed)
        wheel[2].setVelocity(right_speed)
        
    #go straight
    elif right_speed == base_speed or left_speed == base_speed:
        wheel[1].setVelocity(left_speed)
        wheel[3].setVelocity(left_speed)
        wheel[0].setVelocity(right_speed)
        wheel[2].setVelocity(right_speed)
    
    
    pass 

# Enter here exit cleanup code.
