# main.py

from vex import *
import time

# Initialize devices
brain = Brain()
controller_1 = Controller(PRIMARY)
left_motor_a = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
left_motor_b = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
left_motor_c = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
 
right_motor_a = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
right_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
right_motor_c = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)
 
left_drive_smart = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b, right_motor_c)
High_scoring = Motor(Ports.PORT20)
intake_lower = Motor(Ports.PORT21)
intake_upper = Motor(Ports.PORT13)
mogo_p = DigitalOut(brain.three_wire_port.f)
ejection_p = DigitalOut(brain.three_wire_port.a)
ejection_p.set(False)
donker = DigitalOut(brain.three_wire_port.h)
donker.set(False)
intake_p = DigitalOut(brain.three_wire_port.d)
rotational_sensor = Rotation(Ports.PORT19, True)
rotational_sensor.set_position(0, DEGREES)

vertical_rotational_sensor = Rotation(Ports.PORT7, True)
horizontal_rotational_sensor = Rotation(Ports.PORT6, False)
vertical_rotational_sensor.set_position(0, DEGREES)
horizontal_rotational_sensor.set_position(0, DEGREES)


# Constants
MSEC_PER_SEC = 1000

# define an enum for intake state
class IntakeState:
    STOPPED = 0
    RUNNING = 1
    STALLED = 2
    FIXINGSTALL = 3

class RingType:
    NONE = 0
    RED = 1
    BLUE = 2

MIN_REJECT_SIZE=5000
# Define the color signatures based of the config copied below
REDD = Signature(1, 9907, 12073, 10990, -1991, -879, -1435, 2.5, 0)
BLUEE = Signature(2, -4415, -3205, -3810, 5461, 8989, 7225, 2.5, 0)
'''{
  "brightness": 50,
  "signatures": [
    {
      "name": "REDD",
      "parameters": {
        "uMin": 9907,
        "uMax": 12073,
        "uMean": 10990,
        "vMin": -1991,
        "vMax": -879,
        "vMean": -1435,
        "rgb": 5973535,
        "type": 0,
        "name": "REDD"
      },
      "range": 2.5
    },
    {
      "name": "BLUEE",
      "parameters": {
        "uMin": -4415,
        "uMax": -3205,
        "uMean": -3810,
        "vMin": 5461,
        "vMax": 8989,
        "vMean": 7225,
        "rgb": 1714760,
        "type": 0,
        "name": "BLUEE"
      },
      "range": 2.5
    }
  ],
  "codes": []
}'''
Color_sensor = Optical(Ports.PORT15)
Color_sensor.set_light_power(100)
# Initialize eject_counter
eject_counter = 0
eject_object = RingType.NONE

intake_state = IntakeState.STOPPED

# Global variables
slow_drive = False
high_scoring_running = False
current_direction = FORWARD
high_scoring_mode = False
# Constants
STALL_THRESHOLD = 0       # Adjust as needed
STALL_COUNT = 20
RETRY_LIMIT = 15
EJECT_LIMIT= 20
MSEC_PER_SEC = 1000
# Define constants for the target angles
HIGH_SCORE_TARGET_ANGLE_SCORE = -430
HIGH_SCORE_TARGET_ANGLE_WAIT = -200
HIGH_SCORE_TARGET_ANGLE_CAPTURE = -60
HIGH_SCORE_TARGET_ANGLE_DOWN = 0
MAX_CAPTURE_POSITION_COUNT = 51
# Global variables
retry_count = 0
consecutive_stall_count = 0
high_scoring_running = False
high_score_stall = False  # Set this accordingly in your main code if needed
high_score_target_angle = HIGH_SCORE_TARGET_ANGLE_DOWN
capture_position_counter = 0

def set_high_score_angle(angle):
    global high_score_target_angle, capture_position_counter
    if (angle == HIGH_SCORE_TARGET_ANGLE_CAPTURE):
        high_score_target_angle = angle
        capture_position_counter = MAX_CAPTURE_POSITION_COUNT
    elif (angle == HIGH_SCORE_TARGET_ANGLE_SCORE) and high_score_target_angle <= angle:
        high_score_target_angle -= 40
    else:
        high_score_target_angle = angle

# Function to set the state of the high scoring motor
def adjust_high_scoring_motor_position():
    global high_score_target_angle, capture_position_counter

    #print(" Rotating angle is " + str(rotational_sensor.position(DEGREES)) + "high score motor angle is " + str(High_scoring.position(DEGREES)))
    High_scoring.set_stopping(BRAKE)
    High_scoring.set_velocity(100, PERCENT)
    if high_score_target_angle == HIGH_SCORE_TARGET_ANGLE_CAPTURE and abs(High_scoring.position(DEGREES) - rotational_sensor.position(DEGREES)) > 2:
        if capture_position_counter > 0:
            capture_position_counter -= 1
        else:
            print("Chaning motor position")
            print(" Rotating angle is " + str(rotational_sensor.position(DEGREES)) + "high score motor angle is " + str(High_scoring.position(DEGREES)))
            High_scoring.set_position(rotational_sensor.position(DEGREES), DEGREES)
    High_scoring.spin_to_position(high_score_target_angle, DEGREES, 30, PERCENT, False)

def auto_adjust_high_scoring_motor_position():
    global high_score_target_angle, capture_position_counter

    stall_detection_and_handling()

    #print(" Rotating angle is " + str(rotational_sensor.position(DEGREES)) + "high score motor angle is " + str(High_scoring.position(DEGREES)))
    High_scoring.set_stopping(BRAKE)
    High_scoring.set_velocity(100, PERCENT)
    if high_score_target_angle == HIGH_SCORE_TARGET_ANGLE_CAPTURE and abs(High_scoring.position(DEGREES) - rotational_sensor.position(DEGREES)) > 2:
        if capture_position_counter > 0:
            capture_position_counter -= 1
        else:
            print("Chaning motor position")
            print(" Rotating angle is " + str(rotational_sensor.position(DEGREES)) + "high score motor angle is " + str(High_scoring.position(DEGREES)))
            High_scoring.set_position(rotational_sensor.position(DEGREES), DEGREES)
    High_scoring.spin_to_position(high_score_target_angle, DEGREES, 30, PERCENT, False)
    while (High_scoring.position(DEGREES) != high_score_target_angle):
        stall_detection_and_handling()

# Function to set the state of the intake motor
def set_intake_motor_state(direction=FORWARD):
    global intake_state, current_direction, eject_counter, high_score_stall
    if intake_state == IntakeState.RUNNING or intake_state == IntakeState.FIXINGSTALL:
        intake_lower.set_velocity(90, PERCENT)
        intake_upper.set_velocity(80, PERCENT)
        intake_lower.spin(direction)
        if intake_state == IntakeState.FIXINGSTALL:
            if not high_score_stall:
                print("Intake motor state is fixing stall")
                intake_upper.spin(direction)
            else:
                high_score_stall = False
                print("Just stopping as this is high scoring stall " + str(direction))
                intake_upper.stop()
        else:
            intake_upper.spin(REVERSE if direction == FORWARD else FORWARD)
        current_direction = direction
    else:
        intake_lower.stop()
        intake_upper.stop()

# Stall detection and handling for the intake motor
def stall_detection_and_handling():
    global intake_state, consecutive_stall_count, retry_count, high_score_stall, high_score_target_angle, high_scoring_running, eject_counter
    global current_direction
    if intake_state == IntakeState.RUNNING or intake_state == IntakeState.STALLED:
        if intake_state == IntakeState.RUNNING and eject_counter > 0:
                eject_counter = eject_counter - 1
                #print("Decremeting eject counter " + str(eject_counter))
                if eject_counter == 0:
                    #print("stopping the motor momentarily")
                    intake_state = IntakeState.STOPPED
                    set_intake_motor_state(current_direction)
                    wait(100, MSEC)
                    #intake_state = IntakeState.RUNNING
                    #set_intake_motor_state(current_direction)
        current_velocity = intake_upper.velocity(PERCENT)
        if abs(current_velocity) <= STALL_THRESHOLD:
            #print("Stalled" + str(consecutive_stall_count))
            consecutive_stall_count += 1
        else:
            consecutive_stall_count = 0

        if consecutive_stall_count >= STALL_COUNT:
            print("Unstaling")
            intake_state = IntakeState.FIXINGSTALL
            # This state will change upper motor in opposite direction
            if high_scoring_running:
                print("High scoring stall setting it true")
                high_score_stall = True
                set_intake_motor_state(current_direction)
                set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_WAIT)
                adjust_high_scoring_motor_position()
            else:
                set_intake_motor_state(current_direction)

            consecutive_stall_count = 0
            retry_count = RETRY_LIMIT
    else:
        consecutive_stall_count = 0
    if intake_state == IntakeState.FIXINGSTALL:
        if retry_count == 0:
            if high_score_stall:
                print("Stoppping because of high stall")
                intake_state = IntakeState.STOPPED
                set_intake_motor_state(FORWARD)
            else:
                print("Fixed")
                intake_state = IntakeState.RUNNING
                set_intake_motor_state(current_direction)
        else:
            #print("Retrying")
            retry_count -= 1


# wait for rotation sensor to fully initialize
wait(30, MSEC)


#-----------------------------------------------------------------------------------------------------------------------------------------------------


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    #urandom.seed(int(random))
     
# Set random seed
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

#gyro start
gyro = Inertial(Ports.PORT9)
gyro.orientation(OrientationType.YAW)
gyro.calibrate()
gyro.set_rotation(0, DEGREES)
gyro.set_heading(0, DEGREES)
 
 
tolerance = 6
lookahead = 50
current_x = -1
current_y =  -1
previous_vertical_encoder = 0
previous_horizontal_encoder = 0
robot_not_walking = 0
MAX_WAIT_FOR_NO_WALK= 10
forward_velocity = 40
turn_velocity_k = 40
left_velocity = 5
right_velocity = 5
#forward_velocity/100
odom_wheel_circumference = math.pi * 2  # wheel diameter in inches
feet_to_unit = 2.5
odom_gear_ratio = 1
current_angle = 0


def verticalEncoder():
    global vertical_rotational_sensor
    return vertical_rotational_sensor.position(DEGREES)

def horizontalEncoder():
    global horizontal_rotational_sensor
    return horizontal_rotational_sensor.position(DEGREES)

position_debug = True
def update_position():
    global current_x, current_y, current_angle, previous_vertical_encoder, previous_horizontal_encoder, robot_not_walking, position_debug

    # Calculate the distance traveled by each pod
    vertical_encoder = ((verticalEncoder() / 360) * odom_wheel_circumference * odom_gear_ratio) * feet_to_unit
    horizontal_encoder = ((horizontalEncoder() / 360) * odom_wheel_circumference * odom_gear_ratio) * feet_to_unit
    delta_vertical = vertical_encoder - previous_vertical_encoder
    delta_horizontal = horizontal_encoder - previous_horizontal_encoder

    if delta_vertical == delta_horizontal and delta_vertical == 0:
        robot_not_walking = robot_not_walking + 1
    else:
        robot_not_walking = 0
    previous_vertical_encoder = vertical_encoder
    previous_horizontal_encoder = horizontal_encoder

    current_angle = 2* math.pi - math.radians(gyro.heading(DEGREES))

    # Calculate the robot's linear change
    delta_d = delta_vertical
    delta_x = delta_horizontal * math.cos(current_angle)
    delta_y = delta_horizontal * math.sin(current_angle)

    current_y += delta_d * math.sin(current_angle)
    current_x += delta_d * math.cos(current_angle)
   

    if position_debug:
        print("x: " + str(current_x) + " y: " + str(current_y) + " angle: " + str(current_angle))

def calculate_lookahead_point(points_list, lookahead_distance):
    global current_x, current_y, start_pos_size, forward_velocity, tolerance
    closest_offset = -1
    lookahead_offset = -1
    closest_distance = float('inf')

    #if len(points_list) == 0:
    #    return
    min_distance = float('inf')
    min_index = -1  # To keep track of the nearest valid point index

    num_points = len(points_list)  # Number of points to check
    for i in range(num_points-1):
        dist = math.sqrt((points_list[i][0] - current_x) ** 2 + (points_list[i][1] - current_y) ** 2)    
        if dist < tolerance:
            min_index = i
    if min_index != -1:
        del points_list[:min_index]
        min_index = -1
        num_points = len(points_list)  # Number of points to check

    if len(points_list) == 0:
        return
    lookahead_point = None
    closest_point = points_list[0]
    for i in range(num_points-1):
        start = points_list[i]
        end = points_list[i + 1]
        segment_length = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
        if segment_length == 0:
            continue
        t = ((current_x - start[0]) * (end[0] - start[0]) + (current_y - start[1]) * (end[1] - start[1])) / segment_length ** 2
        t = max(0, min(1, t))
        closest_x = start[0] + t * (end[0] - start[0])
        closest_y = start[1] + t * (end[1] - start[1])
        distance = math.sqrt((closest_x - current_x) ** 2 + (closest_y - current_y) ** 2)

        if len(points_list) == 2 and distance <  2* tolerance:
            closest_point = (points_list[1][0], points_list[1][1])
            del points_list[0]
            break

        if distance < closest_distance:
            closest_distance = distance
            closest_offset = i
            closest_point = (closest_x, closest_y)

        if distance >= lookahead_distance:
            if lookahead_point is None:
                temp_smallest_lookahead_distance = distance
                lookahead_offset = i
                lookahead_point = (closest_x, closest_y)
            elif distance < temp_smallest_lookahead_distance:
                temp_smallest_lookahead_distance = distance
                lookahead_offset = i
                lookahead_point = (closest_x, closest_y)

    if closest_offset > 0 and lookahead_point is None:
        #print("Dropping1 :" + str(points_list[:closest_offset]))
        del points_list[:closest_offset]
        closest_offset = 0
    if lookahead_point:
        #print("Dropping2 :" + str(points_list[:lookahead_offset]))
        del points_list[:lookahead_offset]
    return lookahead_point if lookahead_point else closest_point

# Function to calculate drive speeds
def calculate_drive_speeds(lookahead_point, direction):
    global current_x, current_y, current_angle, left_velocity, right_velocity, forward_velocity, turn_velocity_k
    dx = lookahead_point[0] - current_x
    dy = lookahead_point[1] - current_y

    # Calculate the angle to the target point
    point_angle = math.atan2(dy, dx)
   
    # Adjust the current angle based on the direction
    adjusted_current_angle = current_angle
    if direction == -1:
        adjusted_current_angle += math.pi  # Add 180 degrees (π radians) to the current angle

    # Normalize the adjusted current angle to be within the range [-π, π]
    adjusted_current_angle = (adjusted_current_angle + math.pi) % (2 * math.pi) - math.pi

    # Calculate the angle difference between the adjusted current heading and the target point
    point_angle_diff = point_angle - adjusted_current_angle


    # Normalize the angle difference to be within the range [-π, π]
    if point_angle_diff > math.pi:
        point_angle_diff -= 2 * math.pi
    elif point_angle_diff < -math.pi:
        point_angle_diff += 2 * math.pi

    #point_angle_diff = (point_angle_diff + math.pi) % (2 * math.pi) - math.pi

    # Calculate the wheel velocities based on the specified direction
    curr_forward_velocity = forward_velocity * direction
    curr_turn_velocity_k = turn_velocity_k
    left_velocity = curr_forward_velocity - point_angle_diff * curr_turn_velocity_k
    right_velocity = curr_forward_velocity + point_angle_diff * curr_turn_velocity_k

    # Clamp the velocities to the range [-100, 100]
    left_velocity = max(min(left_velocity, 100), -100)
    right_velocity = max(min(right_velocity, 100), -100)

    global current_x, current_y, current_angle, left_velocity, right_velocity, forward_velocity, turn_velocity_k
    dx = lookahead_point[0] - current_x
    dy = lookahead_point[1] - current_y

    # Calculate the angle to the target point
    point_angle = math.atan2(dy, dx)
   
    # Adjust the current angle based on the direction
    adjusted_current_angle = current_angle
    if direction == -1:
        adjusted_current_angle += math.pi  # Add 180 degrees (π radians) to the current angle

    # Normalize the adjusted current angle to be within the range [-π, π]
    adjusted_current_angle = (adjusted_current_angle + math.pi) % (2 * math.pi) - math.pi

    # Calculate the angle difference between the adjusted current heading and the target point
    point_angle_diff = point_angle - adjusted_current_angle


    # Normalize the angle difference to be within the range [-π, π]
    if point_angle_diff > math.pi:
        point_angle_diff -= 2 * math.pi
    elif point_angle_diff < -math.pi:
        point_angle_diff += 2 * math.pi

    #point_angle_diff = (point_angle_diff + math.pi) % (2 * math.pi) - math.pi

    # Calculate the wheel velocities based on the specified direction
    curr_forward_velocity = forward_velocity * direction
    curr_turn_velocity_k = turn_velocity_k
    left_velocity = curr_forward_velocity - point_angle_diff * curr_turn_velocity_k
    right_velocity = curr_forward_velocity + point_angle_diff * curr_turn_velocity_k

    # Clamp the velocities to the range [-100, 100]
    left_velocity = max(min(left_velocity, 100), -100)
    right_velocity = max(min(right_velocity, 100), -100)

def walk_path(points_list, lookahead_distance, stop_threshold, direction, decl = False, decl_dis = 22, decl_rate = 0.6, last_point_tolerance = 2.5):

    global current_x, current_y, start_pos_size, forward_velocity, turn_velocity_k, left_velocity, right_velocity, robot_not_walking
   
    original_list = len(points_list)

    numDeceleratePoints = 0
    start_pos_size = len(points_list)

    if current_x == -1:
        current_x = points_list[0][0]
        current_y = points_list[0][1]

    vertical_rotational_sensor.changed(update_position)
    horizontal_rotational_sensor.changed(update_position)

    running = True
    while running:
        adjust_high_scoring_motor_position()
        check_vision_sensor()
        stall_detection_and_handling()
        if len(points_list) == 0 or robot_not_walking > MAX_WAIT_FOR_NO_WALK:
            running = False
            robot_not_walking = 0
            break

        # Calculate the lookahead point
        next_point = calculate_lookahead_point(points_list, lookahead_distance)

        # Calculate drive speeds based on the specified direction
        calculate_drive_speeds(next_point, direction)
        #print("x: "+ str(current_x)+" y: " + str(current_y) + " angle: " + str(current_angle) + " lspeed" + str(left_velocity) + " rspeed" + str(right_velocity))
        #len(points_list) < original_list/10 and
        if (len(points_list) != 0):
            dis_to_end = math.sqrt((points_list[-1][0] - current_x) ** 2 + (points_list[-1][1] - current_y) ** 2)
            #print(dis_to_end)
            if dis_to_end < decl_dis and decl:
                #print("declerating" + str((dis_to_end/(dis_to_end+5))) + " " + str(len(points_list)))
                if (dis_to_end < decl_dis /2):
                    decl_rate = 0.4
                    left_velocity = left_velocity * decl_rate
                    right_velocity = right_velocity * decl_rate
                else:
                    left_velocity = left_velocity * (dis_to_end/(dis_to_end+(dis_to_end/2))) * decl_rate
                    right_velocity = right_velocity * (dis_to_end/(dis_to_end+(dis_to_end/2))) * decl_rate
       
        # Update the robot's position/stop
        update_position()

        # Check if the robot has reached the current target point
        distance_to_point = math.sqrt((points_list[0][0] - current_x) ** 2 + (points_list[0][1] - current_y) ** 2)
        if len(points_list) == 1:
            final_distance = math.sqrt((points_list[-1][0] - current_x) ** 2 + (points_list[-1][1] - current_y) ** 2)
            if final_distance < last_point_tolerance:
                running = False
        elif distance_to_point < stop_threshold:  # Adjust the threshold as needed
            points_list.pop(0)  # Remove the reached point

        # Check if the robot has reached the last point
       

        # Set motor velocities
        left_drive_smart.set_velocity(left_velocity, PERCENT)
        left_drive_smart.spin(FORWARD)
        right_drive_smart.set_velocity(right_velocity, PERCENT)
        right_drive_smart.spin(FORWARD)
        #print("(" + str(current_x)+"," + str(current_y) + "),")

        wait(20, MSEC)

    # Stop motors when path is complete

    #print(dis_to_end)
    vertical_rotational_sensor.changed(lambda: None)
    horizontal_rotational_sensor.changed(lambda: None)
    left_drive_smart.stop()
    right_drive_smart.stop()

def autonomous_sample():
    global current_x, current_y, current_angle
    print("Starting autonomous sample")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    while True:
        update_position()
        print("x: "+ str(current_x)+" y: " + str(current_y) + " angle: " + str(current_angle))
        wait(1, SECONDS)

def autonomous_blue_right():
    autonomous_more_donuts_side(blue_right_tomogo, blue_right_tofirststack, blue_right_lasttwo, first_blue_right_4, blue_right_back_4, blue_right_totower)

def autonomous_red_left():
    autonomous_more_donuts_side(red_left_tomogo, red_left_tofirststack, red_left_lasttwo, first_red_left_4, red_left_back_4, red_left_totower)

def autonomous_red_right():
    global p1redight, p2redright, p3redright, p4redright, p7redright
    autonomous_more_donuts_side_modified(red_right_tomogo, red_right_tofirststack, red_right_lasttwo, first_red_right_4, red_right_back_4, red_right_totower)
    #autonomous_extra_mogo_side(p1redright, p2redright, p3redright, p4redright, p7redright)

def autonomous_blue_left():
    global p1blueleft, p2blueleft, p3blueleft, p4blueleft, p7blueleft
    autonomous_more_donuts_side(blue_left_tomogo, blue_left_tofirststack, blue_left_lasttwo, first_blue_left_4, blue_left_back_4, blue_left_totower)
    #autonomous_extra_mogo_side(p1blueleft, p2blueleft, p3blueleft, p4blueleft, p7blueleft)

def autonomous_extra_mogo_side(p1, p2, p3, p4, p7):
    global intake_state, lookahead, tolerance
    #confirm about tolerance and direction    
    lookahead = 50
    print("autonomous_red_right: before p1")  
    walk_path(p1, lookahead, tolerance, 1)
   
    walk_path(p2, lookahead, tolerance, -1)
    print("autonomous_red_right: before p2")
    mogo_p.set(True)
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_WAIT)
    adjust_high_scoring_motor_position()
    wait(10, MSEC)
    intake_state = IntakeState.RUNNING
    set_intake_motor_state(REVERSE)
    wait(1000, MSEC)
   
    walk_path(p3, lookahead, tolerance, 1)
    print("autonomous_red_right: before p3")
 
    walk_path(p4, lookahead, tolerance, 1)
    print("autonomous_red_right: before p4")
    mogo_p.set(False)
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_DOWN)
    adjust_high_scoring_motor_position()

    walk_path(p7, lookahead, tolerance, 1)
    print("autonomous_red_right: before p4")
   
    """walk_path(p5, lookahead, tolerance, 1)
    print("autonomous_red_right: before p6")
    wait(100, MSEC)
    donker.set(True)"""
   
    #walk_path(p6, lookahead, tolerance, 1)
   
    print("autonomous_red_right: before p7")
   
    """walk_path(p5, lookahead, tolerance, 1)
    print("autonomous_red_right: END")"""
       

def autonomous_more_donuts_side(tomogo, tofirststack, last_two, first_4, back_4, to_tower):
    global intake_state, lookahead, high_score_target_angle, tolerance, forward_velocity, turn_velocity_k

    # Bring up high scoring motor
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_WAIT)
    adjust_high_scoring_motor_position()

    lookahead = 50
    tolerance = 2
    # go to mogo
    walk_path(tomogo, lookahead, tolerance, -1)
    # Capture the mogo
    mogo_p.set(True)
    wait(100, MSEC)

    # start intake to pick up the top donut including the stall code
    intake_state = IntakeState.RUNNING
    set_intake_motor_state(REVERSE)

    # Bring down the intake to knock off the top donut
    update_position()
    print("autonomous_more_donuts_side: before tofirststack")
    walk_path(tofirststack, lookahead, tolerance, 1)
    update_position()
    lookahead = 20
    tolerance = 6
    print("autonomous_more_donuts_side: before last_two")
    walk_path(last_two, lookahead, tolerance, 1)
    print("autonomous_more_donuts_side: before first_4")
    update_position()
    lookahead = 50
    walk_path(first_4, lookahead, tolerance, 1)
    print("autonomous_more_donuts_side: before to_tower")
    back_reverse = back_4[::-1]
    print("autonomous_more_donuts_side: going back1")
    walk_path(back_4, lookahead, tolerance, -1)
    walk_path(back_reverse, lookahead, tolerance, 1)
    print("autonomous_more_donuts_side: going back2")
    walk_path(back_4, lookahead, tolerance, -1)
    walk_path(back_reverse, lookahead, tolerance, 1)
    walk_path(back_4, lookahead, tolerance, -1)
    # Bring up high scoring motor
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_DOWN)
    adjust_high_scoring_motor_position()
    intake_state = IntakeState.STOPPED
    set_intake_motor_state()
    walk_path(to_tower, lookahead, tolerance, 1)
def autonomous_more_donuts_side_modified(tomogo, tofirststack, last_two, first_4, back_4, to_tower):
    global intake_state, lookahead, high_score_target_angle, tolerance, forward_velocity, turn_velocity_k
    # Bring up high scoring motor
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_WAIT)
    adjust_high_scoring_motor_position()

    lookahead = 50
    tolerance = 2
    # go to mogo
    walk_path(tomogo, lookahead, tolerance, -1)
    # Capture the mogo
    mogo_p.set(True)
    wait(100, MSEC)

    # start intake to pick up the top donut including the stall code
    intake_state = IntakeState.RUNNING
    set_intake_motor_state(REVERSE)

    # Bring down the intake to knock off the top donut
    update_position()
    print("autonomous_more_donuts_side: before tofirststack")
    walk_path(tofirststack, lookahead, tolerance, 1)
    update_position()
    lookahead = 20
    tolerance = 6
    print("autonomous_more_donuts_side: before last_two")
    #walk_path(last_two, lookahead, tolerance, 1)
    print("autonomous_more_donuts_side: before first_4")
    update_position()
    lookahead = 50
    walk_path(first_4, lookahead, tolerance, 1)
    print("autonomous_more_donuts_side: before to_tower")
    back_reverse_first = back_4[::-1]
    back_reverse_second = back_4[::-1]
    back_4_first = back_4[:]
    back_4_second = back_4[:]
    back_4_third = back_4[:]
    print("autonomous_more_donuts_side: going back1")
    walk_path(back_4_first, lookahead, tolerance, -1)
    walk_path(back_reverse_first, lookahead, tolerance, 1)
    wait(300, MSEC)
    print("autonomous_more_donuts_side: going back2")
    walk_path(back_4_second, lookahead, tolerance, -1)
    walk_path(back_reverse_second, lookahead, tolerance, 1)
    wait(300, MSEC)
    walk_path(back_4_third, lookahead, tolerance, -1)
    # Bring up high scoring motor
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_DOWN)
    adjust_high_scoring_motor_position()
    intake_state = IntakeState.STOPPED
    set_intake_motor_state()
    turn_velocity_k = 40
    forward_velocity = 40
    walk_path(to_tower, lookahead, tolerance, 1)

# driver.py
def valid_seen_object(seen_objects):
    # A placeholder to check if the objects array is valid
    if len(seen_objects) > 0:
        for obj in seen_objects:
            object_size = obj.width * obj.height
            #print("Detected object with size: " + str(object_size))
            if object_size > MIN_REJECT_SIZE:
                return True
    return False

BRIGHTNESS_THRESHOLD = 0
# Function to check the vision sensor
def check_vision_sensor():
    global eject_object
    if Color_sensor.brightness() > BRIGHTNESS_THRESHOLD:
        if eject_object == RingType.RED:
            if Color_sensor.color() == Color.RED:
               print("Ejecting Red")
               ejection_p.set(True)
            elif Color_sensor.color() == Color.BLUE:
               ejection_p.set(False)
        else:
            if eject_object == RingType.BLUE:
                if Color_sensor.color() == Color.BLUE:
                    #print("Ejecting Blue")
                    ejection_p.set(True)
                elif Color_sensor.color() == Color.RED:
                    ejection_p.set(False)

# Function to display joystick positions (optional)
def display_joystick_positions():
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    #joystick_positions = f"{int(controller_1.axis3.position())} {int(controller_1.axis2.position())}"
    #brain.screen.print(joystick_positions)
    wait(0.1, SECONDS)

def scale_joystick_input(input_value):
    # Normalize the input to the range [-1, 1]
    normalized_input = input_value / 100.0
    # Apply cubic scaling
    scaled_input = normalized_input ** 3
    # Scale back to the range [-100, 100]
    if slow_drive:
        return scaled_input * 90
    else:
        return scaled_input * 100

# Function to set drive motor velocities based on controller input
def set_drive_motor_velocities():
    global slow_drive
    if controller_1.buttonA.pressing():
        slow_drive = not slow_drive
        while controller_1.buttonA.pressing():
            wait(10, MSEC)

    # Normal control
    left_joystick_y = controller_1.axis3.position()
    right_joystick_y = controller_1.axis2.position()

    # Apply scaling to joystick inputs
    left_joystick_y = scale_joystick_input(left_joystick_y)
    right_joystick_y = scale_joystick_input(right_joystick_y)

    # Set velocities for left and right drive motors
    left_drive_smart.set_velocity(left_joystick_y, PERCENT)
    if abs(left_joystick_y) < 5:
        left_drive_smart.stop()
    else:
        left_drive_smart.spin(FORWARD)

    right_drive_smart.set_velocity(right_joystick_y, PERCENT)
    if abs(right_joystick_y) < 5:
        right_drive_smart.stop()
    else:
        right_drive_smart.spin(FORWARD)
       
# Function to toggle the high scoring motor
def toggle_high_scoring_motor():
    global high_scoring_running, high_score_target_angle
    if controller_1.buttonLeft.pressing():
        set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_SCORE)
        high_scoring_running = False
        while controller_1.buttonLeft.pressing():
            wait(10, MSEC)

    if controller_1.buttonUp.pressing():
        set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_WAIT)
        high_scoring_running = False
        while controller_1.buttonLeft.pressing():
            wait(10, MSEC)

    if controller_1.buttonRight.pressing():
        set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_CAPTURE)
        high_scoring_running = True
        while controller_1.buttonLeft.pressing():
            wait(10, MSEC)

    if controller_1.buttonDown.pressing():
        set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_DOWN)
        high_scoring_running = False
        while controller_1.buttonDown.pressing():
            wait(10, MSEC)

# Function to toggle the intake motor
def toggle_intake_motor():
    global intake_state
    global consecutive_stall_count, retry_count, high_score_stall
    global intake_running

    if controller_1.buttonR1.pressing():

        intake_state = IntakeState.RUNNING if intake_state == IntakeState.STOPPED else IntakeState.STOPPED
        consecutive_stall_count = 0
        retry_count = 0

        set_intake_motor_state(FORWARD)
        wait(100, MSEC)  # Debounce delay
        while controller_1.buttonR1.pressing():
            wait(100, MSEC)

    if controller_1.buttonR2.pressing():
        intake_state = IntakeState.RUNNING if intake_state == IntakeState.STOPPED else IntakeState.STOPPED
        consecutive_stall_count = 0
        retry_count = 0
        set_intake_motor_state(REVERSE)
        wait(100, MSEC)  # Debounce delay
        while controller_1.buttonR2.pressing():
            wait(100, MSEC)

# Function to handle digital outputs based on controller buttons
def handle_digital_outputs():
    if controller_1.buttonL1.pressing():
        print("Mogo 1")
        mogo_p.set(False)
    if controller_1.buttonL2.pressing():
        print("Mogo 2")
        mogo_p.set(True)
    if controller_1.buttonX.pressing():
        intake_p.set(not intake_p.value())
    if controller_1.buttonY.pressing():
        donker.set(True)
    if controller_1.buttonB.pressing():
        donker.set(False)

# Autonomous function
def autonomous():
    global eject_object, gyro
    # Autonomous code
    # For example, move forward for a certain distance
    # define a variable slot_no and switch case based on the slot_no
    # to run the corresponding autonomous routine
    #wait(3, SECONDS)
    slot_no = 5
    if slot_no == 1:
        gyro.set_heading(180, DEGREES)
        eject_object = RingType.BLUE
        autonomous_red_left()
    elif slot_no == 2:
        gyro.set_heading(180, DEGREES)
        eject_object = RingType.BLUE
        autonomous_red_right()
    elif slot_no == 3:
        eject_object = RingType.NONE
        autonomous_blue_left()
    elif slot_no == 4:
        eject_object = RingType.RED
        autonomous_blue_right()
    elif slot_no == 5:
        eject_object = RingType.NONE
        autonomous_test()

    eject_object = RingType.NONE
    ejection_p.set(False)
    left_drive_smart.stop()
    right_drive_smart.stop()

# Driver control function
def drivercontrol():
    # Main control loop for driver control
    global eject_object
    eject_object = RingType.BLUE

    while True:
        set_drive_motor_velocities()
        toggle_high_scoring_motor()
        adjust_high_scoring_motor_position()
        toggle_intake_motor()
        check_vision_sensor()
        handle_digital_outputs()
        stall_detection_and_handling()

        wait(20, MSEC)

def stopIntake():
    wait(1.3, SECONDS)
    intake_lower.set_velocity(70, PERCENT)
    intake_upper.stop()


def autonomous_empty():
    left_drive_smart.set_velocity(95, PERCENT)
    right_drive_smart.set_velocity(95, PERCENT)
    left_drive_smart.spin(FORWARD)
    right_drive_smart.spin(FORWARD)
    wait(200, MSEC)
    left_drive_smart.stop()
    right_drive_smart.stop()


def thread_fn():
    global lookahead, tolerance, increasing_x, decreasing_x, test_square, intake_state, high_score_target_angle, test_circle, gyro, eject_object, forward_velocity, turn_velocity_k, position_debug
    # Reverse the test_circle path
    #reversed_test_square = test_square[::-1]
    #walk_path(test_square, lookahead, tolerance, 1)
    #walk_path(reversed_test_square, lookahead, tolerance, 1)
    decreasing_x = increasing_x[::-1]
    #walk_path(decreasing_x, lookahead, tolerance, -1)
    walk_path(increasing_x, lookahead, tolerance, 1)

def autonomous_test2():
    thr = Thread(thread_fn)

p1_56 = [(-145.0, 0.0), (-143.0, 0.0), (-141.0, 0.0), (-139.0, 0.0), (-137.0, 0.0), (-135.0, 0.0), (-133.0, 0.0), (-131.0, 0.0), (-129.0, 0.0), (-127.0, 0.0), (-125.0, 0.0), (-123.0, 0.0), (-121.0, 0.0), (-120.0, 0.0), (-120.0, 0.0)]

p2_56 = [(-120.0, 0.0), (-120.0, 2.0), (-120.0, 4.0), (-120.0, 6.0), (-120.0, 8.0), (-120.0, 10.0), (-120.0, 12.0), (-120.0, 14.0), (-120.0, 16.0), (-120.0, 18.0), (-120.0, 20.0), (-120.0, 22.0), (-120.0, 24.0), (-120.0, 26.0), (-120.0, 28.0), (-120.0, 30.0), (-120.0, 32.0), (-120.0, 34.0), (-120.0, 36.0), (-120.0, 38.0), (-120.0, 40.0), (-120.0, 42.0), (-120.0, 44.0), (-120.0, 46.0), (-120.0, 48.0), (-120.0, 50.0), (-120.0, 50.0)]
p3_56 = [(-120.0, 50.0), (-118.027, 50.329), (-116.054, 50.658), (-114.082, 50.986), (-112.109, 51.315), (-110.136, 51.644), (-108.163, 51.973), (-106.19, 52.302), (-104.218, 52.63), (-102.245, 52.959), (-100.272, 53.288), (-98.299, 53.617), (-96.327, 53.946), (-94.354, 54.274), (-92.381, 54.603), (-90.408, 54.932), (-88.435, 55.261), (-86.463, 55.59), (-84.49, 55.918), (-82.517, 56.247), (-80.544, 56.576), (-78.571, 56.905), (-76.599, 57.234), (-74.626, 57.562), (-72.653, 57.891), (-70.68, 58.22), (-68.708, 58.549), (-66.735, 58.878), (-64.762, 59.206), (-62.789, 59.535), (-60.816, 59.864), (-58.996, 60.606), (-57.285, 61.641), (-55.574, 62.676), (-53.862, 63.711), (-52.151, 64.747), (-50.44, 65.782), (-48.729, 66.818), (-47.018, 67.853), (-45.306, 68.887), (-43.594, 69.92), (-41.88, 70.952), (-40.166, 71.983), (-38.451, 73.011), (-36.734, 74.037), (-35.016, 75.06), (-33.295, 76.08), (-31.573, 77.097), (-29.848, 78.109), (-28.121, 79.117), (-26.39, 80.12), (-24.657, 81.118), (-22.92, 82.109), (-21.18, 83.095), (-19.436, 84.074), (-17.688, 85.045), (-15.935, 86.01), (-14.179, 86.966), (-12.418, 87.915), (-10.653, 88.856), (-8.884, 89.788), (-7.11, 90.711), (-5.332, 91.627), (-3.549, 92.533), (-1.762, 93.431), (0.029, 94.321), (1.824, 95.203), (3.624, 96.076), (5.427, 96.941), (7.234, 97.799), (9.044, 98.648), (10.858, 99.491), (12.675, 100.326), (14.496, 101.155), (16.319, 101.976), (18.145, 102.792), (19.974, 103.601), (21.806, 104.405), (23.64, 105.202), (25.476, 105.995), (27.315, 106.782), (29.155, 107.564), (30.998, 108.342), (32.842, 109.116), (34.688, 109.885), (36.536, 110.649), (38.386, 111.41), (40.237, 112.168), (42.089, 112.922), (43.943, 113.673), (45.798, 114.42), (47.655, 115.164), (49.512, 115.905), (51.371, 116.644), (53.231, 117.38), (55.091, 118.113), (56.953, 118.844), (59.636, 119.894), (59.636, 119.894)]
p4_56 = [(59.636, 119.894), (57.636, 119.894), (55.636, 119.894), (53.636, 119.894), (51.636, 119.894), (49.636, 119.894), (47.636, 119.894), (45.636, 119.894), (43.636, 119.894), (41.636, 119.894), (39.636, 119.894), (37.636, 119.894), (35.636, 119.894), (33.636, 119.894), (31.636, 119.894), (29.636, 119.894), (27.636, 119.894), (25.636, 119.894), (23.636, 119.894), (21.636, 119.894), (19.636, 119.894), (17.636, 119.894), (15.636, 119.894), (13.636, 119.894), (11.636, 119.894), (9.636, 119.894), (7.636, 119.894), (5.636, 119.894), (3.636, 119.894), (1.636, 119.894), (0.0, 119.894), (0.0, 119.894)]
p5_56 = [(0.0, 119.273), (0.0, 121.273), (0.0, 123.273), (0.0, 125.273), (0.0, 127.273), (0.0, 129.273), (0.0, 131.273), (0.0, 133.273), (0.0, 135.273), (0.0, 137.273), (0.0, 139.273), (0.0, 140.0), (0.0, 140.0)]
p6_56 = [(0.0, 119.273), (0.0, 123.273), (0.0, 127.273), (0.0, 131.273), (0.0, 135.0), (0.0, 135.0)]
p7_56 = p6_56[::-1]
p8_56 = [(0.0, 119.273), (-2.0, 119.273), (-4.0, 119.273), (-6.0, 119.273), (-8.0, 119.273), (-10.0, 119.273), (-12.0, 119.273), (-14.0, 119.273), (-16.0, 119.273), (-18.0, 119.273), (-20.0, 119.273), (-22.0, 119.273), (-24.0, 119.273), (-26.0, 119.273), (-28.0, 119.273), (-30.0, 119.273), (-32.0, 119.273), (-34.0, 119.273), (-36.0, 119.273), (-38.0, 119.273), (-40.0, 119.273), (-42.0, 119.273), (-44.0, 119.273), (-46.0, 119.273), (-48.0, 119.273), (-50.0, 119.273), (-52.0, 119.273), (-54.0, 119.273), (-56.0, 119.273), (-58.0, 119.273), (-60.0, 119.273), (-62.0, 119.273), (-64.0, 119.273), (-66.0, 119.273), (-68.0, 119.273), (-70.0, 119.273), (-72.0, 119.273), (-74.0, 119.273), (-76.0, 119.273), (-78.0, 119.273), (-80.0, 119.273), (-82.0, 119.273), (-84.0, 119.273), (-86.0, 119.273), (-88.0, 119.273), (-90.0, 119.273), (-92.0, 119.273), (-94.0, 119.273), (-96.0, 119.273), (-98.0, 119.273), (-100.0, 119.273), (-102.0, 119.273), (-104.0, 119.273), (-106.0, 119.273), (-108.0, 119.273), (-110.0, 119.273), (-112.0, 119.273), (-114.0, 119.273), (-116.0, 119.273), (-118.0, 119.273), (-120.0, 119.273), (-122.0, 119.273), (-124.0, 119.273), (-126.0, 119.273), (-128.0, 119.273), (-130.0, 119.273), (-130.0, 119.273)]
p9_56 = [(-148.0, 119.273), (-146.618, 120.719), (-145.236, 122.164), (-143.854, 123.61), (-142.471, 125.055), (-141.089, 126.501), (-139.707, 127.946), (-138.325, 129.392), (-136.943, 130.838), (-135.561, 132.283), (-134.179, 133.729), (-132.797, 135.174), (-131.414, 136.62), (-130.032, 138.066), (-128.65, 139.511), (-127.268, 140.957), (-125.886, 142.402), (-124.504, 143.848), (-124.242, 144.121), (-124.242, 144.121)]
p10_56 = [(-124.242, 144.121), (-126.173, 144.643), (-128.104, 145.165), (-130.035, 145.687), (-131.965, 146.208), (-133.896, 146.73), (-135.827, 147.252), (-137.758, 147.774), (-139.688, 148.296), (-141.619, 148.818), (-143.55, 149.339), (-145.48, 149.861), (-147.227, 150.333), (-147.227, 150.333)]
p11_56 = [(-145.364, 152.197), (-143.833, 150.911), (-142.228, 149.717), (-140.579, 148.586), (-138.916, 147.475), (-137.275, 146.332), (-135.694, 145.107), (-134.207, 143.77), (-132.841, 142.311), (-131.608, 140.737), (-130.505, 139.069), (-129.526, 137.325), (-128.655, 135.525), (-127.878, 133.683), (-127.183, 131.808), (-126.556, 129.908), (-125.989, 127.991), (-125.474, 126.058), (-125.004, 124.114), (-124.574, 122.161), (-124.177, 120.201), (-123.813, 118.234), (-123.475, 116.263), (-123.163, 114.288), (-122.872, 112.309), (-122.602, 110.327), (-122.35, 108.343), (-122.115, 106.357), (-121.896, 104.369), (-121.691, 102.38), (-121.499, 100.389), (-121.319, 98.397), (-121.151, 96.404), (-120.993, 94.41), (-120.845, 92.416), (-120.707, 90.421), (-120.577, 88.425), (-120.456, 86.428), (-120.342, 84.432), (-120.236, 82.434), (-120.136, 80.437), (-120.043, 78.439), (-119.956, 76.441), (-119.874, 74.443), (-119.8, 72.444), (-119.73, 70.445), (-119.665, 68.446), (-119.605, 66.447), (-119.55, 64.448), (-119.5, 62.449), (-119.453, 60.449), (-119.41, 58.45), (-119.372, 56.45), (-119.338, 54.45), (-119.307, 52.451), (-119.273, 50.0), (-119.273, 50.0)]
p12_56 = [(-119.273, 50.0), (-119.285, 48.0), (-119.297, 46.0), (-119.309, 44.0), (-119.321, 42.0), (-119.333, 40.0), (-119.345, 38.0), (-119.357, 36.0), (-119.369, 34.0), (-119.381, 32.0), (-119.393, 30.0), (-119.405, 28.0), (-119.417, 26.0), (-119.429, 24.0), (-119.441, 22.001), (-119.453, 20.001), (-119.465, 18.001), (-119.477, 16.001), (-119.489, 14.001), (-119.501, 12.001), (-119.513, 10.001), (-119.525, 8.001), (-119.537, 6.001), (-119.549, 4.001), (-119.561, 2.001), (-119.573, 0.001), (-119.585, -1.999), (-119.597, -3.999), (-119.609, -5.999), (-119.621, -7.999), (-119.633, -9.999), (-119.646, -11.999), (-119.658, -13.999), (-119.67, -15.999), (-119.682, -17.999), (-119.694, -19.999), (-119.706, -21.999), (-119.718, -23.999), (-119.73, -25.999), (-119.742, -27.999), (-119.754, -29.999), (-119.766, -31.999), (-119.778, -33.998), (-119.79, -35.998), (-119.802, -37.998), (-119.814, -39.998), (-119.826, -41.998), (-119.838, -43.998), (-119.85, -45.998), (-119.862, -47.998), (-119.874, -49.998), (-119.886, -51.998), (-119.898, -53.998), (-119.91, -55.998), (-119.922, -57.998), (-119.934, -59.998), (-119.946, -61.998), (-119.958, -63.998), (-119.97, -65.998), (-119.982, -67.998), (-120.0, -71.0), (-120.0, -71.0)]



def pidTurnToAngle(robot_angle, p = 0.45, motor_stop = 1, error_stop = 1.0):
    original = robot_angle
    global gyro, left_drive_smart, right_drive_smart
   
    #error = robot_angle - gyro.heading(DEGREES)
    error = (robot_angle - gyro.rotation(DEGREES) + 180) % 360 - 180  
    print((gyro.rotation(DEGREES), robot_angle, error))
    left_drive_smart.set_velocity(p * error, PERCENT)
    right_drive_smart.set_velocity(-p * error, PERCENT)
    left_drive_smart.spin(FORWARD)
    right_drive_smart.spin(FORWARD)
   
    while abs(error) > error_stop:
        update_position()
        #error = robot_angle - gyro.rotation(DEGREES)
        error = (robot_angle - gyro.rotation(DEGREES) + 180) % 360 - 180
        if (abs(error) < 10): p = 0.7
        if (abs(error) < 5): p = 0.9
        if (abs(error) < 2): p = 1.2
        left_drive_smart.set_velocity(p * error, PERCENT)
        right_drive_smart.set_velocity(-p * error, PERCENT)
        if (abs(error) > 10):
            wait(200, MSEC)
        else:
            wait(100, MSEC)
        if (abs(p * error) < motor_stop):
            break      
       

    left_drive_smart.stop()
    right_drive_smart.stop()

alliance_stake = [(-159.385, 0.0), (-157.385, 0.0), (-155.385, 0.0), (-153.385, 0.0), (-151.385, 0.0), (-149.385, 0.0), (-147.385, 0.0), (-145.385, 0.0), (-143.385, 0.0), (-141.385, 0.0), (-139.385, 0.0), (-137.385, 0.0), (-135.385, 0.0), (-133.385, 0.0), (-131.385, 0.0), (-120.483, 0), (-110, 0), (-100, 0)]
#alliance stake changed because it was bumping into the hanging wall and we made it shorter
grabbing_mogo = [(-120.483, 5.068), (-120.451, 7.067), (-120.419, 9.067), (-120.387, 11.067), (-120.355, 13.067), (-120.323, 15.066), (-120.291, 17.066), (-120.259, 19.066), (-120.227, 21.066), (-120.195, 23.065), (-120.163, 25.065), (-120.131, 27.065), (-120.099, 29.065), (-120.067, 31.064), (-120.035, 33.064), (-120.003, 35.064), (-119.971, 37.064), (-119.939, 39.063), (-119.907, 41.063), (-119.875, 43.063), (-119.843, 45.063), (-119.811, 47.062), (-119.779, 49.062), (-119.747, 51.062), (-119.715, 53.062), (-119.683, 55.061), (-119.651, 57.061), (-119.619, 59.061), (-119.587, 61.061), (-119.555, 63.06), (-119.523, 65.06), (-119.491, 67.06), (-119.468, 68.533), (-119.468, 68.533)]


collect_left_bottom_rings =  [(-119.468, 68.533), (-119.777, 66.557), (-119.992, 64.569), (-120.111, 62.573), (-120.13, 60.573), (-120.045, 58.575), (-119.858, 56.584), (-119.563, 54.606), (-119.161, 52.647), (-118.649, 50.715), (-118.028, 48.814), (-117.292, 46.955), (-116.445, 45.143), (-115.484, 43.39), (-114.407, 41.705), (-113.218, 40.097), (-111.916, 38.58), (-110.504, 37.165), (-108.984, 35.866), (-107.361, 34.697), (-105.643, 33.674), (-103.84, 32.81), (-101.965, 32.118), (-100.031, 31.609), (-98.058, 31.289), (-96.063, 31.162), (-94.064, 31.228), (-92.081, 31.479), (-90.127, 31.904), (-88.216, 32.489), (-86.358, 33.228), (-84.557, 34.097), (-82.821, 35.089), (-81.149, 36.186), (-79.541, 37.376), (-77.999, 38.649), (-76.521, 39.995), (-75.104, 41.407), (-73.746, 42.875), (-72.445, 44.393), (-71.177, 45.94), (-69.912, 47.49), (-68.648, 49.039), (-67.383, 50.588), (-66.118, 52.138), (-64.853, 53.687), (-63.589, 55.236), (-62.324, 56.786), (-61.059, 58.335), (-59.794, 59.884), (-58.53, 61.434), (-57.265, 62.983), (-56.0, 64.532), (-54.735, 66.082), (-53.471, 67.631), (-52.206, 69.18), (-50.941, 70.73), (-49.676, 72.279), (-48.412, 73.828), (-47.147, 75.377), (-45.882, 76.927), (-44.644, 78.494), (-43.626, 80.216), (-42.579, 81.92), (-41.502, 83.606), (-40.394, 85.27), (-39.255, 86.914), (-38.082, 88.534), (-36.875, 90.129), (-35.635, 91.698), (-34.359, 93.238), (-33.047, 94.748), (-31.698, 96.224), (-30.311, 97.665), (-28.888, 99.07), (-27.426, 100.435), (-25.927, 101.759), (-24.39, 103.038), (-22.815, 104.271), (-21.204, 105.455), (-19.557, 106.59), (-17.875, 107.673), (-16.159, 108.7), (-14.412, 109.673), (-12.635, 110.59), (-10.829, 111.449), (-8.998, 112.252), (-7.142, 112.999), (-5.265, 113.688), (-3.369, 114.324), (-1.455, 114.905), (0.473, 115.435), (2.414, 115.917), (4.366, 116.353), (6.327, 116.746), (8.295, 117.101), (10.269, 117.422), (12.248, 117.714), (14.23, 117.982), (16.214, 118.232), (18.2, 118.471), (20.186, 118.707), (22.171, 118.947), (24.155, 119.202), (26.14, 119.448), (28.124, 119.7), (30.108, 119.951), (32.093, 120.194), (34.081, 120.419), (36.071, 120.621), (38.063, 120.791), (40.059, 120.924), (42.057, 121.018), (44.056, 121.068), (46.056, 121.074), (47.671, 121.047), (47.671, 121.047)]

robot_to_left_corner = [(47.671, 121.047), (46.462, 122.641), (45.225, 124.212), (43.959, 125.76), (42.664, 127.284), (41.341, 128.784), (39.988, 130.257), (38.606, 131.702), (37.195, 133.119), (35.754, 134.506), (34.283, 135.862), (32.782, 137.183), (31.251, 138.47), (29.689, 139.719), (28.096, 140.928), (26.472, 142.095), (24.817, 143.218), (23.13, 144.292), (21.412, 145.316), (19.662, 146.285), (17.882, 147.195), (16.071, 148.044), (14.23, 148.825), (12.361, 149.536), (10.464, 150.17), (8.542, 150.722), (6.597, 151.19), (4.633, 151.566), (2.653, 151.845), (0.661, 152.024), (-1.334, 151.92), (-3.316, 151.659), (-5.27, 151.235), (-7.173, 150.624), (-8.998, 149.807), (-10.719, 148.79), (-12.335, 147.613), (-13.869, 146.33), (-15.362, 145.0), (-16.857, 143.671), (-18.388, 142.384), (-19.974, 141.166), (-21.624, 140.036), (-23.337, 139.004), (-25.105, 138.07), (-26.921, 137.233), (-28.775, 136.484), (-30.661, 135.82), (-32.572, 135.23), (-34.503, 134.708), (-36.449, 134.248), (-38.407, 133.842), (-40.375, 133.485), (-42.351, 133.172), (-44.332, 132.901), (-46.318, 132.667), (-48.308, 132.464), (-50.301, 132.294), (-52.295, 132.15), (-54.292, 132.035), (-56.29, 131.941), (-58.289, 131.869), (-60.288, 131.82), (-62.288, 131.789), (-64.288, 131.775), (-66.288, 131.778), (-68.288, 131.797), (-70.287, 131.83), (-72.287, 131.878), (-74.287, 131.882), (-76.287, 131.882), (-78.287, 131.882), (-80.287, 131.882), (-82.287, 131.882), (-84.287, 131.882), (-86.287, 131.882), (-88.287, 131.882), (-90.287, 131.882), (-92.287, 131.882), (-94.287, 131.882), (-96.287, 131.882), (-98.287, 131.882), (-100.287, 131.882), (-102.287, 131.882), (-104.287, 131.882), (-105.436, 131.882), (-105.436, 131.882)]

randomOdomTest = [(-152.4, -76.2), (-150.4, -76.2), (-148.4, -76.2), (-146.4, -76.2), (-144.4, -76.2), (-142.4, -76.2), (-140.4, -76.2), (-138.4, -76.2), (-136.4, -76.2), (-134.4, -76.2), (-132.4, -76.2), (-130.4, -76.2), (-128.4, -76.2), (-126.4, -76.2), (-124.4, -76.2), (-122.4, -76.2), (-120.4, -76.2), (-118.4, -76.2), (-116.4, -76.2), (-114.4, -76.2), (-112.4, -76.2), (-110.4, -76.2), (-108.4, -76.2), (-106.4, -76.2), (-104.4, -76.2), (-102.4, -76.2), (-100.4, -76.2), (-98.4, -76.2), (-96.4, -76.2), (-94.4, -76.2), (-92.4, -76.2), (-90.4, -76.2), (-88.4, -76.2), (-86.4, -76.2), (-84.4, -76.2), (-82.4, -76.2), (-80.4, -76.2), (-78.4, -76.2), (-76.2, -76.2), (-76.2, -76.2)]

def autonomous_test():
    global lookahead, tolerance, increasing_x, test_square, intake_state, high_score_target_angle, test_circle, gyro, eject_object, forward_velocity, turn_velocity_k
    #walk_path(increasing_x, lookahead, tolerance, 1)
   
    #mogo_p.set(True)
    #wait(1, SECONDS)
    #high_score_target_angle = HIGH_SCORE_TARGET_ANGLE_WAIT
    #adjust_high_scoring_motor_position()
    #intake_state = IntakeState.RUNNING  
    #set_intake_motor_state(REVERSE)
    #gyro.set_heading(180, DEGREES)
    # Reverse the test_circle path
    # reversed_test_circle = test_circle[::-1]


    #walk_path(reversed_test_circle, lookahead, tolerance, 1)
    lookahead = 50
    forward_velocity = 45
    forward_velocity = 45
    turn_velocity_k = 40
    wall_score_on = False

    
    print(vertical_rotational_sensor.position(DEGREES))
    set_high_score_angle(HIGH_SCORE_TARGET_ANGLE_WAIT)
    adjust_high_scoring_motor_position()
    intake_state = IntakeState.RUNNING
    wait(100, MSEC)
    set_intake_motor_state(REVERSE)
    wait(1000, MSEC)


    walk_path(randomOdomTest, lookahead, tolerance, 1, True)
    print("DONE")
    """
    print("alliance done with turning")
    walk_path(grabbing_mogo, lookahead, tolerance, -1)
    print("grabbing done with turning")
    mogo_p.set(True)
    wait(50, MSEC)


    # Left bottom corner
    forward_velocity = 50
    turn_velocity_k = 45
    intake_upper.set_velocity(100, PERCENT)
    walk_path(collect_left_bottom_rings, lookahead , tolerance, 1)
    wait(300, MSEC)
    print("done 8")
    forward_velocity = 45
    turn_velocity_k = 45

    #brings mogo back to left corner
    walk_path(robot_to_left_corner, lookahead, tolerance, 1)
    print("done 9")"""


def main():
    # Any initialization code before the match starts
    print("Running main.py")
    wait(3, SECONDS)
    #mogo_p.set(False)
    #intake_p.set(True)
    #autonomous()
    autonomous_test()
    #drivercontrol()
    #autonomous()
    #intake_p.set(True)
    #drive
    #unscoring()
main()