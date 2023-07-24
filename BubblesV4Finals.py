import RPi.GPIO as GPIO
import adafruit_tcs34725
import board
import time
import RPi.GPIO as GPIO
import busio

# Define GPIO pins
ir1_pin = 17
ir2_pin = 18
ir3_pin = 15
ircounter_pin = 14
ir1back_pin = 10
ir2back_pin = 0
ir3back_pin = 9

ena_pin = 13
in1_pin = 23
in2_pin = 24

enb_pin = 12
in3_pin = 27
in4_pin = 22

# Ultrasonic sensor GPIO p
SENSOR_PIN = 5
GPIO_ECHO = 6

# GPIO pins for servos
SERVO1_PIN = 20
SERVO2_PIN = 21
SERVO3_PIN = 26
SERVO4_PIN = 16

# ~ counter_increment_delay = 0.0005
# ~ counter = 0

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pins as inputs and outputs

GPIO.setup(SENSOR_PIN, GPIO.IN)
GPIO.setup(ir1_pin, GPIO.IN)
GPIO.setup(ir2_pin, GPIO.IN)
GPIO.setup(ir3_pin, GPIO.IN)
GPIO.setup(ircounter_pin, GPIO.IN)
GPIO.setup(ir1back_pin, GPIO.IN)
GPIO.setup(ir2back_pin, GPIO.IN)
GPIO.setup(ir3back_pin, GPIO.IN)

GPIO.setup(ena_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

GPIO.setup(enb_pin, GPIO.OUT)
GPIO.setup(in3_pin, GPIO.OUT)
GPIO.setup(in4_pin, GPIO.OUT)

GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)
GPIO.setup(SERVO3_PIN, GPIO.OUT)
GPIO.setup(SERVO4_PIN, GPIO.OUT)

# Create PWM objects
pwm_a = GPIO.PWM(ena_pin, 100)
pwm_b = GPIO.PWM(enb_pin, 100)
servo1 = GPIO.PWM(SERVO1_PIN, 50)
servo2 = GPIO.PWM(SERVO2_PIN, 50)
servo3 = GPIO.PWM(SERVO3_PIN, 50)
servo4 = GPIO.PWM(SERVO4_PIN, 50)

# Start PWM
pwm_a.start(0)
pwm_b.start(0)
servo1.start(0)
servo2.start(0)
servo3.start(0)
servo4.start(0)

# Initialize I2C bus
# ~ i2c = busio.I2C(board.SCL, board.SDA)

# Initialize TCS34725 sensor
# ~ sensor = adafruit_tcs34725.TCS34725(i2c)

def slow_forward():
    #print("forward")
    GPIO.output(in1_pin, GPIO.HIGH)
    GPIO.output(in2_pin, GPIO.LOW)
    GPIO.output(in3_pin, GPIO.HIGH)
    GPIO.output(in4_pin, GPIO.LOW)

def forward():
    #print("forward")
    GPIO.output(in1_pin, GPIO.HIGH)
    GPIO.output(in2_pin, GPIO.LOW)
    GPIO.output(in3_pin, GPIO.HIGH)
    GPIO.output(in4_pin, GPIO.LOW)
    
def move_backward():
    GPIO.output(in1_pin, GPIO.LOW)
    GPIO.output(in2_pin, GPIO.HIGH)
    GPIO.output(in3_pin, GPIO.LOW)
    GPIO.output(in4_pin, GPIO.HIGH)
    
    
def right():
    #print("right")
    GPIO.output(in1_pin, GPIO.LOW)
    GPIO.output(in2_pin, GPIO.HIGH)
    GPIO.output(in3_pin, GPIO.HIGH)
    GPIO.output(in4_pin, GPIO.LOW)

    
def left():
    GPIO.output(in1_pin, GPIO.HIGH)
    GPIO.output(in2_pin, GPIO.LOW)
    GPIO.output(in3_pin, GPIO.LOW)
    GPIO.output(in4_pin, GPIO.HIGH)
    
def stop_motors():
    GPIO.output(in1_pin, GPIO.LOW)
    GPIO.output(in2_pin, GPIO.LOW)
    GPIO.output(in3_pin, GPIO.LOW)
    GPIO.output(in4_pin, GPIO.LOW)
    
def set_angle(servo, angle):
    duty = angle / 18 + 2  # Map angle to duty cycle (2 to 12)
    servo.ChangeDutyCycle(duty)

def open_servos():
    # Set angles to open the servos
    set_angle(servo1, 120)  # Set servo 1 to 0 degrees
    set_angle(servo2, 90)  # Set servo 2 to 180 degrees
    
def closemidleft_servos():
    set_angle(servo1, 120)  # Set servo 1 to 0 degrees
    set_angle(servo2, 150)  # Set servo 2 to 180 degrees
    
def closemidright_servos():
    set_angle(servo1, 40)  # Set servo 1 to 0 degrees
    set_angle(servo2, 150)  # Set servo 2 to 180 deg

def close_servos():
    # Set angles to close the servos
    set_angle(servo1, 30)  # Set servo 1 to 90 degrees
    set_angle(servo2, 180)  # Set servo 2 to 90 degrees
    
def rise_servos():
    # Set angles to close the servos
    set_angle(servo3, -10)  # Set servo 1 to 90 degrees
    #time.sleep(5)
    
def fall_servo():
    set_angle(servo3, 76.7)
    time.sleep(1)
    
def rear_servo():
    print("opening servo")
    set_angle(servo4, 90)
    time.sleep(3)
    print("open servo")
    set_angle(servo4, 0)
    time.sleep(5)
    print("close servo")
    set_angle(servo4, 85)
    time.sleep(2)
    
def checkpoint_one():
    print("checkpoint one")
    forward()
    pwm_a.ChangeDutyCycle(100)
    pwm_b.ChangeDutyCycle(100)
    time.sleep(0.2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    right()
    pwm_a.ChangeDutyCycle(100)
    pwm_b.ChangeDutyCycle(100)
    time.sleep(0.2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    forward()
    pwm_a.ChangeDutyCycle(70)
    pwm_b.ChangeDutyCycle(70)
    time.sleep(0.2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    forward()
    pwm_a.ChangeDutyCycle(70)
    pwm_b.ChangeDutyCycle(70)
    time.sleep(0.2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    move_backward()
    pwm_a.ChangeDutyCycle(40)
    pwm_b.ChangeDutyCycle(40)
    time.sleep(0.7)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    left()
    pwm_a.ChangeDutyCycle(40)
    pwm_b.ChangeDutyCycle(40)
    time.sleep(0.5)
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    left()
    pwm_a.ChangeDutyCycle(40)
    pwm_b.ChangeDutyCycle(40)
    time.sleep(0.3)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    forward()
    pwm_a.ChangeDutyCycle(30)
    pwm_b.ChangeDutyCycle(30)
    time.sleep(0.2)
    forward()
    pwm_a.ChangeDutyCycle(30)
    pwm_b.ChangeDutyCycle(30)
    time.sleep(0.2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    move_backward()
    pwm_a.ChangeDutyCycle(40)
    pwm_b.ChangeDutyCycle(40)
    time.sleep(0.2)
        
def checkpoint_two():
    print("checkpoint two")
    forward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.7)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    right()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.5)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    forward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.7)
    forward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.7)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(1)
    move_backward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.14)
    left()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.5)
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    left()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.5)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    forward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.7)
    forward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.7)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    move_backward()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.14)

    
def dispense_ball_red():
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(1)
    set_angle(servo4, 0)
    time.sleep(5)
    set_angle(servo4, 90)
    time.sleep(1)
    forward()
    pwm_a.ChangeDutyCycle(30)
    pwm_b.ChangeDutyCycle(30)
    time.sleep(0.2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    # ~ right()
    # ~ pwm_a.ChangeDutyCycle(30)
    # ~ pwm_b.ChangeDutyCycle(30)
    # ~ time.sleep(0.1)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    line_following_reset()
    time.sleep(1)

def dispense_ball_green():
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(1)
    set_angle(servo4, 0)
    time.sleep(5)
    set_angle(servo4, 90)
    time.sleep(1)
    # ~ forward()
    # ~ pwm_a.ChangeDutyCycle(50)
    # ~ pwm_b.ChangeDutyCycle(50)
    # ~ time.sleep(0.2)
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(2)
    left()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.75)
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(2)
    line_following_reset()
    
def dispense_ball_blue():
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(1)
    set_angle(servo4, 0)
    time.sleep(5)
    set_angle(servo4, 90)
    time.sleep(1)
    # ~ forward()
    # ~ pwm_a.ChangeDutyCycle(50)
    # ~ pwm_b.ChangeDutyCycle(50)
    # ~ time.sleep(0.2)
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(2)
    left()
    pwm_a.ChangeDutyCycle(50)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(0.66)
    stop_motors()
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(2)
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0) 
    time.sleep(2)
    move_backward()
    pwm_a.ChangeDutyCycle(25)
    pwm_b.ChangeDutyCycle(25)
    time.sleep(2)
    
    
def gather_ball():
    while True:
        
        # ~ time.sleep(0.1)
        
        left_state = GPIO.input(ir1_pin)
        center_state = GPIO.input(ir2_pin)
        right_state = GPIO.input(ir3_pin)
        # ~ ir_counter = GPIO.input(ircounter_pin)
        if not GPIO.input(SENSOR_PIN):
            print("ASD")
            stop_motors
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            
            rise_servos()
            time.sleep(2)

            open_servos()
            time.sleep(2)  # Wait for 2 seconds

            fall_servo()
            time.sleep(2)

            open_servos()
            time.sleep(2)  # Wait for 2 seconds

            # ~ closemidright_servos()
            # ~ time.sleep(2)
# Close the servos 
            close_servos()
            time.sleep(2)  # Wait for 2 seconds

            rise_servos()
            time.sleep(2)
            
            # ~ line_following_reverse()
            color_detect()
            time.sleep(1)
            
        else:
            print("SD")
            #checkpoint_one()
            #time.sleep(1)
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(.1)
            #checkpoint_two()
            #time.sleep(1)
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(.1)
            

            
            
            
        # ~ else:  # Stop motors if object is detected within 20 cm
            # ~ if left_state == 1:
                # ~ #print("left")
                # ~ left()
                # ~ pwm_a.ChangeDutyCycle(50)
                # ~ pwm_b.ChangeDutyCycle(25)
            # ~ elif center_state == 1:
                # ~ #print("forward")
                # ~ forward()
                # ~ pwm_a.ChangeDutyCycle(17)
                # ~ pwm_b.ChangeDutyCycle(17)
                
            # ~ elif right_state == 1:
                # ~ #print("right")
                # ~ right()
                # ~ pwm_a.ChangeDutyCycle(25)
                # ~ pwm_b.ChangeDutyCycle(50)
                
            # ~ else:
                # ~ forward()
                # ~ pwm_a.ChangeDutyCycle(17)
                # ~ pwm_b.ChangeDutyCycle(17)
                
def dispense_red():
    counter = 0
    counter_increment_timer = time.time()
    counter_increment_delay = 0.5
    last_detected_time = 0.0
    line_count = 0
    line_detected = False
    cooldown_time = 2  #1.2 Time in seconds to ignore line detections after a detection
    last_detection_time = 0.0
    while True:
        leftback_state = GPIO.input(ir1back_pin)
        centerback_state = GPIO.input(ir2back_pin)
        rightback_state = GPIO.input(ir3back_pin)
        left_state = GPIO.input(ir1_pin)
        center_state = GPIO.input(ir2_pin)
        right_state = GPIO.input(ir3_pin)
        ir_counter = GPIO.input(ircounter_pin)
           
        if ((leftback_state and rightback_state)) and not line_detected and time.time() - last_detection_time > cooldown_time:
            line_detected = True
            last_detection_time = time.time()
            line_count+=1
            print(f"Line count: {line_count}")
        
            
        if not leftback_state and not rightback_state:
            line_detected = False 

        if line_count == 2:
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            move_backward()
            pwm_a.ChangeDutyCycle(30)
            pwm_b.ChangeDutyCycle(30)
            time.sleep(1)
            dispense_ball_red()  
        
        if counter < 10:
            if leftback_state and centerback_state and rightback_state:
                # ~ print("backward 2") #change
                move_backward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                time.sleep(.05)
            if leftback_state == 1:
                # ~ print("left i")
                left()
                pwm_a.ChangeDutyCycle(70)
                pwm_b.ChangeDutyCycle(50)
                #time.sleep(0.115)
            elif centerback_state == 1:
                # ~ print("backward 1")
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
            elif rightback_state == 1:
                # ~ print("right i")
                right()
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(70)
                #time.sleep(0.0862)
            elif leftback_state == 1 and centerback_state == 0 and rightback_state == 1:
                # ~ print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(70)  
                time.sleep(0.15)
            elif leftback_state == 1 and centerback_state == 1 and rightback_state == 0:
                # ~ print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(70)
                time.sleep(0.15)
            else:
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
            
            
def dispense_green():
    counter = 0
    counter_increment_timer = time.time()
    counter_increment_delay = 0.5
    last_detected_time = 0.0
    line_count = 0
    line_detected = False
    cooldown_time = 2.4  #1.2 Time in seconds to ignore line detections after a detection
    last_detection_time = 0.0
    while True:
        leftback_state = GPIO.input(ir1back_pin)
        centerback_state = GPIO.input(ir2back_pin)
        rightback_state = GPIO.input(ir3back_pin)
        left_state = GPIO.input(ir1_pin)
        center_state = GPIO.input(ir2_pin)
        right_state = GPIO.input(ir3_pin)
        ir_counter = GPIO.input(ircounter_pin)
           
        if ((leftback_state and rightback_state)) and not line_detected and time.time() - last_detection_time > cooldown_time:
            line_detected = True
            last_detection_time = time.time()
            line_count+=1
            print(f"Line count: {line_count}")
        
            
        if not leftback_state and not rightback_state:
            line_detected = False 

        if line_count == 3:
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            move_backward()
            pwm_a.ChangeDutyCycle(30)
            pwm_b.ChangeDutyCycle(30)
            time.sleep(0.5)
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            right()
            pwm_a.ChangeDutyCycle(100)
            pwm_b.ChangeDutyCycle(100)
            time.sleep(0.3)
            dispense_ball_green()  
        
        if counter < 10:
            if leftback_state and centerback_state and rightback_state:
                # ~ print("backward 2") #change
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
                time.sleep(.05)
            if leftback_state == 1:
                # ~ print("left i")
                left()
                pwm_a.ChangeDutyCycle(100)
                pwm_b.ChangeDutyCycle(50)
                #time.sleep(0.115)
            elif centerback_state == 1:
                # ~ print("backward 1")
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
            elif rightback_state == 1:
                # ~ print("right i")
                right()
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)
                #time.sleep(0.0862)
            elif leftback_state == 1 and centerback_state == 0 and rightback_state == 1:
                # ~ print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)  
                time.sleep(0.15)
            elif leftback_state == 1 and centerback_state == 1 and rightback_state == 0:
                # ~ print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)
                time.sleep(0.15)
            else:
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
            
def dispense_blue():
    counter = 0
    counter_increment_timer = time.time()
    counter_increment_delay = 0.5
    last_detected_time = 0.0
    line_count = 0
    line_detected = False
    cooldown_time = 2.4  #1.2 Time in seconds to ignore line detections after a detection
    last_detection_time = 0.0
    while True:
        leftback_state = GPIO.input(ir1back_pin)
        centerback_state = GPIO.input(ir2back_pin)
        rightback_state = GPIO.input(ir3back_pin)
        left_state = GPIO.input(ir1_pin)
        center_state = GPIO.input(ir2_pin)
        right_state = GPIO.input(ir3_pin)
        ir_counter = GPIO.input(ircounter_pin)
           
        if ((leftback_state and rightback_state)) and not line_detected and time.time() - last_detection_time > cooldown_time:
            line_detected = True
            last_detection_time = time.time()
            line_count+=1
            print(f"Line count: {line_count}")
        
            
        if not leftback_state and not rightback_state:
            line_detected = False 

        if line_count == 4:
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            move_backward()
            pwm_a.ChangeDutyCycle(30)
            pwm_b.ChangeDutyCycle(30)
            time.sleep(0.5)
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            right()
            pwm_a.ChangeDutyCycle(100)
            pwm_b.ChangeDutyCycle(100)
            time.sleep(0.38)
            dispense_ball_blue()  
        
        if counter < 10:
            if leftback_state and centerback_state and rightback_state:
                # ~ print("backward 2") #change
                move_backward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                time.sleep(.05)
            if leftback_state == 1:
                # ~ print("left i")
                left()
                pwm_a.ChangeDutyCycle(100)
                pwm_b.ChangeDutyCycle(50)
                #time.sleep(0.115)
            elif centerback_state == 1:
                # ~ print("backward 1")
                move_backward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
            elif rightback_state == 1:
                # ~ print("right i")
                right()
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)
                #time.sleep(0.0862)
            elif leftback_state == 1 and centerback_state == 0 and rightback_state == 1:
                # ~ print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)  
                time.sleep(0.15)
            elif leftback_state == 1 and centerback_state == 1 and rightback_state == 0:
                # ~ print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)
                time.sleep(0.15)
            else:
                move_backward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
            
def color_detect():
    red, green, blue = sensor.color_rgb_bytes

# Check color
    if red > blue and red > green:
        print("Color is red")
        dispense_red()
    elif blue > red and blue > green:
        print("Color is blue")
        dispense_blue()
    elif green > red and green > blue:
        print("Color is green")
        dispense_green()
    else:
        print("Color is not red or blue or green")
        color_detect()
        
        time.sleep(1)
# Line following algorithm
def line_following():
    counter = 0
    counter_increment_timer = time.time()
    counter_increment_delay = 5
    last_detected_time = 0.0
    line_detected = True
    kent = 0
    while True:
        left_state = GPIO.input(ir1_pin)
        center_state = GPIO.input(ir2_pin)
        right_state = GPIO.input(ir3_pin)
        ir_counter = GPIO.input(ircounter_pin)
        # ~ forward()
        # ~ pwm_a.ChangeDutyCycle(15)
        # ~ pwm_b.ChangeDutyCycle(25)
        # ~ time.sleep(0.3)
        #forward()
        #pwm_a.ChangeDutyCycle(100)
        #pwm_b.ChangeDutyCycle(100)
                    
        
        if counter < 10:
            if left_state == 1 and center_state == 1 and right_state == 1:
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
            if ir_counter == 1:
                counter += 1
                print("Counter:", counter)
                last_detected_time = time.time()
                time.sleep(0.3)
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
            # ~ if ir_counter == 0 and left_state == 1 and center_state == 1:
                # ~ counter += 1
                # ~ print("Counter:", counter)
                # ~ last_detected_time = time.time()
                # ~ time.sleep(0.3)
                # ~ forward()
                # ~ pwm_a.ChangeDutyCycle(20)
                # ~ pwm_b.ChangeDutyCycle(20)
            
            
            #if ir_counter == 0 and time.time() - last_detected_time > counter_increment_delay and not line_detected:
            #    print("Counter:", counter)
            #    counter += 1
            #    last_detected_time = time.time()
            #    time.sleep(0.1)
            #    line_detected = True 
                
            
            if left_state == 1:
                #print("left")
                left()
                pwm_a.ChangeDutyCycle(60)
                pwm_b.ChangeDutyCycle(30)
            elif center_state == 1:
                #print("forward")
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
            elif right_state == 1:
                #print("right")
                right()
                pwm_a.ChangeDutyCycle(30)
                pwm_b.ChangeDutyCycle(60)
                
            elif left_state == 1 and center_state == 0 and right_state == 1:
                #print("right")
                right() 
                pwm_a.ChangeDutyCycle(30)
                pwm_b.ChangeDutyCycle(60) 
                time.sleep(0.1)
                
            elif left_state == 0 and center_state == 1 and right_state == 1:
                #print("right")
                right() 
                pwm_a.ChangeDutyCycle(30)
                pwm_b.ChangeDutyCycle(60)
                time.sleep(0.1)
            # ~ elif left_state == 0 and center_state3
            else:
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
                
            
        if counter == 1:
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            right()
            pwm_a.ChangeDutyCycle(100)
            pwm_b.ChangeDutyCycle(100)
            time.sleep(0.01)
            print("hello")
            gather_ball()
            
        # ~ if kent == 1:
            # ~ print("kent")
            # ~ gather_ball()
            
            
def line_following_reverse():
    counter = 2
    counter_increment_timer = time.time()
    counter_increment_delay = 0.5
    last_detected_time = 0.0
    line_count = 0
    line_detected = False
    cooldown_time = 20  #1.2 Time in seconds to ignore line detections after a detection
    last_detection_time = 0.0
    while True:
        leftback_state = GPIO.input(ir1back_pin)
        centerback_state = GPIO.input(ir2back_pin)
        rightback_state = GPIO.input(ir3back_pin)
        ir_counter = GPIO.input(ircounter_pin)
        
        if ((rightback_state == 1 and leftback_state == 1 or ir_counter == 0) and centerback_state ) and not line_detected and time.time() - last_detection_time > cooldown_time:
            line_count += 1
            line_detected = True
            last_detection_time = time.time()
            print(f"Line count: {line_count}")
            
        if not rightback_state and not leftback_state:
            line_detected = False 

        if line_count == 99:
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(20)   
        
  
        if counter < 10:
            if leftback_state and centerback_state and rightback_state:
                print("backward 2") #change
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
                time.sleep(.05)
            if leftback_state == 1:
                print("left i")
                left()
                pwm_a.ChangeDutyCycle(100)
                pwm_b.ChangeDutyCycle(50)
                #time.sleep(0.115)
            elif centerback_state == 1:
                print("backward 1")
                move_backward()
                pwm_a.ChangeDutyCycle(25)
                pwm_b.ChangeDutyCycle(25)
            elif rightback_state == 1:
                print("right i")
                right()
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)
                #time.sleep(0.0862)
            elif leftback_state == 1 and centerback_state == 0 and rightback_state == 1:
                print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)  
                time.sleep(0.15)
            elif leftback_state == 1 and centerback_state == 1 and rightback_state == 0:
                print("right")
                right() 
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(100)
                time.sleep(0.15)
            else:
                move_backward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
        #if counter == 3:
         #   gather_ball()

def line_following_reset():
    counter = 0
    counter_increment_timer = time.time()
    counter_increment_delay = 5
    last_detected_time = 0.0
    line_detected = True
    kent = 0
    while True:
        left_state = GPIO.input(ir1_pin)
        center_state = GPIO.input(ir2_pin)
        right_state = GPIO.input(ir3_pin)
        ir_counter = GPIO.input(ircounter_pin)

        
        if counter < 10:
            if left_state == 1 and center_state == 1 and right_state == 1:
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
            if ir_counter == 1:
                counter += 1
                print("Counter:", counter)
                last_detected_time = time.time()
                time.sleep(0.3)
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
            
            if left_state == 1:
                # ~ print("left")
                left()
                pwm_a.ChangeDutyCycle(70)
                pwm_b.ChangeDutyCycle(50)
            elif center_state == 1:
                # ~ print("forward")
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
            elif right_state == 1:
                # ~ print("right")
                right()
                pwm_a.ChangeDutyCycle(50)
                pwm_b.ChangeDutyCycle(70)
                
            # ~ elif left_state == 1 and center_state == 0 and right_state == 1:
                # ~ #print("right")
                # ~ right() 
                # ~ pwm_a.ChangeDutyCycle(50)
                # ~ pwm_b.ChangeDutyCycle(70) 
                # ~ time.sleep(0.1)
                
            # ~ elif left_state == 0 and center_state == 1 and right_state == 1:
                # ~ #print("right")
                # ~ right() 
                # ~ pwm_a.ChangeDutyCycle(50)
                # ~ pwm_b.ChangeDutyCycle(70)
                # ~ time.sleep(0.1)

            else:
                forward()
                pwm_a.ChangeDutyCycle(20)
                pwm_b.ChangeDutyCycle(20)
                
        if counter == 2  and kent == 0:
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(1)
            right()
            pwm_a.ChangeDutyCycle(50)
            pwm_b.ChangeDutyCycle(100)
            time.sleep(0.1)
            print("hello")
            kent = 1
            
        if kent == 1:
            print("kent")
            gather_ball()
            
# Run the line following algorithm
try:
    # ~ line_following_reset()
    # ~ line_following_reset()
    # ~ dispense_red()
    # ~ dispense_blue()
    # ~ dispense_green()
    gather_ball()
    # ~ line_following_reverse()
    #line_following()
except KeyboardInterrupt:
    pass

# Cleanup GPIO
pwm_a.stop()
pwm_b.stop()
pwm_servo1.stop()
pwm_servo2.stop()
pwm_servo3.stop()
pwm_servo4.stop()  
GPIO.cleanup()
