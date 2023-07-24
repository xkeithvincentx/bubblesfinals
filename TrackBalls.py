import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

ena_pin = 13
in1_pin = 23
in2_pin = 24

enb_pin = 12
in3_pin = 27
in4_pin = 22

# GPIO pins for servos
SERVO1_PIN = 20
SERVO2_PIN = 21
SERVO3_PIN = 26
SERVO4_PIN = 16


# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pins as inputs and outputs
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
    
def search_balls(cap):

    ret, frame = cap.VideoCapture(0)

    
    distance_ball = 110

    loopVar = 0

    while True:
        print("searching left")
        left()
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.3)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)
    
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)

        ###START CODE SEARCH RIGHT - CENTER
        while loopVar < 5:
            right() 
            pwm_a.ChangeDutyCycle(70)
            pwm_b.ChangeDutyCycle(70)
            time.sleep(0.1)
            stop_motors()
            pwm_a.ChangeDutyCycle(0)
            pwm_b.ChangeDutyCycle(0)
            time.sleep(2)    

            if verify_distance(cap) < 1000:

            loopVar += 1

        right() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)

        right() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)

        right() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)

        right() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)

        print("searching right")
        right()
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.3)
        stop_motors()

        ###############BACK TO CENTER############
        



        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)
        left() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)
        left() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)
        left() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)
        left() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)
        left() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)

def confirm_distance(desired_distance):

    ret, frame = cap.read()

    tolerance_percentage = 36
    known_radius = 35  #25 Radius of the ball in real life (mm)
    focal_length = 700  # Known focal length of the camera (mm)

    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])

    lower_green = np.array([36, 50, 50])
    upper_green = np.array([86, 255, 255])

    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])

    balls = []
    balls += detect_balls(frame, lower_red, upper_red, color_name="red")
    balls += detect_balls(frame, lower_green, upper_green, color_name="green")
    balls += detect_balls(frame, lower_blue, upper_blue, color_name="blue")

    # Calculate distance for each ball and store it along with the ball info
    balls_with_dist = [(x, y, radius, color_name, calculate_distance(radius, known_radius, focal_length)) for
                       (x, y, radius, color_name) in balls]

    # Sort balls by distance and then by their x-coordinate (closest and more centered first)
    balls_with_dist = sorted(balls_with_dist, key=lambda ball: (ball[4], abs(ball[0] - frame.shape[1] // 2)))

    if balls_with_dist:
        x, y, radius, color_name, distance = balls_with_dist[0]  # Closest and more centered ball

        print(f"Detected ball: {color_name.capitalize()}, Distance: {distance:.2f} mm")

        if distance < desired_distance and abs(x - frame.shape[1] // 2) < 10:
            stop()
        else:
            center_x = frame.shape[1] // 2
            offset_x = x - center_x
            distance_from_center = abs(offset_x)
            verified_dst = verify_distance()

            if verified_dst < 1000:#300
                if distance_from_center <= tolerance_percentage * frame.shape[1] / 100:
                    # Ball is centered
                    second_verify_dst = verify_distance()
                    print("Ball is centered")
                        
                    if second_verify_dst > 110:
                        GPIO.output(m1_ina, GPIO.HIGH)
                        GPIO.output(m1_inb, GPIO.LOW)
                        GPIO.output(m2_ina, GPIO.HIGH)
                        GPIO.output(m2_inb, GPIO.LOW)
                        motor1_pwm.ChangeDutyCycle(35)
                        motor2_pwm.ChangeDutyCycle(25)
                        time.sleep(.1)
                        motor1_pwm.ChangeDutyCycle(0)
                        motor2_pwm.ChangeDutyCycle(0)
                        time.sleep(.5)
                    elif second_verify_dst <= 110:
                        color_name = identify_color(cap)

                        if color_name in "red" and distance <= 110:
                            time.sleep(.3)

                            print("Color is red")
                            time.sleep(2)
                            gather_ball()
                        elif color_name in "green" and distance <= 110:
                            time.sleep(.3)

                            print("Color is green")
                            time.sleep(2)
                            gather_ball()
                        elif color_name in "blue" and distance <= 110:
                            time.sleep(.3)

                            print("Color is blue")
                            time.sleep(2)
                            gather_ball()                    




                    motor1_pwm.ChangeDutyCycle(0)
                    motor2_pwm.ChangeDutyCycle(0)
                    time.sleep(.5)
                else:
                    if offset_x < 0:
                        turn_left_slow()
                        
                        GPIO.output(m1_ina, GPIO.LOW)
                        GPIO.output(m1_inb, GPIO.HIGH)
                        GPIO.output(m2_ina, GPIO.HIGH)
                        GPIO.output(m2_inb, GPIO.LOW)

                        motor1_pwm.ChangeDutyCycle(45)
                        motor2_pwm.ChangeDutyCycle(45)
                        time.sleep(.3)
                        motor1_pwm.ChangeDutyCycle(0)
                        motor2_pwm.ChangeDutyCycle(0)
                        time.sleep(2)
                    else:
                        turn_right_slow()

                        GPIO.output(m1_ina, GPIO.HIGH)
                        GPIO.output(m1_inb, GPIO.LOW)
                        GPIO.output(m2_ina, GPIO.LOW)
                        GPIO.output(m2_inb, GPIO.HIGH)   
                        
                        motor1_pwm.ChangeDutyCycle(45)
                        motor2_pwm.ChangeDutyCycle(45)
                        time.sleep(.3)
                        
                        motor1_pwm.ChangeDutyCycle(0)
                        motor2_pwm.ChangeDutyCycle(0)
                        time.sleep(2)
            else:
                GPIO.output(m1_ina, GPIO.HIGH)
                GPIO.output(m1_inb, GPIO.LOW)
                GPIO.output(m2_ina, GPIO.HIGH)
                GPIO.output(m2_inb, GPIO.LOW)
                motor1_pwm.ChangeDutyCycle(50)
                motor2_pwm.ChangeDutyCycle(50)
                time.sleep(.1) 
                motor1_pwm.ChangeDutyCycle(0)
                motor2_pwm.ChangeDutyCycle(0)
                time.sleep(2)

        return distance   
    else:
        right() 
        pwm_a.ChangeDutyCycle(70)
        pwm_b.ChangeDutyCycle(70)
        time.sleep(0.1)
        stop_motors()
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        time.sleep(2)          

      
    
def set_angle(servo, angle):
    duty = angle / 18 + 2  # Map angle to duty cycle (2 to 12)
    servo.ChangeDutyCycle(duty)

def open_servos():
    # Set angles to open the servos
    set_angle(servo1, 0)  # Set servo 1 to 0 degrees
    set_angle(servo2, 90)  # Set servo 2 to 180 degrees
    
def close_servos():
    # Set angles to close the servos
    set_angle(servo1, 180)  # Set servo 1 to 90 degrees
    set_angle(servo2, 180)  # Set servo 2 to 90 degrees
    
def rise_servos():
    # Set angles to close the servos
    set_angle(servo3, -10)  # Set servo 1 to 90 degrees
    #time.sleep(5)
    
def fall_servo():
    set_angle(servo3, 110)
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
    
def gather_ball():
    print("ASD")
    stop_motors
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(.1)
    
    rise_servos()
    time.sleep(2)
# ~ fall_servo()
# ~ time.sleep(2)
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
            # ~ color_detect()
            # ~ time.sleep(1)

def track_balls(cap):
    known_radius = 25  # Radius of the ball in real life (mm)
    focal_length = 700  # Known focal length of the camera (mm)
    
    coloredBallDistance = 110#62

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        lower_green = np.array([36, 50, 50])
        upper_green = np.array([86, 255, 255])

        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        balls = []
        balls += detect_balls(frame, lower_red, upper_red, color_name="red")
        balls += detect_balls(frame, lower_green, upper_green, color_name="green")
        balls += detect_balls(frame, lower_blue, upper_blue, color_name="blue")

        # Calculate distance for each ball and store it along with the ball info
        balls_with_dist = [(x, y, radius, color_name, calculate_distance(radius, known_radius, focal_length)) for (x, y, radius, color_name) in balls]

        # Sort balls by distance and then by their x-coordinate (closest and more centered first)
        balls_with_dist = sorted(balls_with_dist, key=lambda ball: (ball[4], abs(ball[0] - frame.shape[1] // 2)))

        if balls_with_dist:
            
            x, y, radius, color_name, distance = balls_with_dist[0]  # Closest and more centered ball
            cv2.circle(frame, (x, y), radius, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            print(f"Closest ball: {color_name.capitalize()}, Distance: {distance:.2f} mm")
                
            if color_name in "red" and distance < coloredBallDistance:
                time.sleep(.3)

                print("Color is red")
                time.sleep(2)
                gather_ball()
                break
            elif color_name in "green" and distance < coloredBallDistance:
                time.sleep(.3)

                print("Color is green")
                time.sleep(2)
                gather_ball()
                break
            elif color_name in "blue" and distance < coloredBallDistance:
                time.sleep(.3)

                print("Color is blue")
                time.sleep(2)
                gather_ball()
                break
        else: 
            search_balls(cap, frame)
            # ~ return

                
        cv2.imshow('Frame', frame)
            
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()

def identify_color(cap):
    known_radius = 25  # Radius of the ball in real life (mm)
    focal_length = 700  # Known focal length of the camera (mm)
    
    coloredBallDistance = 110#62

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        lower_green = np.array([36, 50, 50])
        upper_green = np.array([86, 255, 255])

        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        balls = []
        balls += detect_balls(frame, lower_red, upper_red, color_name="red")
        balls += detect_balls(frame, lower_green, upper_green, color_name="green")
        balls += detect_balls(frame, lower_blue, upper_blue, color_name="blue")

        # Calculate distance for each ball and store it along with the ball info
        balls_with_dist = [(x, y, radius, color_name, calculate_distance(radius, known_radius, focal_length)) for (x, y, radius, color_name) in balls]

        # Sort balls by distance and then by their x-coordinate (closest and more centered first)
        balls_with_dist = sorted(balls_with_dist, key=lambda ball: (ball[4], abs(ball[0] - frame.shape[1] // 2)))

        if balls_with_dist:
            
            x, y, radius, color_name, distance = balls_with_dist[0]  # Closest and more centered ball

            return color_name


    cap.release()
    cv2.destroyAllWindows()


def verify_distance(cap):
    known_radius = 25  # Radius of the ball in real life (mm)
    focal_length = 700  # Known focal length of the camera (mm)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        lower_green = np.array([36, 50, 50])
        upper_green = np.array([86, 255, 255])

        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        balls = []
        balls += detect_balls(frame, lower_red, upper_red, color_name="red")
        balls += detect_balls(frame, lower_green, upper_green, color_name="green")
        balls += detect_balls(frame, lower_blue, upper_blue, color_name="blue")

        # Calculate distance for each ball and store it along with the ball info
        balls_with_dist = [(x, y, radius, color_name, calculate_distance(radius, known_radius, focal_length)) for (x, y, radius, color_name) in balls]

        # Sort balls by distance and then by their x-coordinate (closest and more centered first)
        balls_with_dist = sorted(balls_with_dist, key=lambda ball: (ball[4], abs(ball[0] - frame.shape[1] // 2)))

        if balls_with_dist:
            
            x, y, radius, color_name, distance = balls_with_dist[0]  # Closest and more centered ball
            # Draw fixed crosshair centered on the frame
            crosshair_color = (0, 0, 255)
            cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), crosshair_color, 1)
            cv2.line(frame, (0, frame.shape[0] // 2), (frame.shape[1], frame.shape[0] // 2), crosshair_color, 1)
            cv2.putText(frame, "+", (frame.shape[1] // 2 - 10, frame.shape[0] // 2 + 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, crosshair_color, 2)

            # Draw crosshair
            cv2.line(frame, (x, 0), (x, frame.shape[0]), (0, 255, 0), 1)
            cv2.line(frame, (0, y), (frame.shape[1], y), (0, 255, 0), 1)
            return distance
        return 0

def detect_balls(image, lower, upper, min_size=2000, color_name=""):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, lower, upper)

    if color_name == "red":
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    balls = []
    for contour in contours:
        if cv2.contourArea(contour) >= min_size:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            balls.append((int(x), int(y), int(radius), color_name))
    return balls

def calculate_distance(radius, known_radius, focal_length):
    return (known_radius * focal_length) / radius
	

try:
    cap = cv2.VideoCapture(0)
    #track_balls(cap)
    confirm_distance(cap)
except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    pwm_a.stop()
    pwm_b.stop()
    servo1.stop()
    servo2.stop()
    servo3.stop()
    servo4.stop()  
    GPIO.cleanup()
