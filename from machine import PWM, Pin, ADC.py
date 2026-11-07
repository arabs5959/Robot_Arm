from machine import PWM, Pin, ADC
import math
import time
#CONVERTOR

#Arm length
L1 = 155
L2 = 155

#Set up the pins for the knobs to controll the arm
x_axis = ADC(Pin(26))  #X potentiometer on GPIO 26
y_axis = ADC(Pin(27))  #Y potentiometer on GPIO 27



#PWM for shoulder and elbow servos
shoulder_pwm = PWM(Pin(0))# Shoulder servo on GPIO 0
elbow_pwm = PWM(Pin(1)) #Elbow servo on GPIO 1


# Set PWM frequency for arm
shoulder_pwm.freq(50)  # Standard servo frequency 50 Hz
elbow_pwm.freq(50)


#Best guess at min and max duty cycles
D_MIN = 2300  # Minimum allowed duty cycle
D_MAX = 7500  # Maximum allowed duty cycle
PEN_UP = 2300    # Duty cycle for pen up
PEN_DOWN = 3000  # Duty cycle for pen down

#FUNctions :)
#Saftey functions for duty and angle conversions
def safe_duty(servo, duty):
#Make sure the angle is within the range of 0 to 180 degrees
    if D_MIN <= duty <= D_MAX:
         servo.duty_u16(duty)
    else:
        print(f"Duty cycle {duty} is out of range.")
        return
#Make sure angle set safe
def safe_angle(servo, angle):
    if 0 <= angle <= 180:
        duty = translate(angle)
        safe_duty(servo, duty)

###############################
def inverse_kinematics(x_cord, y_cord):
    #Most was taken from https://www.mathworks.com/help/symbolic/derive-and-apply-inverse-kinematics-to-robot-arm.html
    distance = math.sqrt(x_cord**2 + y_cord**2)
    #make sure the targets in reach
    if  (distance > (L1 + L2) or distance < abs(L1 - L2)):
            print(f"Target position is out of reach: {x_cord}, {y_cord}")
            return None, None
    else:
    #calculate elbow angle (theta2)
        theta2 = math.acos((distance**2 - L1**2 - L2**2) / (2 * L1 * L2))
    #calculate shoulder angle (theta1)
        theta1 = math.atan2(y_cord, x_cord) - math.acos((L2**2 + distance**2 - L1**2) / (2 * L2 * distance))
        return math.degrees(theta1), math.degrees(theta2)
##########################################
def translate(angle: float) -> int:
    pulse_width = 500 + (2500 - 500) * angle / 180  #Pulse width equation
    duty_cycle = pulse_width / 20000  #20000 microseconds 
    duty_u16_value = int(duty_cycle * 65535)#Scale to 16-bit range
    duty_u16_value = max(D_MIN, min(D_MAX, duty_u16_value))#clamp(limit it) to safe range
    return duty_u16_value
################
def read_knobs():
     """read the values from the potentiometers and convert them to x and y coordinates"""
     x_value = x_axis.read_u16()  #read the X potentiometer value (0-65535)
     y_value = y_axis.read_u16()  #read the Y potentiometer value (0-65535)
    
    
     return x_value, y_value
################Set up pen control (up and down)
pen_switch = Pin(12, Pin.IN, Pin.PULL_DOWN)
pen_pwm = PWM(Pin(2))
pen_pwm.freq(50)  # Standard servo frequency 50 Hz
pen_write = False # False = up, True = down
last_pen_button_state = 0
##########
def pen_control():
     global pen_write, last_pen_button_state
     current_pen_button_state = pen_switch.value()
     if current_pen_button_state == 1 and last_pen_button_state == 0: #have it like this to actaully see its pressed!
            pen_write = not pen_write  #toggle the pen state
            print(f"Pen state changed: {'Ready!' if pen_write else 'Waiting!'}")
            pen_state = PEN_DOWN if pen_write else PEN_UP
            safe_duty(pen_pwm, pen_state)
    
     last_pen_button_state = current_pen_button_state
######################################################33
def ITS_ALIVE():
    print("The robot arm is alive and ready to move!")
    #Gets the intial values of the knobs and converts them to x and y coordinates
    x_value, y_value = read_knobs()

    #mapping the values in the range (1, 300(total arm length)) 
    x_cord = (x_value / 65535) * (2 * (L1 + L2)) - (L1 + L2)
    y_cord = (y_value / 65535) * (2 * (L1 + L2)) - (L1 + L2)

    #Ugly math
    theta1, theta2 = inverse_kinematics(x_cord, y_cord)
    if theta1 is None or theta2 is None:
        print("Initial position is out of reach. Please adjust the knobs.")
        return

    print(f"Initial position: x={x_cord:.2f}, y={y_cord:.2f}, theta1={theta1:.2f}, theta2={theta2:.2f}")
    safe_angle(shoulder_pwm, theta1)
    safe_angle(elbow_pwm, theta2)

    #PEN!
    pen_control()  # Set the initial pen state


def main():
    """Main entry point to run the robotic arm control."""
    print("Starting Arm Control...")
    while True:
        ITS_ALIVE()
        time.sleep(0.5)

if __name__ == "__main__": #The things ai just kinda added this for me, but it works so I guess its fine.
    main()
