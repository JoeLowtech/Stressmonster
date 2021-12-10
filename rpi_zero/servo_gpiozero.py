import sys
from gpiozero import AngularServo
from time import sleep

# Just for demonstration, lots of delay , Not suitable for stable PWM

stress_level = float(sys.argv[1])
if stress_level > 100 :
    stress_level =100
angle = stress_level * 110/100 -20

servo_left = AngularServo(18,initial_angle = angle,min_angle=90,max_angle=-90, min_pulse_width= 1/1000, max_pulse_width=2.5/1000)
servo_right = AngularServo(13,initial_angle = angle,min_angle=-90,max_angle=90, min_pulse_width= 1/1000, max_pulse_width=2.5/1000)
sleep(3)

servo_left.close()
servo_right.close()
