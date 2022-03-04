import time
import cv2
import os
import RPi.GPIO as GPIO

## Added a for loop within the if statements

# Pin Definitions
motor_pin_a = 18  # BOARD pin 18
motor_pin_b = 12  # BOARD pin 12


def main():
    try:
        # Pin Setup
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # Set both pins to LOW
        GPIO.setup(motor_pin_a, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(motor_pin_b, GPIO.OUT, initial=GPIO.LOW)
        
        curr_value_pin_a = GPIO.LOW
        curr_value_pin_b = GPIO.LOW
                
        # Initialise variables
        camerafov = 60  # FOV (degrees)
        camerares = [1920, 1080]  # Resolution
        app = camerafov / camerares[0]  # Angle per pixel
        motorposition = 0  # Motor angle

        while True:
            # Apriltag position system
            # determine position of apriltag
            # script about detecting apriltag position
            # corners = [[#,#],[#,#],[#,#],[#,#]]
            # pixel = (corners[0,1]+corners[1,1]+corners[2,1]+corners[3,1])/4 # average apriltag position in x direction

            #pixel = input("Enter measured value")
            #pixelangle = (pixel - 540) * app
            pixelangle = int(input("Enter measured value"))
            print("Measured angle is: ", pixelangle)
            # middle is 540

            # Alignment system
            if abs(motorposition - pixelangle) > 0.45:  # if greater than highest precision
                if motorposition > pixelangle:  # Set GPIO pin anti-clockwise
                    curr_value_pin_b = GPIO.LOW
                    GPIO.output(motor_pin_b, 0)
                    print("anticlockwise")
                elif motorposition < pixelangle:  # Set GPIO pin clockwise
                    curr_value_pin_b = GPIO.HIGH
                    GPIO.output(motor_pin_b, curr_value_pin_b)
                    print("clockwise")
                for i in range(1, int((round(abs(motorposition - pixelangle) / 1.8)*2) + 1)):
                    # Set GPIO pin signal high-low
                    curr_value_pin_a ^= GPIO.HIGH
                    GPIO.output(motor_pin_a, curr_value_pin_a)
                    curr_value_pin_a ^= GPIO.LOW
                    GPIO.output(motor_pin_a, curr_value_pin_a)
                    time.sleep(0.02)

                    curr_value_pin_a ^= GPIO.HIGH
                    GPIO.output(motor_pin_a, curr_value_pin_a)
                    curr_value_pin_a ^= GPIO.LOW
                    GPIO.output(motor_pin_a, curr_value_pin_a)
                    time.sleep(0.02)

                    
                    if curr_value_pin_b == GPIO.LOW:  # Anti-clockwise
                        motorposition -= 0.9
                    elif curr_value_pin_b == GPIO.HIGH:  # Clockwise
                        motorposition += 0.9
                    print("Motor position is: ", motorposition)

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
