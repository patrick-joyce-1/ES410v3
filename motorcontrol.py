import RPi.GPIO as GPIO
import time

# Pin Definitions
motor_pin_a = 18  # BOARD pin 12, BCM pin 18

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # Set both pins LOW to keep the motor idle
    # You can keep one of them HIGH and the LOW to start with rotation in one direction
    GPIO.setup(motor_pin_a, GPIO.OUT, initial=GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value_pin_a = GPIO.HIGH

    try:
        while True:
            time.sleep(1)
            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value_pin_a,motor_pin_a))
            GPIO.output(motor_pin_a, curr_value_pin_a)
            curr_value_pin_a ^= GPIO.HIGH
            curr_value_pin_a ^= GPIO.LOW
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
