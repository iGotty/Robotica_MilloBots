import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO

class CarController(Node):

    def __init__(self):
        super().__init__('car_controller')
        
        self.Motor_A_EN    = 4
        self.Motor_B_EN    = 17

        self.Motor_A_Pin1  = 14
        self.Motor_A_Pin2  = 15
        self.Motor_B_Pin1  = 27
        self.Motor_B_Pin2  = 18

        self.Dir_forward   = 0
        self.Dir_backward  = 1

        self.left_forward  = 0
        self.left_backward = 1

        self.right_forward = 0
        self.right_backward= 1

        self.pwn_A = 0
        self.pwm_B = 0
        self.setup_motors()

        self.cmd_vel_pub = self.create_publisher(Twist, '/robot_cmdVel', 10)

        self.key_sub = self.create_subscription(String, '/key_pressed', self.key_callback, 10)

    def motorStop(self):#Motor stops
        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_A_EN, GPIO.LOW)
        GPIO.output(self.Motor_B_EN, GPIO.LOW)
    
    def setup_motors(self):
        global pwm_A, pwm_B
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)

        self.motorStop()
        try:
            pwm_A = GPIO.PWM(self.Motor_A_EN, 1000)
            pwm_B = GPIO.PWM(self.Motor_B_EN, 1000)
        except:
            pass
    
    def motor_left(self,status, direction, speed):#Motor 2 positive and negative rotation
        if status == 0: # stop
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
            GPIO.output(self.Motor_B_EN, GPIO.LOW)
        else:
            if direction == self.Dir_backward:
                GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
                GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
                pwm_B.start(100)
                pwm_B.ChangeDutyCycle(speed)
            elif direction == self.Dir_forward:
                GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
                GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)
                pwm_B.start(0)
                pwm_B.ChangeDutyCycle(speed)


    def motor_right(self,status, direction, speed):#Motor 1 positive and negative rotation
        if status == 0: # stop
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
            GPIO.output(self.Motor_A_EN, GPIO.LOW)
        else:
            if direction == self.Dir_forward:#
                GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
                GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
                pwm_A.start(100)
                pwm_A.ChangeDutyCycle(speed)
            elif direction == self.Dir_backward:
                GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
                GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)
                pwm_A.start(0)
                pwm_A.ChangeDutyCycle(speed)
        return direction


    def move(self,speed, direction, turn, radius=0.6):   # 0 < radius <= 1  
        #speed = 100
        if direction == 'forward':
            if turn == 'right':
                self.motor_left(0, self.left_backward, int(speed*radius))
                self.motor_right(1, self.right_forward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(0, self.right_backward, int(speed*radius))
            else:
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(1, self.right_forward, speed)
        elif direction == 'backward':
            if turn == 'right':
                self.motor_left(0, self.left_forward, int(speed*radius))
                self.motor_right(1, self.right_backward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(0, self.right_forward, int(speed*radius))
            else:
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(1, self.right_backward, speed)
        elif direction == 'no':
            if turn == 'right':
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(1, self.right_forward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(1, self.right_backward, speed)
            else:
                self.motorStop()
        else:
            pass

    def key_callback(self, msg):
        key = msg.data
        if key == 'w':
            self.move(80,"forward","no",0.8)
        elif key == 's':
            self.move(80,"backward","no",0.8)
        elif key == 'q':
            self.move(80,"forward","left",0.8)
        elif key == 'e':
            self.move(80,"forward","rigth",0.8)
        else:
            self.motorStop()


    def destroy(self):
        self.motorStop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    car_controller = CarController()

    rclpy.spin(car_controller)

if __name__ == '__main__':
    print("CarController node initialized")
    main()
