#Import Library:
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from onrobot_rg_control.msg import OnRobotRGOutput

class TwistController:

    def __init__(self):
        print('Robot is controlles by Controller!')

        # publisher to moveit servo
        self.pub = rospy.Publisher('/twist_controller/command', Twist, queue_size=10)
        self.command = Twist()
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Gripper:
        self.gripper_pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
        self.command_pub = OnRobotRGOutput()

        self.command_loop()

    ## - OpenGripper Function:
    def OpenGripper(self,force=400):
        # Define a message:
        self.command_pub.rGFR = force
        self.command_pub.rGWD = 1100
        self.command_pub.rCTR = 16
        self.gripper_pub.publish(self.command_pub)
    
    ## - CloseGripper Function:
    def CloseGripper(self, force=400):
        # Define a message:
        self.command_pub.rGFR = force
        self.command_pub.rGWD = 600
        self.command_pub.rCTR = 16
        self.gripper_pub.publish(self.command_pub)

    def joy_callback(self, msg):
        # Define the command and send to the robot:
        if msg.buttons[5] == 1:
            self.command.linear.x = msg.axes[0]*0.1
            self.command.linear.y = msg.axes[1]*0.1
            self.command.linear.z = (msg.buttons[1]-msg.buttons[0])*0.1
            self.command.angular.x = msg.axes[4]*0.1
            self.command.angular.y = msg.axes[3]*0.1
            self.command.angular.z = msg.axes[6]*0.5
            print(f'Sennding Linear x: {msg.axes[0]*0.05}, Linear y: {msg.axes[1]*0.05}, Linear z: {msg.axes[2]*0.05}')
            
            if msg.buttons[3] == 1:
                self.OpenGripper()
                print('Open Gripper')
            
            if msg.buttons[2] == 1:
                self.CloseGripper()
                print('Close Gripper')

        else:
            print('Press RB to begin sending velocity to the robot')
            self.command.linear.x = 0
            self.command.linear.y = 0
            self.command.linear.z = 0
            self.command.angular.x = 0
            self.command.angular.y = 0
            self.command.angular.z = 0
        print(msg)

    def command_loop(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            # Update the header timestamp to current time
            self.pub.publish(self.command)
            rate.sleep()

        print('Sending command')
    

    
        
        