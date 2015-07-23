#!/usr/bin/python
__author__ = 'davinellulinvega'


# Import the required packages
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBroker
from optparse import OptionParser
from time import sleep
from math import pi
from math import atan
from math import tan
from math import sqrt
import motion
import almath

# Define global parameters
nao_ip = "192.168.0.100"
nao_port = 9559
memory = None
soccer_module = None
mark_id = 85


def compute_path(proxy, effector, frame):
    # Initialize the parameters
    dx = 0.08  # translation axis X (meters)
    dz = 0.07  # translation axis Z (meters)
    dwy = 5.0 * almath.TO_RAD  # rotation axis Y (radian)

    path = []
    current_tf = []

    try:
        # Try to get the current transform for the effector
        current_tf = proxy.getTransform(effector, frame, False)
    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()

    # Compute the first portion of the movement (moving backward)
    targetTf = almath.Transform(current_tf)
    targetTf *= almath.Transform(0.0, 0.0, dz)
    targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))

    # Compute the second part of the movement (moving forward)
    targetTf = almath.Transform(current_tf)
    targetTf *= almath.Transform(dx, 0.0, dz)
    path.append(list(targetTf.toVector()))

    # Finally compute the path for moving the effector in its original position
    path.append(current_tf)

    # Return the computed path
    return path


# Define the soccer module
class SoccerModule(ALModule):
    """A simple module allowing Nao to play soccer"""

    def __init__(self, var_name):
        """Define and initialize the class' attributes"""

        # Request the parent module to initialize and register our module
        ALModule.__init__(self, var_name)

        # Initialize the memory
        global memory
        memory = ALProxy("ALMemory")
        # Initialize the motion module
        self.motion = ALProxy("ALMotion")
        # Initialize the posture module
        self.posture = ALProxy("ALRobotPosture")

        # Initialize the goal position
        self.goal = [0, 0]
        # Initialize the ball position
        self.ball = [0, 0]
        # Initialize a command stack
        self.move = None

    def initialize(self):
        """Subscribe to the required events and enable the different sub-modules"""

        # Enable the fall manager
        self.motion.setFallManagerEnabled(True)
        # Enable the body balancer
        self.motion.wbEnable(True)

        # Wake the robot up
        self.motion.wakeUp()
        # Request the robot to stand
        self.posture.goToPosture("StandInit", 0.5)

        # Subscribe to the redBallDetected event
        memory.subscribeToEvent("redBallDetected", self.getName(), "on_red_ball")
        # Subscribe to the robot has fallen event
        memory.subscribeToEvent("robotHasFallen", self.getName(), "on_fall")
        # memory.subscribeToEvent("ALMotion/RobotIsFalling", self.getName(), "on_fall")
        # Subscribe to the bumperPressed event
        # memory.subscribeToEvent("RightBumperPressed", self.getName(), "on_bumper_pressed")
        # memory.subscribeToEvent("LeftBumperPressed", self.getName(), "on_bumper_pressed")
        # Subscribe to the landmarkDetected event
        memory.subscribeToEvent("LandmarkDetected", self.getName(), "on_landmark_detected")

    def shutdown(self):
        """Define the procedure to follow upon shutting down"""

        # Request the robot to go in resting position
        self.motion.rest()
        # Set the body stiffness to 0
        self.motion.setStiffnesses("Body", 0)

    def on_red_ball(self):
        """Compute the position of the ball relative to the robot and move towards it"""

        # Check if we are not already planning a move for the red ball
        if self.move is None:
            # Get the data from the memory
            data = memory.getData("redBallDetected")
            ball_info = data[1]
            camera_pos = data[3]

            # Compute the position of the ball relative to the robot
            self.ball[0] = camera_pos[2] * tan(camera_pos[4] + ball_info[1] + ball_info[2]) + camera_pos[0]
            self.ball[1] = camera_pos[2] * tan(camera_pos[3]) + camera_pos[1]

            # Compute the distance between the robot and the ball
            dist = sqrt(self.ball[0] ** 2 + self.ball[1] ** 2)
            # Compute the rotation of the ball to the camera center
            rotation = camera_pos[-1] + ball_info[0]

            # Set the next moves to be executed
            self.move = ([0, 0, rotation], [dist, 0, 0])

            # Move the head to face the ball
            # self.motion.setAngles("HeadYaw", -ball_info[0], 0.2)

            # Warn that we found a red ball
            print("RED BALL DETECTED " + str(self.ball))

    def on_landmark_detected(self):
        """Compute the position of the landmark relative to the robot"""

        # Check if we are not already planning a move for the landmark
        if self.move is None:
            # Get the data from the memory
            data = memory.getData("LandmarkDetected")
            # Check that we have the right mark
            if data[1][0][1] == mark_id:
                # Extract information regarding the mark
                mark_info = data[1][0][0]
                camera_pos = data[3]
                # Compute the position of the landmark relative to the robot
                self.goal[0] = camera_pos[2] * tan(camera_pos[4] + mark_info[2] + mark_info[3]) + camera_pos[0]
                self.goal[1] = camera_pos[2] * tan(camera_pos[3]) + camera_pos[1]
                # Turn the robot toward the goal
                rotation = atan((self.goal[0] - self.ball[0]) / (self.goal[1] - self.ball[1]))

                # Add the action to the stack
                self.move = ([0, 0, rotation], "kick")
                # Warn that we found a landmark
                print("LANDMARK DETECTED " + str(self.goal))

    def on_fall(self):
        """Set the stiffness back and request the robot to get up"""

        # Set the body stiffness
        self.motion.setStiffnesses("Body", 1)
        # Ask the robot to stand up
        self.posture.goToPosture("StandInit", 0.5)

    def on_bumper_pressed(self):
        """Have the robot turn around and walk away"""

        # Stop all movements
        self.motion.stopMove()
        # Turn around
        self.motion.moveTo(0, 0, pi / 2)

    def look_around(self):
        """Move the robot head from -2.08 to 2.08"""

        # Set the head pitch angle
        self.motion.setAngles(["HeadPitch"], [20 * almath.TO_RAD], 0.5)
        # Set the head yaw angle
        self.motion.setAngles(["HeadYaw"], [-2.08], 0.1)
        # Wait for some time
        sleep(3)
        # set the head yaw angle
        self.motion.setAngles(["HeadYaw"], [2.08], 0.1)
        # Wait for some time
        sleep(3)
        # Set the head back to 0
        self.motion.setAngles(["HeadYaw"], [0], 0.5)
        # Wait for some time
        sleep(1)

    def next_move(self):
        """Execute the next move in the stack"""

        # Check if there is any move to be done
        if self.move is None:
            # Return a bad result
            result = False
        else:
            # Execute the move
            for moves in list(self.move):
                # Check if a kick
                if moves == "kick":
                    # Have the robot kick
                    self.kick()
                else:
                    # Make the robot move
                    self.motion.moveTo(moves[0], moves[1], moves[2])
                # Return a good result
                result = True
                # Display the next move
                print("NEXT ACTION: " + str(moves))
            # Assign None to move
            self.move = None

        # Return the final result
        return result

    def kick(self):
        """Have the robot execute a kick"""

        # Request robot to go to stand init posture
        self.posture.goToPosture("StandInit", 0.5)

        # Activate whole body balancer
        self.motion.wbEnable(True)

        # Constraint both legs as fixed
        self.motion.wbFootState("Fixed", "Legs")

        # Request legs joints to move to keep robot's balance
        self.motion.wbEnableBalanceConstraint(True, "Legs")

        # Balance the robot on the left leg
        self.motion.wbGoToBalance("LLeg", 2.5)

        # Remove fixed constraint from the right leg
        self.motion.wbFootState("Free", "RLeg")

        # Compute the path for moving the right leg back and forth
        path = compute_path(self.motion, "RLeg", motion.FRAME_WORLD)

        # Request the right leg to move following the computed path
        times = [1.2, 1.6, 2.3]
        self.motion.transformInterpolations("RLeg", motion.FRAME_WORLD, path, 63, times)

        # Request the robot to stand in initial position
        self.posture.goToPosture("StandInit", 0.2)


# Definition of the main function
def main():
    """Define the main entry point"""

    # Get an option parser
    parser = OptionParser()
    # Define the options to be used
    parser.add_option("--pip", help="Parent Broker ip. The IP address of your robot.", dest="pip")
    parser.add_option("--pport", help="Parent Broker port. The port your robot is listening to.", dest="pport",
                      type=int)
    # Define the default values for these options
    parser.set_defaults(pip=nao_ip, pport=nao_port)
    # Parse the options
    (opts, args) = parser.parse_args()
    pip = opts.pip
    pport = opts.pport

    # Create a broker for other modules/people to be able to access this module methods
    broker = ALBroker("broker", "0.0.0.0", 0, pip, pport)

    try:
        # Create a search module
        global soccer_module
        soccer_module = SoccerModule("soccer_module")
        # Initialize all sub-modules
        soccer_module.initialize()
        while True:
            # Execute the next move if possible
            if not soccer_module.next_move():
                # Have a look around
                soccer_module.look_around()
                sleep(3)
                # TODO: Move in a strategic way
    except KeyboardInterrupt:
        # Shut the module down
        soccer_module.shutdown()
        # Shut the broker down
        broker.shutdown()
        # Leave a message for the guy in front of the computer
        print("\nInterruption received from the user.")
        # Leave the program
        exit(0)


if __name__ == "__main__":
    main()
