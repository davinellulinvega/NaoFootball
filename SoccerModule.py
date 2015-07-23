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

# Define global parameters
nao_ip = "192.168.0.100"
nao_port = 9559
memory = None
soccer_module = None
mark_id = 85


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
		# Initialize the tracker module
		self.tracker = ALProxy("ALTracker")

		# Initialize the tracking state
		self.is_tracking = False

	def initialize(self):
		"""Subscribe to the required events and enable the different sub-modules"""

		# Enable the fall manager
		self.motion.setFallManagerEnabled(True)
		# Enable the body balancer
		self.motion.wbEnable(True)

		# Register the targets for the tracker
		self.tracker.registerTarget("RedBall", 0.5)
		self.tracker.registerTarget("LandMark", [0.2, [85]])
		# Turn of the search
		self.tracker.toggleSearch(False)

		# Subscribe to the redBallDetected event
		memory.subscribeToEvent("redBallDetected", self.getName(), "on_red_ball")
		# Subscribe to the robot has fallen event
		memory.subscribeToEvent("robotHasFallen", self.getName(), "on_fall")
		#memory.subscribeToEvent("ALMotion/RobotIsFalling", self.getName(), "on_fall")
		# Subscribe to the bumperPressed event
		memory.subscribeToEvent("RightBumperPressed", self.getName(), "on_bumper_pressed")
		memory.subscribeToEvent("LeftBumperPressed", self.getName(), "on_bumper_pressed")
		# Subscribe to the landmarkDetected event
		memory.subscribeToEvent("LandmarkDetected", self.getName(), "on_landmark_detected")

		# Wake the robot up
		self.motion.wakeUp()
		# Request the robot to stand
		self.posture.goToPosture("StandInit", 0.5)

	def shutdown(self):
		"""Define the procedure to follow upon shutting down"""

		# Stop the tracker
		self.tracker.stopTracker()
		# Unregister all tracker targets
		self.tracker.unregisterAllTargets()
		# Request the robot to go in resting position
		self.motion.rest()
		# Set the body stiffness to 0
		self.motion.setStiffnesses("Body", 0)

	def on_red_ball(self):
		"""Compute the position of the ball relative to the robot and move towards it"""

		# Check if we are not already tracking the red ball
		if self.tracker.getActiveTarget() != "RedBall":
			# Set tracker mode to Move
			self.tracker.setMode("Move")
			# Start to track the red ball
			self.tracker.track("RedBall")
			# Warn that we found a red ball
			print("RED BALL DETECTED")
			# Set the tracking state
			self.is_tracking = True

	def on_landmark_detected(self):
		"""Compute the position of the landmark relative to the robot"""

		# Check if we are not already tracking the land mark
		if self.tracker.getActiveTarget() != "LandMark":
			# Set tracker mode to WholeBody
			self.tracker.setMode("WholeBody")
			# Start tracking the landmark
			self.tracker.track("LandMark")
			# Warn that we found a landmark
			print("LANDMARK DETECTED")
			# Set the tracking state
			self.is_tracking = True

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
		self.motion.moveTo(0, 0, pi/2)

	def look_around(self):
		"""Move the robot head from -2.08 to 2.08"""

		# Set the head yaw angle
		self.motion.setAngles(["HeadYaw"], [-2.08], 0.2)
		# Wait for some time
		sleep(3)
		# set the head yaw angle
		self.motion.setAngles(["HeadYaw"], [2.08], 0.2)
		# Wait for some time
		sleep(3)
		# Set the head back to 0
		self.motion.setAngles(["HeadYaw"], [0], 1.0)

	def is_target_lost(self):
		"""Check if the tracker is still locked on a target or not"""

		# Check the state of the target
		if self.tracker.isTargetLost():
			# Stop the tracker
			self.tracker.stopTracker()
			# Set the state of the tracking
			self.is_tracking = False
			# Warn that we have lost the target
			print("TRACKER TARGET LOST")

	# TODO: Kick function and event triggering the kick delivery


# Definition of the main function
def main():
	"""Define the main entry point"""

	# Get an option parser
	parser = OptionParser()
	# Define the options to be used
	parser.add_option("--pip", help="Parent Broker ip. The IP address of your robot.", dest="pip")
	parser.add_option("--pport", help="Parent Broker port. The port your robot is listening to.", dest="pport", type=int)
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
			if not soccer_module.is_tracking:
				# Have a look around
				soccer_module.look_around()
				# Wait for some time
				sleep(3)
				#TODO: Move in a strategic way
			else:
				# Check if the target is still in sight
				soccer_module.is_target_lost()
				# Wait for some time
				sleep(1)
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
