#!/usr/bin/python
__author__ = 'davinellulinvega'

# Import the required packages
from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker
from optparse import OptionParser
import almath
from time import sleep

# Declare global variables
nao_ip = "192.168.0.100"
nao_port = 9559
memory = None
test_module = None


# Declare a test module
class TestModule(ALModule):
	"""A module used for testing Nao's functionality"""

	def __init__(self, var_name):
		"""Define the module's attributes and initialize their value"""

		# Request the parent module to initialize and register the module
		ALModule.__init__(self, var_name)

		# Declare the memory
		global memory
		memory = ALProxy("ALMemory")

		# Initialize a motion module
		self.motion = ALProxy("ALMotion")
		# Initialize a posture module
		self.posture = ALProxy("ALRobotPosture")

	def initialize(self):
		"""Initialize the inner modules of the module"""

		# Wake the robot up
		self.motion.wakeUp()
		# Enable the whole body motion
		self.motion.wbEnable(True)

	def shutdown(self):
		"""Shut the module down gracefully"""

		# Disable the whole body motion
		self.motion.wbEnable(False)
		# Put the robot to rest
		self.motion.rest()

	def kick(self):
		"""Define the procedure to follow to accomplish a kick"""

		# Go to stand init posture
		self.posture.goToPosture("StandInit", 0.5)

		# Define the joints we are interested in
		hip_joints = ["RHipRoll", "LHipRoll"]

		# Get the angles for HipRolls
		angles = self.motion.getAngles(hip_joints, True)

		# Modify the value of the hip rolls
		angles[0] -= 10 * almath.TO_RAD
		angles[1] += 10 * almath.TO_RAD

		# Set the angles to have Nao stand on one foot
		self.motion.setAngles(hip_joints, angles, 0.2)

		# Wait for some time
		sleep(5)

		# Go back to stand init posture
		self.posture.goToPosture("StandInit", 0.5)


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

	# Create a search module
	global test_module
	test_module = TestModule("test_module")
	# Initialize all sub-modules and the brain
	test_module.initialize()
	try:
		# Execute a kick
		test_module.kick()
		# Shutdown the module
		test_module.shutdown()
	except KeyboardInterrupt:
		# Shutdown the module
		test_module.shutdown()
		# Shut the broker down
		broker.shutdown()
		# Leave a message for the guy in front of the computer
		print("\nInterruption received from the user.")
		# Leave the program
		exit(0)

if __name__ == "__main__":
	main()
