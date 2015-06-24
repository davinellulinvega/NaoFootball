#!/usr/bin/python
__author__ = 'davinellulinvega'

# Import the required packages
from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker
from optparse import OptionParser
import almath

# Declare global variables
nao_ip = "127.0.0.1"
#nao_ip = "192.168.0.100"
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

	def compute_path(self, effector):
		"""Compute the optimal path to realize a kick through the given effector"""

		# Initialize the parameters
		path = []
		current_tf = []
		dx = 0.01
		dz = 0.05
		dwy = 2 * almath.TO_RAD

		# Get the current transform for the effector
		try:
			current_tf = self.motion.getTransform(effector, self.motion.FRAME_WORLD, False)
		except Exception, error_msg:
			# Print the error message
			print(error_msg)
			# Exit the program
			exit(1)

		# Move the foot back
		target_tf = almath.Transform(current_tf)
		target_tf *= almath.Transform(-dx, 0, dz)
		target_tf *= almath.Transform(dwy)

		# Append the first chunk to the path
		path.append(list(target_tf.toVector()))

		# Move the foot forward
		target_tf = almath.Transform(current_tf)
		target_tf *= almath.Transform(dx, 0, dz)

		# Append the second chunk to the path
		path.append(list(target_tf.toVector()))

		# Append the third chunk to the path (move back to initial position)
		path.append(current_tf)

		# Return the path
		return path

	def kick(self):
		"""Define the procedure to follow to accomplish a kick"""

		# Initialize parameters
		axis_mask = 63  # Control translation and rotation
		effector = "RLeg" # We will kick with the right leg

		# Go to stand init posture
		self.posture.goToPosture("StandInit", 0.8)

		# Define both legs as fixed
		self.motion.wbFootState("Fixed", "Legs")
		# Constrain balance through leg motion
		self.motion.wbEnableBalanceConstraint(True, "Legs")

		# Define the left leg as support
		self.motion.wbGoToBalance("LLeg", 1)
		# Define the right leg as free
		self.motion.wbFootState("Free", "RLeg")

		# Compute the path
		path = self.compute_path(effector)

		# Execute the kicking movement
		self.motion.transformInterpolations(effector, self.motion.FRAME_WORLD, path, axis_mask, [2, 0.5, 2])
		# The last parameter correspond to the relative times for executing each path chunk

		# Go back to stand init posture
		self.posture("StandInit", 0.8)


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
