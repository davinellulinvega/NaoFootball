#!/usr/bin/python
__author__ = 'davinellulinvega'

# Import the required packages
from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker
from optparse import OptionParser

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
		# Initialize a tracker module
		self.tracker = ALProxy("ALTracker")

	def initialize(self):
		"""Initialize the inner modules of the module"""

		# Wake the robot up
		self.motion.wakeUp()
		# Set the target name
		self.tracker.registerTarget("RedBall", 0.1)
		# Set the tracking mode
		self.tracker.setMode("Move")

	def shutdown(self):
		"""Shut the module down gracefully"""

		# Unregister all targets
		self.tracker.unregisterAllTargets()
		# Put the robot to rest
		self.motion.rest()


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
		# Track the target
		test_module.tracker.track("RedBall")
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
