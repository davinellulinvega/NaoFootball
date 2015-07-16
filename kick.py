import argparse
import ALMath
from naoqi import ALProxy

def main(robotIP, PORT=9559):

	#Enable Balancer

	motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

	# Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Activate Whole Body Balancer
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    #Balance on left leg

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 2
    motionProxy.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motionProxy.wbFootState(stateName, supportLeg)

	#From parameters, calculate kick
		#LKneePitch = LKneePitch + x
		motionProxy.setAngles("RKneePitch", motionProxy.getAngles("RKneePitch")+20, 2)
		#LAnklePitch = LAnklePitch - x
		motionProxy.setAngles("RAnklePitch", motionProxy.getAngles("RAnklePitch")-20, 2)
		#getAngles() #setAngles()

	#PROFIT

	#Return to both legs (ALPosture, Stand Posture)

	postureProxy.goToPosture("StandInit", 0.5)

	#Disable balancer

	time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motionProxy.wbEnable(isEnabled)

    # send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.3)

    # Go to rest position
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.0.100",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)