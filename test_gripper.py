#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool
from wsg_50_common.msg import Cmd


MODE_DEBUG = False
MODE_REDUCE_FULL = True
NODE_NAME = "test_gripper"

def c_print(str, f = False):
    if(MODE_DEBUG == True and f == False):
        print(str)


#Si la pince est en mouvement, un premier message a True est reçu, une fois le moouvement terminé un message contenant False est envoyé.
#Attends la fin d'un mouvement de la pince
def verif_move_gripper():
	while True:
		b = rospy.wait_for_message("/wsg_50_driver/moving", Bool)
		cprint(str(b.data), MODE_REDUCE_FULL) 
	    	if(b.data == False):
			break

def main():
    try:
	if(len(sys.argv)>1):
		if(sys.argv[1] == 'debug'):
        		MODE_DEBUG=True
	#print "MODE_DEBUG=" + str(MODE_DEBUG)
	if(len(sys.argv)>2):
		if(sys.argv[2] == 'full'):
        		MODE_REDUCE_FULL = False
        #print "Mode Debug Full ? : " + str(MODE_REDUCE_FULL)
	c_print("Initialisation du Publisher /wsg_50_driver/goal_position", MODE_REDUCE_FULL)
	pubGripper = rospy.Publisher('/wsg_50_driver/goal_position',Cmd, queue_size=10)
	c_print(str("Initialisation du noeud :" + NODE_NAME))
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

        inp = raw_input("Run Script (Close 50mm, Open 80mm) ? Y/N: ")[0]

	if (inp == 'Y' or inp =='y'):
		cprint("close 50.0 4.0")
		pubGripper.publish("close", 50.0, 4.0)	
		verif_move_gripper()
		cprint("open 80.0 4.0")
		pubGripper.publish("open", 80.0, 4.0)	
		verif_move_gripper()	
	else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
if __name__ == '__main__': main()