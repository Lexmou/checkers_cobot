#!/usr/bin/env python
import numpy as np
import time as titi
import roslib;
import rospy
import actionlib
import operator
import test_kinematics as TestKin
from std_srvs.srv import Empty
from std_msgs.msg import String, Bool
from wsg_50_common.srv import Move
from wsg_50_common.msg import Cmd
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from ur_kin_py.kin import Kinematics
from math import pi, sqrt,cos, sin, acos, pow

MODE_DEBUG = False
MODE_REDUCE_FULL = True
NODE_NAME = "test_move"


Q1 = [-1.4089363257037562, -1.9426568190204065, -1.930267635975973, -0.8719747702227991, 1.5822780132293701, 0.2276924103498459]
Q2 = [-1.9145882765399378, -1.6094115416156214, -2.288931910191671, -0.872202221547262, 1.5822780132293701, 0.22765646874904633]
Q3 = [-0.7521079222308558, -1.6040228048907679, -2.2669156233416956, -0.8775017897235315, 1.5839924812316895, 0.22839948534965515]


Q4 =[-1.6464884916888636, -1.6402900854693812, -1.9309981505023401, -1.1182931105243128, 1.582206130027771, 0.22770440578460693]
Q5 =[-1.2603300253497522, -1.0286968390094202, -2.2685559431659144, -1.2694934050189417, 1.5822420120239258, 0.22764447331428528]
Q6 =[-1.074174706135885, -1.6677449385272425, -1.8674176375018519, -1.2822840849505823, 1.584004521369934, 0.2283635288476944]

client = None
time = 6.0   
#Cette fonction parcourt les 6 points du dessus.
#Q1,Q2,Q3 sont proche du plan base, Q4, Q5, Q6 entre chaque comme position intermediaire

def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = time
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(d))]
        for i in range(20):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
	    c_print("Position Q1 " + str([Q1[i] for i in xrange(0,6)]), MODE_REDUCE_FULL) 
            d += time
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(d)))
	    c_print("Position Q4 " + str([Q4[i] for i in xrange(0,6)]), MODE_REDUCE_FULL) 
            d += time
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
	    c_print("Position Q2 " + str([Q2[i] for i in xrange(0,6)]), MODE_REDUCE_FULL) 
	    d += time + 2 
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(d)))
	    c_print("Position Q5 " + str([Q5[i] for i in xrange(0,6)]), MODE_REDUCE_FULL)
            d += time +2
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
	    c_print("Position Q3 " + str([Q5[i] for i in xrange(0,6)]), MODE_REDUCE_FULL)
	    d += time
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(d)))
	    c_print("Position Q6 " + str([Q6[i] for i in xrange(0,6)]), MODE_REDUCE_FULL)
            d += time
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise


#Parametrisation Sphere a entre 0 et 2PI et b entre 0 et PI
def sphere_cart(a, b, r):
	x = r*cos(a)*sin(b)
	y = r*sin(a)*sin(b)
	z = r*cos(b)
	if False:
            print 'x', x
            print 'y', y
            print 'z', z
	return [x,y,z]



def move_circle(p1, p2, step):
	#print "p1 " , p1, "p2 " , p2
	pC = [(p1[0,3] + p2[0,3])/2, (p1[1,3] + p2[1,3])/2, (p1[2,3] + p2[2,3])/2]
	#xC = (p1[0] + p2[0])/2
	#yC = (p1[1] + p2[1])/2
	#zC = (p1[2] + p2[2])/2
	m = pow((p1[0,3] - pC[0]),2) + pow((p1[1,3] - pC[1]),2) + pow((p1[2,3] - pC[2]),2)
	#print pC
	demiMedian = sqrt(m)
	#print (p1[1] - pC[1])	
	print p2[0,3],pC[0], demiMedian, (p2[0,3] - pC[0])/demiMedian
	a = acos((p1[0,3] - pC[0])/demiMedian)
	i = np.pi / step
	#k = acos((p1[2,3] - pC[2])/demiMedian)
	#print "k" , k
	k =0
	tabResult = []
	for j in range(0, step +1):
		tmp = p1.copy()
		print "a", a, "b", k
		xRel = sphere_cart(a, k , demiMedian)
		#print "xRel", xRel
		tmp[0,3] = xRel[0] + pC[0]	
		tmp[1,3] = - xRel[2] + pC[1]
		tmp[2,3] = xRel[1] + pC[2]
		tabResult.append(tmp)
		k += i
        print tmp
	return tabResult


#Traitement en temps reel de la trajectoire. Pour obtenir du temps réel il faut separer la boucle d envoi de la boucle de calcul pour conserver la fréquence 
def test_incr(trans, M, kin, pub, pubGripper, n):
	    otherM = M.copy()
	    otherM[0,3] += trans[0]
	    otherM[1,3] += trans[1]
	    otherM[2,3] += trans[2]
	    c_print("Actual Pose : " + str(M), MODE_REDUCE_FULL)
	    c_print("Next Pose : " + str(otherM), MODE_REDUCE_FULL)	    
	    results = move_circle(M, otherM, n)
	    sends = []
	    for result in results:
            	joint_states = rospy.wait_for_message("joint_states", JointState)
            	joints_pos = joint_states.position
		sol = kin.inverse(result,joints_pos)
		tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    	sends.append("servoj("+tmp+", t =0.050, gain=200)")
		#pub.publish(send) Fonctionne aussi ici mais si le calcul est trop long ça tremble
	    for send in sends:
            	titi.sleep(0.025) #frequence d'envoi, minimum 0.024
	    	pub.publish(send)


#Mode Debug
   
def c_print(str, f = False):
    if(MODE_DEBUG == True and f == False):
        print(str)




def verif_move(sol):
	while True:
		joint_states = rospy.wait_for_message("joint_states", JointState)
	    	joints_pos = joint_states.position
		print "sol", sol , "pos", joints_pos
		L = map(abs,list(map(operator.sub, sol, joints_pos)))
		s = sum(L)
		print s
		titi.sleep(0.400)
		if s < 0.001:
			break   

def verif_move_gripper():
	while True:
		b = rospy.wait_for_message("/wsg_50_driver/moving", Bool)  
		print "b", b.data
	    	if(b.data == False):
			break
		
width = 0.036


# right => newx- x>0
def move_right(x):
	x[0,3] += width

def move_left(x):
	x[0,3] -= width
def move_top(x):
	x[1,3] += width

def move_down(x):
	x[1,3] -= width

def move_diag_right_top(x):
	move_right(x)
	move_top(x)
def move_diag_right_down(x):
	move_right(x)
	move_down(x)
def move_diag_left_top(x):
	move_left(x)
	move_top(x)
def move_diag_left_down(x):
	move_left(x)
	move_down(x)

def move_take(x):
	x[2,3] -= 0.10
def move_leave(x):
	x[2,3] += 0.10
def move_kill(x):
	x[0,3] -= 0.30


#M est la matrice 4x4 effecteur.   x y z en m
#A utiliser quand on se sert de Kinematics
def set_TCP(x,y,z,M):
	M[0,3] += x
	M[1,3] += y
	M[2,3] += z
def main():
    global client, MODE_DEBUG, MODE_REDUCE_FULL
    try:
	if(len(sys.argv)>1):
		if(sys.argv[1] == 'debug'):
        		MODE_DEBUG=True
	print "MODE_DEBUG=" + str(MODE_DEBUG)
	if(len(sys.argv)>2):
		if(sys.argv[2] == 'full'):
        		MODE_REDUCE_FULL = False
        print "Mode Debug Full ? : " + str(MODE_REDUCE_FULL)
	c_print("Initialisation du Publisher /ur_driver/URScript", MODE_REDUCE_FULL)
	pub = rospy.Publisher('/ur_driver/URScript', String,queue_size=10)
	c_print("Initialisation du Publisher /wsg_50_driver/goal_position", MODE_REDUCE_FULL)
	pubGripper = rospy.Publisher('/wsg_50_driver/goal_position',Cmd, queue_size=10)
	c_print(str("Initialisation du noeud :" + NODE_NAME), MODE_REDUCE_FULL)
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

        print "This program makes the robot move, choose a mode:"
        print "     - Mode ROS (ROS FollowJointTrajectoryAction) : R"
        print "     - Mode URSCRIPT (movej, servoj) : U"
	print "     - Demo : D"
	print "     - Kinematics : K"

        inp = raw_input("Mode? R/U/D/K: ")[0]
        if (inp == 'R'):
            client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            c_print("Waiting for server...", MODE_REDUCE_FULL)
            client.wait_for_server()
            c_print("Connected to server")
            #move1()
            move_repeated()
            #move_disordered()
            #move_interrupt()
        elif(inp == 'U'):
	    c_print(str("Instanciation Kinematic UR10"), MODE_REDUCE_FULL)
	    kin = Kinematics('ur10')
	    c_print(str("Attente d'un message de JointState"), MODE_REDUCE_FULL)
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
	    c_print("JointState.position: " + str(tuple(joints_pos)), MODE_REDUCE_FULL)
            x = kin.forward(np.array(joints_pos))
	    c_print(str("Declaration point de depart du bras"), True)
	    c_print("First Pose " + str(x))	
	    #Dessine un arc de cercle pour une translation de 10cm
	    test_incr([0.00, 0.30, 0.00],x, kin, pub, pubGripper, 300)
        elif(inp == 'D'):
	    pubGripper.publish("open", 70.0, 4.0)
	    verif_move_gripper()		   	
	    c_print(str("Instanciation Kinematic UR10"), MODE_REDUCE_FULL)
	    kin = Kinematics('ur10')
	    c_print(str("Attente d'un message de JointState"), MODE_REDUCE_FULL)
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
	    c_print("JointState.position: " + str(tuple(joints_pos)), MODE_REDUCE_FULL)
            x = kin.forward(np.array(joints_pos))
	    c_print(str("Placement pour Pion 1"), True)
	    x=[[ 0.00, -1.00,  0.00,  0.00],
 	       [0.00, 0.00,  1.00,  0.38],
 	       [-1.00, 0.00, 0.00, 0.19],
 	       [ 0., 0.,  0.,  1. ]]
	    c_print("First Pose " + str(x))	
	    #print "Modif : ",x
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)	
	    pubGripper.publish("close",34.8, 4.0);
	    verif_move_gripper()

	    c_print(str("Deplacement pour Pion 1"), True)
	    move_diag_right_top(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("open",50.0, 4.0);
	    verif_move_gripper()

	    c_print(str("Position temporaire"), True)
	    move_leave(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("open",60.0, 4.0);
	    verif_move_gripper()

	    c_print(str("Placement pour Pion 2"), True)
	    move_left(x)
	    move_left(x)
	    move_top(x)
	    move_top(x)
	    move_take(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("close",34.8, 4.0);
	    verif_move_gripper()

	    c_print(str("Deplacement pour Pion 2"), True)
	    move_diag_right_down(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("open",50.0, 4.0);
	    verif_move_gripper()

	    c_print(str("Position temporaire"), True)
	    move_leave(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    print send
	    pub.publish(send)
	    verif_move(sol)

	    c_print(str("Placement Pion 1 mange Pion2"), True)
	    move_diag_right_down(x)
	    move_take(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("close",34.8, 4.0);
	    verif_move_gripper()

	    c_print(str("Position temporaire"), True)
	    move_leave(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)


	    c_print(str("Placement pour manger Pion 2"), True)
	    move_left(x)
	    move_left(x)
	    move_top(x)
	    move_top(x)
	    move_take(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("open",50.0, 4.0);
	    verif_move_gripper()

	    c_print(str("Position temporaire"), True)
	    move_leave(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)

	    c_print(str("Pion qui meurt"), True)
	    move_diag_right_down(x)
	    move_take(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("close", 34.8 , 4.0);
	    verif_move_gripper()

	    c_print(str("Position temporaire"), True)
	    move_leave(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)

	    c_print(str("Pion 2 mort"), True)
	    move_kill(x)
	    move_take(x)
	    sol = kin.inverse(x,joints_pos)
	    tmp =  np.array2string(sol, precision=8, separator=',',suppress_small=True)
	    send = "movej("+tmp+",t=5.0)"
	    c_print(send, MODE_REDUCE_FULL)
	    pub.publish(send)
	    verif_move(sol)
	    pubGripper.publish("open", 50.0 , 4.0);
	    verif_move_gripper()
        elif(inp == 'K'):
		TestKin.main()
	else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()