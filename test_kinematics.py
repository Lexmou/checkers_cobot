#!/usr/bin/env python
import numpy as np
import roslib
import rospy
import sys
from ur_kin_py.kin import Kinematics


MODE_DEBUG = False
MODE_REDUCE_FULL = True
NODE_NAME = "test_gripper"

def c_print(str, f = False):
    if(MODE_DEBUG == True and f == False):
        print(str)

def best_sol(sols, q_guess, weights):
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and 
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]

#Test la diff entre la solution attendue et connue et la meilleure des solutions de la cinematique
#Si ecart >0.001 pour tous les angles alors on affiche les valeurs
def test_q(q):
    global kin
    kin = Kinematics('ur10')
    x = kin.forward(q)
    sols = kin.inverse(np.array(x),q, test = True)
    qsol = best_sol(sols, q, [1.]*6)
    if qsol is None:
        qsol = [999.]*6
    diff = np.sum(np.abs(np.array(qsol) - q))
    if diff > 0.001:
        print np.array(sols)
        print 'Best q:', qsol
        print 'Actual:', np.array(q)
        print 'Diff:  ', q - qsol
        print 'Difdiv:', (q - qsol)/np.pi
        #print i1-3, i2-3, i3-3, i4-3, i5-3, i6-3
        if raw_input() == 'q':
            sys.exit()

def main():
    global MODE_DEBUG, MODE_REDUCE_FULL
    try:
	if(len(sys.argv)>1):
		if(sys.argv[1] == 'debug'):
        		MODE_DEBUG=True
	#print "MODE_DEBUG=" + str(MODE_DEBUG)
	if(len(sys.argv)>2):
		if(sys.argv[2] == 'full'):
        		MODE_REDUCE_FULL = False
        #print "Mode Debug Full ? : " + str(MODE_REDUCE_FULL)
	c_print(str("Instanciation Kinematic UR10"))
	kin = Kinematics('ur10')

        inp = raw_input("Run Script Test ? Y/N: ")[0]

	if (inp == 'Y' or inp =='y'):
	    np.set_printoptions(precision=3)
	    print("Testing multiples of pi/2...")
	    for i1 in range(0,5):
		for i2 in range(0,5):
		    c_print(str(i1) + str(i2), MODE_REDUCE_FULL)
		    for i3 in range(0,5):
		        for i4 in range(0,5):
		            for i5 in range(0,5):
		                for i6 in range(0,5):
		                    q = np.array([i1*np.pi/2., i2*np.pi/2., i3*np.pi/2., 
		                                  i4*np.pi/2., i5*np.pi/2., i6*np.pi/2.])
		                    test_q(q)
				    exit
	    print "Testing random configurations..."
	    for i in range(10000):
		q = (np.random.rand(6)-.5)*4*np.pi
		test_q(q)
	    print "Done!"	
	else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
if __name__ == '__main__': main()