import numpy as np
import modern_robotics as mr
import sys
__console__=sys.stdout

# print function
def printIterates(i, theta, Tsb, Vb, ew, ev):
    print(f"Iteration {i}:\n")
    print("Joint vector:", *theta,'\n')
    print("SE(3) end-effector config:", "  ".join(" ".join(str(cell) for cell in row) for row in Tsb), '\n')
    print("error twist V_b:", *Vb,'\n')
    print("angular error magnitude ||omega_b||：", ew, '\n')
    print("linear error magnitude ||v_b||：", ev)
    print("=================================================================================")

# modified IK function
def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot
       and report iteration number, joint angles, end-effector configuration, Twist,
       angular error, linear error at each iteration.
       The console ouput is logged into log.txt, joint angles at all iterations
       are stored in a matrix and logged into iterates.csv

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Tsb = mr.FKinBody(M, Blist, thetalist)
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
    omg_b_norm = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    omg_v_norm = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    
    err = omg_b_norm > eomg or omg_v_norm > ev
    
    thetamat = np.copy(thetalist)
    printIterates(i, thetalist, Tsb, Vb, omg_b_norm, omg_v_norm)
    
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        
        Tsb = mr.FKinBody(M, Blist, thetalist)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
        
        omg_b_norm = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        omg_v_norm = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        
        printIterates(i, thetalist, Tsb, Vb, omg_b_norm, omg_v_norm)
        
        # store current theta list to the theta matrix and 
        thetamat = np.row_stack((thetamat, thetalist))
        err = omg_b_norm > eomg or omg_v_norm > ev
        
    # after convergence wrtie the theta matrix into a csv file    
    np.savetxt("iterates.csv", thetamat, delimiter=',')
    
    return (thetalist, not err, thetamat)

# UR5 6R robot arm parameters

W1 = 109.0e-3
W2 = 82.0e-3
L1 = 425.0e-3
L2 = 392.0e-3
H1 = 89.0e-3
H2 = 95.0e-3

B1 = np.array([0,1,0,W1+W2,0,L1+L2]).T
B2 = np.array([0,0,1,H2,-L1-L2,0]).T
B3 = np.array([0,0,1,H2,-L2,0]).T
B4 = np.array([0,0,1,H2,0,0]).T
B5 = np.array([0,-1,0,-W2,0,0]).T
B6 = np.array([0,0,1,0,0,0]).T

B = np.array([B1, B2, B3, B4, B5, B6]).T

M = np.array([[-1,0,0,L1+L2],
              [0,0,1,W1+W2],
              [0,1,0,H1-H2],
              [0,0,0,1]])

theta0 = np.array([2.571, -0.879, 1.594, -0.293, -0.488, -2.115])

Tsd = np.array([[0, 1, 0, -0.5],
                [0, 0, -1, 0.1],
                [-1, 0, 0, 0.1],
                [0, 0, 0, 1]])

# tolerance [epsilon_omega, eplison_velocity]
epsi = np.array([0.001, 0.0001])

# write the Console output to a log file 'log.txt'
f = open('log.txt', 'wt')
sys.stdout = f

# call the modified IK function
[thetalist, success, thetamat] = IKinBodyIterates(B, M, Tsd, theta0, epsi[0], epsi[1])

sys.stdout=__console__

# check whether algorithm convergence
print(success)