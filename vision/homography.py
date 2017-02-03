# coding: utf-8

import cv2
import sys
import numpy as np

def rotate(A,B,C):
    return (B[0]-A[0])*(C[1]-B[1]) - (B[1]-A[1])*(C[0]-B[0])

def grahamscan(A):
    n = len(A) # number of points 
    P = range(n) # the list of point numbers 
    for i in range(1,n):
        if A[P[i]][0] < A[P[0]][0]:
            P[i], P[0] = P[0], P[i] # swap the numbers of these points 
    for i in range(2,n): # insertion sort 
        j = i
        while j > 1 and ( rotate(A[P[0]], A[P[j-1]], A[P[j]]) < 0 ): 
            P[j], P[j-1] = P[j-1], P[j]
            j -= 1
    S = [P[0], P[1]] # create the stack
    for i in range(2,n):
        while rotate( A[S[-2]], A[S[-1]], A[P[i]] ) < 0:
            del S[-1] # pop(S)
        S.append(P[i]) # push(S,P[i])
    return S

# Grahamscan and rotate from kukuruku.co, "Building a Minimal Convex Hull"


# return the list of the contours' barycentres
def centroids(liste):
    bary = []
    for contour in liste:
        moment = cv2.moments(contour)
        cx = int(moment['m10']/moment['m00'])
        cy = int(moment['m01']/moment['m00'])
        bary.append([cx, cy])
    convex = grahamscan(bary)
    res = []
    for p in convex:
        res.append(bary[p])

    return np.float32(res)


def homography(frame, masked):    
    outputs = cv2.blur(masked,(5,5))

    # detecting marks and extracting they centrois
    gray = cv2.cvtColor(outputs, cv2.COLOR_BGR2GRAY)
    (cnts, _) = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    centrois = centroids(cnts)

    for i in range(len(cnts)):
        cv2.drawContours(frame, cnts, i, (0,255,0), 3)

    if len(centrois) != 4: # we need 4 points
        print "nombre de barycentres trouvés: " + str(len(centrois))
        return -1

    print centrois
    M = cv2.getPerspectiveTransform(centrois, np.float32([[0,0], [0,500], [500,500], [500, 0]]) )
    dst = cv2.warpPerspective(masked, M, (500,500))
    
    return M