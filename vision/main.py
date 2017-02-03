# coding: utf-8

import homography
import trackbars
import cv2
import numpy as np

DIMX, DIMY = 500, 500
BLACK = 0
WHITE = 1
NONE = -1

def makemap():
    mymap = list()
    for i in range(10):
        for j in range(10):
            p1 = [i*DIMX/10, j*DIMY/10]
            p2 = [(i+1)*DIMX/10, (j+1)*DIMY/10]

            if (-1)**(i+j) == 1 and i < 4:
                p3 = BLACK
            elif (-1)**(i+j) == 1 and i >= 6:
                p3 = WHITE
            else: 
                p3 = NONE

            mymap.append( (p1, p2, p3) )
    return mymap


def pun_played(map, centroids): #centroids of the two possible positions
    """ 
    determine when a move occurs, 
    where is the new pun
    and update the map of coordinates
    """
    res = map[:]
    K = 0
    p3temp = -3 #random value
    for i in centroids:
        x, y = i[0], i[1]
        for k in range(len(map)):
            p1, p2, p3 = map[k]
            x1, y1 = p1[0], p1[1]
            x2, y2 = p2[0], p2[1]

            if (x >= x1 and x <= x2) and (y >= y1 and y <= y2):
                if p3 != NONE:
                    p3temp = p3
                    p3 = - 1
                else:
                    K = k

    p1, p2, p3 = res[K]
    res[K] = (p1, p2, p3temp)
    return (res, K)


def centroids(image):
    mylist = []
    for i in range(0, len(image) - DIMX/10, DIMX/10):
        #print "i " +str(i)
        for j in range(0, len(image[0]) - DIMY/10, DIMY/10):
            #print "j " + str(j)
            (accx, accy, length) = (0, 0, 0)
            for m in range(i, i + DIMX/10):
                #print "m " + str(m)
                for n in range(j, j + DIMY/10):
                    if image[n][m] != 0:
                        accx += m
                        accy += n
                        length += 1
            if length != 0 and length >= 500:
                mylist.append([accx/length, accy/length])
    return mylist


if __name__ == '__main__':
    # color alibration
    masked = trackbars.trackbars()
    bool_homography = False

    video_capture = cv2.VideoCapture(0)
    ret, frame = video_capture.read()

    take_first_shot = False
    INITIAL_MAP = []

    count = 0 #used for map_before and map_now
    map_before =  []
    map_now = []
    map_now_cxy = [] #list of cases that contain points as delimiters

    while ret == True:
        if not bool_homography: 
            print "calibration des marqueurs"
            M = homography.homography(frame, masked)
            bool_homography = True
            
        while type(M) == type(-1):
            print "échec de la calibration"
            print "calibration des marqueurs"
            masked = trackbars.trackbars()
            M = homography.homography(frame, masked)
        
        ret, frame = video_capture.read()
        r = 700.0 / frame.shape[1]
        dim = (700, int(frame.shape[0] * r))
        frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA) 

        frame = cv2.warpPerspective(frame, M, (DIMX,DIMY))

        if take_first_shot == False:
            take_first_shot = True
            INITIAL_MAP = frame  
            map_before = INITIAL_MAP #initialization
            map_now_cxy = makemap() #initialization
        
        map_now = frame

        cv2.imwrite("initial_frame.png", frame)
        cv2.imshow("party", frame)

        if cv2.waitKey(1) & 0xFF == ord('m'): # a move happens
            map_before_gray = cv2.cvtColor(map_before, cv2.COLOR_BGR2GRAY)
            map_now_gray = cv2.cvtColor(map_now, cv2.COLOR_BGR2GRAY)

            map_before_gray = cv2.blur(map_before_gray,(5,5))
            map_now_gray = cv2.blur(map_now_gray,(5,5))

            diff = map_before_gray - map_now_gray #initialize the diff matrix
            cv2.absdiff(map_now_gray, map_before_gray, diff)
            max = np.max(diff)
            ret, diff_ = cv2.threshold(diff, max/3, 255, cv2.THRESH_BINARY)

            centr = centroids(diff_)
            # compute the move's position, regarding the centroids of errors between two snapshots
            map_now_cxy, pos = pun_played(map_now_cxy, centr) 

            print "nouvelle position du pion :"
            print map_now_cxy[pos]

            cv2.imwrite("map_before.png", map_before)
            cv2.imwrite("map_now.png", map_now)
            map_before = map_now


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        count += 1
    video_capture.release()
    cv2.destroyAllWindows()