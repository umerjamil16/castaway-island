#!/usr/bin/env python

from scipy.special import cbrt
import math
#Find cubic root of 27 & 64 using cbrt() function
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

    
if __name__ == '__main__':
    arr1 = [[1,2], [3,4], [5,6], [2, 4], [8,2]]
    total_dist = 0

  #  print arr1[0][0]
    for i in range(1, len(arr1)):
#        elem = arr1[i]
 #       print elem[0]
  #      print elem[1]
        dist_compute = distance(arr1[i-1], arr1[i])
        total_dist = total_dist + dist_compute
    

        #print value of cb
    print "total distance: " + str(total_dist)
    print "hello" 