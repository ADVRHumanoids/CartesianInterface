# -*- coding: utf-8 -*-
"""
Created on Wed Nov 28 15:55:03 2018

@author: arturo
"""

from cartesian_interface.pyci_all import *

if __name__ == '__main__':

    ci = pyci.CartesianInterfaceRos()

    
    left_ee_ref  = Affine3(pos=[   1.183, -0.02465,    1.522], rot=[0.5817, 0.2247, 0.5606, 0.5447])
    right_ee_ref = Affine3(pos=[ 0.1477, -0.2796,   1.205], rot=[-0.001817,    0.3351,    0.7231,     0.604])

    ci.setTargetPose('arm1_8', left_ee_ref,  5.0)
    ci.setTargetPose('arm2_8', right_ee_ref, 5.0)    
    
    print 'Waiting for task to complete..'
    ci.waitReachCompleted('arm1_8')
    ci.waitReachCompleted('arm2_8')
    print 'Exiting...'