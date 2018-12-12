# -*- coding: utf-8 -*-
"""
Created on Tue Nov 27 14:40:36 2018

@author: arturo
"""

from cartesian_interface.affine3 import Affine3

def get_pose_0123wa_stable():
    
    fl = Affine3(pos=[ 0.35,  0.35, -0.70])
    fr = Affine3(pos=[ 0.35, -0.35, -0.70])
    rr = Affine3(pos=[-0.35, -0.35, -0.70])
    rl = Affine3(pos=[-0.35,  0.35, -0.70])
    wa = Affine3(pos=[0.0, 0.0, 0.82], rot=[0, 0, 0, 1])
    
    return fl, fr, rr, rl, wa
    