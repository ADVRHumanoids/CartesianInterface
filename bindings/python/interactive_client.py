#!/usr/bin/env ipython3 -i

from cartesian_interface.pyci_all import *
import numpy as np

ci = pyci.CartesianInterfaceRos()
tasks = ci.getTaskList()

for t in tasks:
    exec(t + '= ci.getTask(\'{}\')'.format(t))
