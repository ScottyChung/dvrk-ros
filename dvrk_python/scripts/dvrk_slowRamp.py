import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import numpy as np
import time
import wfu_helperss


k = dvrk.psm('PSM2')
l = k.get_current_position().p
print(l)
l[2] = l[2] + 0.05
print(l)
wfu_helperss.vmove(k,l,0.05)
