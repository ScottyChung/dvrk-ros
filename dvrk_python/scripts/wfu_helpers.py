import PyKDL
import numpy as np
import rospy

def vmove(arm, goal, speed, frame=PyKDL.Frame(), iterations=50):
        current_position = arm.get_current_position().p
        robot_goal = frame * goal
        distance =  robot_goal - current_position
        sleep_time = (distance/(speed*iterations)).Norm()

        #intermediate points
        interp_points = [np.linspace(s,e,iterations) for s,e in zip (current_position, robot_goal)]
        for x,y,z in zip(*interp_points):
            arm.move(PyKDL.Vector(x,y,z), blocking=False)
            rospy.sleep(sleep_time)

def test_matrix(arm,speed=0.05, frame=PyKDL.Frame()): 
    r = 0.05 #amplitude
    theta = np.repeat(np.linspace(0,np.pi/2,3),3) #must multiply to equal each other.
    phi = np.tile(np.linspace(0,np.pi/2,3),3)

    #calculating cartesian motions
    x = r*np.sin(theta)*np.cos(phi)
    y = r*np.sin(theta)*np.sin(phi)
    z = r*np.cos(theta)
    z_home = -0.1135
    for a,b,c in zip(x[2:],y[2:],z[2:]):
        print(a,b,c)
        vmove(arm, PyKDL.Vector(a,b,c+z_home),speed,frame)
        rospy.sleep(1)
        vmove(arm, PyKDL.Vector(0,0,z_home),speed,frame)
        rospy.sleep(1)
