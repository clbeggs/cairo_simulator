import pybullet as p
import cv2
import time
import rospy
import sys
import os 
import argparse
from cairo_simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator import Sawyer
from geometry_msgs.msg import Pose 
from pose_detection import Pose_From_Image
import numpy as np

def add_marker( pose ):
    """ 
        Add small_sphere marker, no mass and no collision.
        marker_sphere.urdf 
    """
    _ = SimObject( "marker" , "marker_sphere.urdf" , pose )
    
def add_object( obj_file , obj_name="obj" , pos_vec=None , orient_vec=None ):
    """ 
        Add object to sim env. 
    """
    if( orient_vec == () ):
        orient_vec = None
    if( pos_vec == () ):
       pos_vec = None
             
    _ = SimObject( obj_name, obj_file, pos_vec , orient_vec )
    

def main(args):
    """ Init. CairoSim Node """
    rospy.init_node("CAIRO_Sawyer_Simulator")
    #y = env.Env()
    #dim , im = y.measure_workspace( "./media/centered_ws2.jpg" )
    #z = y.define_sim_ws(np.array([0.9,0]))
    
    """ Create simulator and populate with objects """
    c_sim = Simulator() # Initialize the Simulator
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.9, 0, 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)
    

    """ Inverse Kinematics test """
    ik_solve_to = [0.9,0.1,0.7] #Where to solve to.

    add_marker((0.9,0,0.5)) #Center of table
    add_marker(tuple(ik_solve_to)) #marker for inverse kinematics
    time.sleep(5)
    
    joint_config = sawyer_robot.solve_inverse_kinematics( ik_solve_to , None)
    sawyer_robot.move_to_joint_pos(joint_config)
    
    time.sleep(10)

    p.disconnect()
    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Teleoperation via camera with Cairosim and openpose')
    parser.add_argument('--media' , nargs=1 , default=[] , help='Video or image location')
    
    args = parser.parse_args()
    
    main(args)



