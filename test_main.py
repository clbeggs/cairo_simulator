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
import env
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
    add_marker((0.9,0,0.5))
    add_marker((0.9,0.1,0.7))
    time.sleep(5)
    
    joint_config = sawyer_robot.solve_inverse_kinematics([0.9,0.1,0.7], None)
    sawyer_robot.move_to_joint_pos(joint_config)
    
    #print("HERE")
    #for i in z:
    #    print(i)
    #    add_marker(i)
    #t = lambda x : tuple(x) if(isinstance(x,list)) else x #Convert input to tuple if list. 

    time.sleep(10)

    joint_angs = np.array([0.4493345488433066, -0.15264013369292084, 0.5246021257607938, -0.622124537542545, -0.29306412321553604, -1.93889201589517, -0.15973077175720515, -2.2767884734182654e-08, -0.02083300000000173])

    sawyer_robot.move_to_joint_pos(joint_angs)
    return
    while rospy.is_shutdown() is not True:
        c_sim.step()
        a= sawyer_robot.get_current_joint_states()
        print(a)
        rospy.sleep(0.8)
    p.disconnect()



def test():
    stuff = []
    s = Pose_From_Image()

    kp_list = s.get_body_pose_mapping()
    for i in range(1,5):
        r = s.image("./media/ava_scene_frames/ezgif-frame-00{}.jpg".format(i))
        stuff.append(r)
    
    kp = stuff[0].poseKeypoints #each keypoint is (x , y  , confidence score )
    img = stuff[0].cvOutputData
    for i , body_part in enumerate(kp[0]):
        print("{} - {}\n".format(kp_list[i] , body_part))
        print("({},{})".format(body_part[0] , body_part[1]))
        cv2.circle(img , (body_part[0] , body_part[1]) , 10 , (0, 255, 0) )
        break
        
    cv2.imshow("S" , img) 
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    
    
def test2():
    y = env.Env()
    dim , im = y.measure_workspace( "./media/centered_ws2.jpg" )
    z = y.define_sim_ws(np.array([0,0.9]))
    

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Teleoperation via camera with Cairosim and openpose')
    parser.add_argument('--media' , nargs=1 , default=[] , help='Video or image location')
    
    args = parser.parse_args()
    
    #if( args.media == [] ):
      #  print("No media passed in, use \"--media\" to point to location of video" )
    main(args)
    #test2()





""" 
Datum Class Attributes:

['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', 
'__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
 '__init__', '__init_subclass__', '__le__', '__lt__', '__module__',
  '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', 
  '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 
  'cameraExtrinsics', 'cameraIntrinsics', 'cameraMatrix', 'cvInputData', 
  'cvOutputData', 'cvOutputData3D', 'elementRendered', 'faceHeatMaps',
   'faceKeypoints', 'faceKeypoints3D', 'faceRectangles', 'frameNumber', 
   'handHeatMaps', 'handKeypoints', 'handKeypoints3D', 'handRectangles', 
   'id', 'inputNetData', 'name', 'netInputSizes', 'netOutputSize', 
   'outputData', 'poseCandidates', 'poseHeatMaps', 'poseIds', 'poseKeypoints',
    'poseKeypoints3D', 'poseNetOutput', 'poseScores', 'scaleInputToNetInputs', 
    'scaleInputToOutput', 'scaleNetToOutput', 'subId', 'subIdMax']



"""