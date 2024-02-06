import math

import numpy as np

from airsim_base.client import VehicleClient, MultirotorClient
from airsim_base.types import Pose, Vector3r, Quaternionr
from airsim_base.utils import to_quaternion, to_eularian_angles

#from cv_bridge import CvBridge, CvBridgeError

def set_client(client : str, ip = str) -> VehicleClient or MultirotorClient:
     if client == "computer_vision":
          return VehicleClient(ip)
     
     else: 
        return MultirotorClient(ip)

def angular_diference(current : float, to : float) -> float:
    heading= to - current
    if heading > math.pi:
        heading -= 2 * math.pi

    elif heading < -math.pi:
        heading += 2 * math.pi

    return heading 

def eularian_diference(vehicle_pose : Pose, target_position : Vector3r):
    vx, vy, vz = vehicle_pose.position
    vpitch, vroll, vyaw = to_eularian_angles(vehicle_pose.orientation)

    tx, ty, tz = target_position
    
    u = Vector3r(tx, ty, vz)
    v = Vector3r(tx, ty, tz)
    
    pitch = np.arccos(u.dot(v)/(u.get_length() * v.get_length()))
    pitch = angular_diference(vpitch, pitch)

    roll = np.arctan2(ty - vy, tz - vz)
    roll = angular_diference(vroll, roll)
    
    yaw = np.arctan2(ty - vy, tx - vx)
    yaw = angular_diference(vyaw, yaw)

    return pitch, roll, yaw

     
      

def angular2D_difference(orientation : Quaternionr, position : Vector3r, target_position : Vector3r) -> float:
    """Calculate the angular difference between origin and target position

    Args:
        position (tuple): (x, y, z) current position.
        target_position (tuple): (x, y, z) target's position.
        degrees: Define if radian's or degree's at result type 

    Returns:
        float: distance
    """    
    _, _, yaw = to_eularian_angles(orientation)
    x, y, _= position
    tx, ty, _= target_position

    to= np.arctan2(ty - y, tx - x)

    heading= to - yaw
    if heading > math.pi:
        heading -= 2 * math.pi

    elif heading < -math.pi:
        heading += 2 * math.pi

    return heading 

def angular3D_difference(vehicle_position : Vector3r, target_position : Vector3r) -> float:
    _, _, zv = vehicle_position
    xo, yo, zo = target_position
    
    u = Vector3r(xo, yo, zv)
    v = Vector3r(xo, yo, zo)
    
    pitch = np.arccos(u.dot(v)/(u.get_length() * v.get_length()))
    
    return pitch
    
def pose(position : tuple, eularian_orientation : tuple) -> Pose:
            """_summary_

            Args:
                position (tuple): position (x, y ,z).
                eularian_orientation (tuple): (roll, pitch, yaw).

            Returns:
                Pose: AirSim pose type.
            """        
            x, y, z = position
            pitch, roll, yaw =  np.deg2rad(eularian_orientation[0]),\
                                np.deg2rad(eularian_orientation[1]),\
                                np.deg2rad(eularian_orientation[2])
            pose_ = Pose()
            pose_.position.x_val = x
            pose_.position.y_val = y
            pose_.position.z_val = z
            
            pose_.orientation = to_quaternion(pitch, roll, yaw)
            
            return pose_

# def image_transport(img_msg):
#         rospy.logwarn(img_msg.header)
#         try:
#             return CvBridge().imgmsg_to_cv2(img_msg, "passthrough")

#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
            

