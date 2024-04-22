import copy
import cv2 
import numpy as np
import mediapipe as mp

import utils
import matplotlib.pyplot as plt

import time
import yarp

##################################################################
############### POSE ESTIMATION USING MPOSE ######################
##################################################################

   
landmark_index_dict = {0: 'nose', 
                      1: 'l_eye_inner', 
                      2: 'l_eye', 
                      3: 'l_eye_outer', 
                      4: 'r_eye _inner', 
                      5: 'r_eye', 
                      6: 'r_eye_outer', 
                      7: 'r_ear', 
                      8: 'r_ear', 
                      9: 'mouth_left', 
                      10: 'mouth_right', 
                      11: 'l_shoulder', 
                      12: 'r_shoulder', 
                      13: 'l_elbow', 
                      14: 'r_elbow', 
                      15: 'l_wrist', 
                      16: 'r_wrist', 
                      17: 'l_pinky', 
                      18: 'r_pinky', 
                      19: 'l_index', 
                      20: 'r_index', 
                      21: 'l_thumb', 
                      22: 'r_thumb', 
                      23: 'l_hip', 
                      24: 'r_hip', 
                      25: 'l_knee', 
                      26: 'r_knee', 
                      27: 'l_ankle', 
                      28: 'r_ankle', 
                      29: 'l_heel', 
                      30: 'r_heel', 
                      31: 'l_foot_index', 
                      32: 'r_foot_index'}
                      
# For robot control: 15 keypoints required
kp_to_save = ['l_hip', 'l_knee', 'l_ankle', 'l_foot_index',
            'r_hip', 'r_knee', 'r_ankle', 'r_foot_index',
            'l_shoulder', 'l_elbow', 'l_wrist',
            'r_shoulder', 'r_elbow', 'r_wrist',
            'nose']

keypoints_to_index = {'nose': 0, 
                    'l_shoulder': 1, 'l_elbow': 3, 'l_wrist': 5, 
                    'r_shoulder': 2, 'r_elbow': 4, 'r_wrist': 6,  
                    'l_hip': 7, 'l_knee': 9, 'l_ankle': 11, 'l_foot_index': 13,
                    'r_hip': 8, 'r_knee': 10, 'r_ankle': 12, 'r_foot_index': 14}

def gen_landmark_list(result_pose_landmark, save_joints):
    """ saving only keypoints from save_joints list """
    pose_list = []

    for i in range(len(result_pose_landmark.landmark)):
        if landmark_index_dict[i] in save_joints:
            data_point = result_pose_landmark.landmark[i]
            pose_list.append(data_point.x)
            pose_list.append(data_point.y)
            pose_list.append(data_point.z)
    output = np.reshape(np.array(pose_list), (len(kp_to_save),-1))
    return output

def convert_to_dictionary(kpts, keypoints_to_index):
    kpts_dict = {}
    for key, k_index in keypoints_to_index.items():
        kpts_dict[key] = kpts[k_index]
    kpts_dict['joints'] = list(keypoints_to_index.keys())
    return kpts_dict

def add_hips_and_neck(kpts):
    # we add two new keypoints which are the mid point between the hips and mid point between the shoulders
    # add hips kpts
    difference = kpts['l_hip'] - kpts['r_hip']
    difference = difference/2
    hips = kpts['r_hip'] + difference
    kpts['hips'] = hips
    kpts['joints'].append('hips')

    #add neck kpts 
    difference = kpts['l_shoulder'] - kpts['r_shoulder']
    difference = difference/2
    neck = kpts['r_shoulder'] + difference
    kpts['neck'] = neck
    kpts['joints'].append('neck')

    #add T8 kpts (torso)
    difference = kpts['neck'] - kpts['hips']
    difference = difference/2
    T8 = kpts['hips'] + difference
    kpts['T8'] = T8
    kpts['joints'].append('T8')

    #define the hierarchy of the joints
    hierarchy = {'hips': [],
                 'l_hip': ['hips'], 'l_knee': ['l_hip', 'hips'], 'l_ankle': ['l_knee', 'l_hip', 'hips'], 'l_foot_index': ['l_ankle', 'l_knee', 'l_hip', 'hips'],
                 'r_hip': ['hips'], 'r_knee': ['r_hip', 'hips'], 'r_ankle': ['r_knee', 'r_hip', 'hips'], 'r_foot_index': ['r_ankle', 'r_knee', 'r_hip', 'hips'],
                 'T8':['hips'], 'neck': ['T8', 'hips'], 'nose': ['neck', 'T8', 'hips'],
                 'l_shoulder': ['neck', 'T8', 'hips'], 'l_elbow': ['l_shoulder', 'neck', 'T8', 'hips'], 'l_wrist': ['l_elbow', 'l_shoulder', 'neck', 'T8', 'hips'],
                 'r_shoulder': ['neck', 'T8', 'hips'], 'r_elbow': ['r_shoulder', 'neck', 'T8', 'hips'], 'r_wrist': ['r_elbow', 'r_shoulder', 'neck', 'T8', 'hips']
                }

    kpts['hierarchy'] = hierarchy
    kpts['root_joint'] = 'hips'
    kpts['ang_history'] = {key: [] for key in kpts['joints']}    # for calculating angular velocity
    kpts['pos_history'] = {key: [] for key in kpts['joints']}    # for calculating linear velocity
    return kpts

def get_bone_lengths(kpts):

    """
    We have to define an initial skeleton pose(T pose).
    In this case we need to known the length of each bone.
    Here we calculate the length of each bone from data
    """

    bone_lengths = {}
    for joint in kpts['joints']:
        if joint == 'hips': continue
        parent = kpts['hierarchy'][joint][0]

        joint_kpts = kpts[joint]
        parent_kpts = kpts[parent]

        _bone = joint_kpts - parent_kpts
        _bone_lengths = np.sqrt(np.sum(np.square(_bone), axis = -1))

        _bone_length = np.median(_bone_lengths)
        bone_lengths[joint] = _bone_length

    kpts['bone_lengths'] = bone_lengths
    return

#Here we define the T pose and we normalize the T pose by the length of the hips to neck distance.
def get_base_skeleton(kpts, normalization_bone = 'neck'):

    #this defines a generic skeleton to which we can apply rotations to
    body_lengths = kpts['bone_lengths']

    #define skeleton offset directions
    offset_directions = {}
    offset_directions['l_hip']          = np.array([0, 1, 0])
    offset_directions['l_knee']         = np.array([0, 0,-1])
    offset_directions['l_ankle']        = np.array([0, 0,-1])
    offset_directions['l_foot_index']   = np.array([1, 0,0])

    offset_directions['r_hip']          = np.array([0,-1, 0])
    offset_directions['r_knee']         = np.array([0, 0,-1])
    offset_directions['r_ankle']        = np.array([0, 0,-1])
    offset_directions['r_foot_index']   = np.array([1, 0,0])

    offset_directions['neck']           = np.array([0, 0, 1])
    offset_directions['nose']           = np.array([0, 0, 1])
    offset_directions['T8']             = np.array([0, 0, 1])

    offset_directions['l_elbow']        = np.array([0, 1, 0])
    offset_directions['l_shoulder']     = np.array([0, 1, 0])
    offset_directions['l_wrist']        = np.array([0, 1, 0])

    offset_directions['r_shoulder']     = np.array([0,-1, 0])
    offset_directions['r_elbow']        = np.array([0,-1, 0])
    offset_directions['r_wrist']        = np.array([0,-1, 0])

    #set bone normalization length. Set to 1 if you dont want normalization
    normalization = kpts['bone_lengths'][normalization_bone]

    #base skeleton set by multiplying offset directions by measured bone lengths. In this case we use the average of two sided limbs. E.g left and right hip averaged
    base_skeleton = {'hips': np.array([0,0,0])}
    def _set_length(joint_type):
        base_skeleton['l_' + joint_type] = offset_directions['l_' + joint_type] * ((body_lengths['l_' + joint_type] + body_lengths['r_' + joint_type])/(2 * normalization))
        base_skeleton['r_' + joint_type] = offset_directions['r_' + joint_type] * ((body_lengths['l_' + joint_type] + body_lengths['r_' + joint_type])/(2 * normalization))

    _set_length('hip')
    _set_length('knee')
    _set_length('ankle')
    _set_length('foot_index')
    _set_length('shoulder')
    _set_length('elbow')
    _set_length('wrist')

    base_skeleton['neck'] = offset_directions['neck'] * (body_lengths['neck']/normalization)
    base_skeleton['T8'] = offset_directions['T8'] * (body_lengths['T8']/normalization)
    base_skeleton['nose'] = offset_directions['nose'] * (body_lengths['nose']/normalization)

    kpts['offset_directions'] = offset_directions
    kpts['base_skeleton'] = base_skeleton
    kpts['normalization'] = normalization

    return

#calculate the rotation of the root joint with respect to the world coordinates
def get_hips_position_and_rotation(frame_pos, root_joint = 'hips', root_define_joints = ['l_hip', 'neck']):

    #root position is saved directly
    root_position = frame_pos[root_joint]

    #calculate unit vectors of root joint
    """
    root_u = frame_pos[root_define_joints[0]] - frame_pos[root_joint]
    root_u = root_u/np.sqrt(np.sum(np.square(root_u)))
    root_v = frame_pos[root_define_joints[1]] - frame_pos[root_joint]
    root_v = root_v/np.sqrt(np.sum(np.square(root_v)))
    root_w = np.cross(root_u, root_v)
    """
    root_w = frame_pos[root_define_joints[1]] - frame_pos[root_joint]
    root_w = root_w/np.sqrt(np.sum(np.square(root_w)))
    root_v = frame_pos[root_define_joints[0]] - frame_pos[root_joint]
    root_v = root_v/np.sqrt(np.sum(np.square(root_v)))
    root_u = -np.cross(root_w, root_v)

    #Make the rotation matrix
    C = np.array([root_u, root_v, root_w]).T
    thetaz,thetay, thetax = utils.Decompose_R_ZXY(C)
    root_rotation = np.array([thetaz, thetax, thetay])

    return root_position, root_rotation

#calculate the rotation matrix and joint angles input joint
def get_joint_rotations(joint_name, joints_hierarchy, joints_offsets, frame_rotations, frame_pos):

    _invR = np.eye(3)
    for i, parent_name in enumerate(joints_hierarchy[joint_name]):
        if i == 0: continue
        _r_angles = frame_rotations[parent_name]
        R = utils.get_R_z(_r_angles[0]) @ utils.get_R_x(_r_angles[1]) @ utils.get_R_y(_r_angles[2])
        _invR = _invR@R.T

    b = _invR @ (frame_pos[joint_name] - frame_pos[joints_hierarchy[joint_name][0]])

    _R = utils.Get_R2(joints_offsets[joint_name], b)
    tz, ty, tx = utils.Decompose_R_ZXY(_R)
    joint_rs = np.array([tz, tx, ty])
    #print(np.degrees(joint_rs))

    return joint_rs

#helper function that composes a chain of rotation matrices
def get_rotation_chain(joint, hierarchy, frame_rotations):

    hierarchy = hierarchy[::-1]

    #this code assumes ZXY rotation order
    R = np.eye(3)
    for parent in hierarchy:
        angles = frame_rotations[parent]
        #print(parent, angles)
        _R = utils.get_R_z(angles[0])@utils.get_R_x(angles[1])@utils.get_R_y(angles[2])
        R = R @ _R

    return R


def anglesToQuaternion(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) # scalar/real part
  return [qw, qx, qy, qz]

#calculate the joint angles frame by frame.
def calculate_joint_angles(kpts, out_dict, enc_list):

    #set up emtpy container for joint angles
    for joint in kpts['joints']:
        kpts[joint+'_angles'] = []

    #get the keypoints positions in the current frame
    frame_pos = {}
    for joint in kpts['joints']:
        frame_pos[joint] = kpts[joint]

    root_position, root_rotation = get_hips_position_and_rotation(frame_pos)

    frame_rotations = {'hips': root_rotation}

    #center the body pose
    for joint in kpts['joints']:
        frame_pos[joint] = frame_pos[joint] - root_position

    #get the max joints connectsion
    max_connected_joints = 0
    for joint in kpts['joints']:
        if len(kpts['hierarchy'][joint]) > max_connected_joints:
            max_connected_joints = len(kpts['hierarchy'][joint])

    depth = 2
    while(depth <= max_connected_joints):
        for joint in kpts['joints']:
            if len(kpts['hierarchy'][joint]) == depth:
                joint_rs = get_joint_rotations(joint, kpts['hierarchy'], kpts['offset_directions'], frame_rotations, frame_pos)
                parent = kpts['hierarchy'][joint][0]
                frame_rotations[parent] = joint_rs
        depth += 1

    #for completeness, add zero rotation angles for endpoints. This is not necessary as they are never used.
    for _j in kpts['joints']:
        if _j not in list(frame_rotations.keys()):
            frame_rotations[_j] = np.array([0.,0.,0.])

    #update dictionary with current angles.
    for joint in kpts['joints']:
        kpts[joint + '_angles'].append(frame_rotations[joint])

    #convert joint angles list to numpy arrays.
    file_string = ''
    for joint in kpts['joints']:
        kpts[joint+'_angles'] = np.array(kpts[joint + '_angles'][0])
        #kpts[joint+'_quaternion'] = anglesToQuaternion(kpts[joint+'_angles'][0], kpts[joint+'_angles'][1], kpts[joint+'_angles'][2])
        roll, pitch, yaw = kpts[joint+'_angles'][1], kpts[joint+'_angles'][2], kpts[joint+'_angles'][0]
        qua = anglesToQuaternion(roll, pitch, yaw) #tz, tx, ty
        output_dict[joint]['orientation'] = [qua[0], qua[1], qua[2], qua[3]]
        output_dict[joint]['vel_lin'] = [0.0, 0.0, 0.0]
        output_dict[joint]['vel_ang'] = [0.0, 0.0, 0.0]

    for enc in enc_list: 
        roll, pitch, yaw = kpts[enc+'_angles'][1], kpts[enc+'_angles'][2], kpts[enc+'_angles'][0]
        file_string += f' {roll} {pitch} {yaw}'

    return file_string

def calculate_joint_veloc(kpts):
    """ calculates joint's linear and angular velocity """
    for joint in kpts['joints']:
        kpts[joint+'_ang_vel'] = []
        kpts[joint+'_lin_vel'] = []
    return 
    
def draw_skeleton_from_joint_angles(kpts):
    # Create a Figure and 3D Axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
        
    # Get a dictionary containing the rotations for the current frame
    frame_rotations = {}
    for joint in kpts['joints']:
        frame_rotations[joint] = kpts[joint+'_angles']

    # For plotting
    for _j in kpts['joints']:
        if _j == 'hips':
            continue

        # Get hierarchy of how the joint connects back to the root joint
        hierarchy = kpts['hierarchy'][_j]
        
        # Get the current position of the parent joint
        r1 = kpts['hips'] / kpts['normalization']
        for parent in hierarchy:
            if parent == 'hips':
                continue
            R = get_rotation_chain(parent, kpts['hierarchy'][parent], frame_rotations)
            r1 = r1 + R @ kpts['base_skeleton'][parent]

        # Get the current position of the joint. Note: r2 is the final position of the joint. r1 is simply calculated for plotting.
        r2 = r1 + get_rotation_chain(hierarchy[0], hierarchy, frame_rotations) @ kpts['base_skeleton'][_j]
        ax.plot(xs=[r1[0], r2[0]], ys=[r1[1], r2[1]], zs=[r1[2], r2[2]], color='red')

    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.azim = 90
    ax.elev = -85
    ax.set_title('Pose from joint angles')
    ax.set_xlim3d(-4, 4)
    ax.set_xlabel('x')
    ax.set_ylim3d(-4, 4)
    ax.set_ylabel('y')
    ax.set_zlim3d(-4, 4)
    ax.set_zlabel('z')
    
    return fig


def fill_output_dict(values, output_dict, key_value, keynames):
    for i in keynames:
        if output_dict.get(i) is None:
            output_dict[i] = {key_value: values[i]}
        else: 
            output_dict[i][key_value] = values[i]
    return output_dict


def dict_to_string(out_dict):
    msg = []
    order = ['hips', 'T8', 'nose',
             'r_shoulder', 'r_elbow', 'r_wrist',  
             'l_shoulder', 'l_elbow', 'l_wrist', 
             'r_hip', 'r_knee', 'r_ankle',
             'l_hip', 'l_knee', 'l_ankle']
    
    for target in order:
        pos = output_dict[target]['position']
        orientation = output_dict[target]['orientation']
        v_lineal = output_dict[target]['vel_lin']
        v_ang = output_dict[target]['vel_ang']
        msg = msg + [pos[0], pos[1], pos[2], v_lineal[0], v_lineal[1], v_lineal[2], v_ang[0], v_ang[1], v_ang[2], orientation[0], orientation[1], orientation[2], orientation[3]]
    
    return np.array(msg)

def send_yarp_msg(msg, output_port):
    """ send message as float through yarp port """
    # Connect to the destination port
    message = yarp.Bottle()
    message.clear()
    for i in msg:
    	message.addFloat32(i)
    # Send the message through the output port
    output_port.write(message)

def send_yarp_str(msg, output_port):
    """ send message as string through yarp port """
    # Connect to the destination port
    message = yarp.Bottle()
    message.clear()
    message.addString(msg)
    #message.addString(msg)
    # Send the message through the output port
    output_port.write(message)

if __name__ == '__main__':

    # initiates data port 
    yarp.Network.init() 
    output_port = yarp.Port()
    output_port.open("/wport")
    yarp.Network.connect("/wport", "/dataPort")

    ang_port = yarp.Port()
    ang_port.open("/w_ang_port")
    yarp.Network.connect("/w_ang_port", "/r_ang_port")
    print('PUERTOS INICIADOS')
    time.sleep(6)

    # Initiates camera 
    cap = cv2.VideoCapture(2)     #configure depending on your device
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    fps = cap.get(cv2.CAP_PROP_FPS)

    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5)

    R = utils.get_R_z(np.pi)
    
    # Initiates velocity historial
    start = time.time()
    # Inicia temporizador 
    start_temp = time.time()

    if True:
        while True:
            # Shows camera
            ret, image = cap.read()
            if not ret:
                break
            debug_image = copy.deepcopy(image)
            output_dict = {}

            # 1. opens camera and get keypoints
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)

            # Draw the pose annotation on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)       

            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            
            cv2.imshow('MediaPipe Pose Detection', image)
            
            if results.pose_world_landmarks:
                kpts = gen_landmark_list(results.pose_world_landmarks, kp_to_save) # vector de vectores con las posiciones de los keypoints

                for kpt_num in range(kpts.shape[0]):
                    kpts[kpt_num] = R @ kpts[kpt_num]
                
                kpts = convert_to_dictionary(kpts, keypoints_to_index)
                kpts = add_hips_and_neck(kpts)
                output_dict = fill_output_dict(kpts, output_dict, 'position', kpts['joints'])

                get_bone_lengths(kpts)
                get_base_skeleton(kpts)
                ang_to_print = ['r_shoulder', 'r_elbow', 'r_wrist']
                fstring = calculate_joint_angles(kpts, output_dict, ang_to_print)
                fstring = f'{time.time()-start} # ' + fstring
                send_yarp_str(fstring, ang_port)
                
                output_str = dict_to_string(output_dict)
                fig = draw_skeleton_from_joint_angles(kpts)

                # redraw the canvas
                fig.canvas.draw()
                
                # convert canvas to image
                plot_img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
                        sep='')
                plot_img  = plot_img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

                # 2. send keypoints to yarp port
                msg = dict_to_string(kpts)
                send_yarp_msg(msg, output_port)
                time.sleep(0.08)

            # Flip the image horizontally for a selfie-view display.
            key = cv2.waitKey(1)
            time_temp = time.time() - start_temp
            if key == 27 or time_temp>60:  # ESC
                break
            
    cap.release()
    cv2.destroyAllWindows()
    
