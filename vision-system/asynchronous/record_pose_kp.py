import cv2
import mediapipe as mp
import time
import threading

""" detección en tiempo real de pose humana (pose) """
""" traza el esqueleto en el video, no hace gráfica aparte """
""" se almacena la data en un archivo .dat """
# utilizando script de https://google.github.io/mediapipe/solutions/pose.html

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

landmark_index_dict = {0: 'nose',
                      1: 'l_eye_inner', 
                      2: 'l_eye', 
                      3: 'l_eye_outer', 
                      4: 'r_eye _inner', 
                      5: 'r_eye', 
                      6: 'r_eye_outer', 
                      7: 'l_ear', 
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

def gen_landmark_list(result_pose_landmark, save_joints):
    """ guardamos solo los puntos indicados en la lista save_joints """
    output = []

    for i in range(len(result_pose_landmark.landmark)):
        if landmark_index_dict[i] in save_joints:
            data_point = result_pose_landmark.landmark[i]
            output.append(data_point.x)
            output.append(data_point.y)
            output.append(data_point.z)
    return output

# For webcam input:

cap = cv2.VideoCapture(0)
# we will use ankle point as foot point

kp_to_save = ['nose',
              'l_shoulder', 'r_shoulder',
              'l_elbow', 'r_elbow',
              'l_wrist', 'r_wrist',
              'l_thumb', 'r_thumb',
              'l_hip', 'r_hip',
              'l_knee', 'r_knee',
              'l_ankle', 'r_ankle',
              'l_heel', 'r_heel',
              'l_foot_index', 'r_foot_index'
            ]

time.sleep(0.5)

frame_counter = 0

## UNCOMENT FOR USAGE ##
'''
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
  
  file = open('pose_data_demo.dat', "w+") # data file name 

  while cap.isOpened():
    frame_counter += 1
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.

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

    pose_dict_xyz = gen_landmark_list(results.pose_landmarks, kp_to_save)
    pose_dict_xyz = str(pose_dict_xyz).replace('[', '').replace(']', '').replace(',', '')
    file.write(f"{pose_dict_xyz}\n")
    #file.write(f"{results.pose_world_landmarks}\n")

    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      file.close()
      break
    if frame_counter>2000:
       file.close()
       break
cap.release()

'''