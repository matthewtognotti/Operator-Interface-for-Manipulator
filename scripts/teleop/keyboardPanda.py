import sys
import os
import rospy
import moveit_commander
import copy
import pickle
import geometry_msgs.msg

from math import pi
from pynput import keyboard

from teleopCSV import pandaRecord

module_path = os.path.join(os.path.dirname(__file__), '..', 'utility')
sys.path.append(module_path)
from pandaGoHome import movePanda


# File: keyboardPanda.py

# This script can control Panda with keyboard, record trajectory in CSV, and replay and save trajectories

# The script first puts the Panda in its home position


# length of box in lab
box_length = 0.0587375


# Coordinates of the boxes on table for picking and placing
boxes = [
    (0.35, 0.0, box_length/2, 'box 0'),
    (0.35, 0.06, box_length/2, 'box 1'),
    (0.35, 0.12, box_length/2, 'box 2'),
    (0.35, 0.18, box_length/2, 'box 3'),
    (0.35, 0.24, box_length/2, 'box 4'),
    (0.35, 0.30, box_length/2, 'box 5')
]



class pandaControl():
    def __init__(self) -> None:

        # Get  CSV file name
       
        if len(sys.argv) < 3:
            print("\nPlease enter a file path for the data. <csv_file_path> <trajectory_file_path>\n")
            print("Exiting program...\n")
            sys.exit()

        self.csvFileName = sys.argv[1] 
        self.trajectoryFileName = sys.argv[2]

        # instantiate pandaRecord from recordPoseCSV.py
        self.record = pandaRecord()

        # Open the CSV file and write the headers
        self.record.openCSV(self.csvFileName)

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("panda_teleop", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # List of poses as wayoints for compute_cartesian_path
        self.poses = []

        # Amount to move on key press
        self.del_cartesian = 0.015
    
        eef_link = self.group.get_end_effector_link()

        # slow down the panda eef link
        self.group.limit_max_cartesian_link_speed(speed = 0.035, link_name = eef_link)

    # put panda in home position
    def pandaHomePosition(self):
        panda = movePanda()
        panda.goHome()

    # To open and close the gripper
    def jointGoalGripper(self, joint0=0, joint1=0) -> None:
        group_name = "panda_hand"
        group = moveit_commander.MoveGroupCommander(group_name)

        # Move panda fingers
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = joint0
        joint_goal[1] = joint1

        # Move to commands
        group.go(joint_goal, wait=True)
        # Calling stop() ensures that there is no residual movement
        group.stop()
    
    def cartesianPath(self, x, y, z) -> None: 

        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.x += x
        wpose.position.y += y
        wpose.position.z += z
        waypoints.append(copy.deepcopy(wpose))

        # Interpolate cartesian path at resolution of 1 cm (eef_step). Disable jump threshold. Avoid
        # collisions with object you are picking up.
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            True)       # avoid_collisions

        self.group.execute(plan, wait=True)

    # rotate end effector
    def rotateEEF(self):
    
        joint_goal = self.group.get_current_joint_values()

        joint_goal[6] = 3/4 * pi

        self.group.go(joint_goal, wait=True)
        self.group.stop()

    # Replay the previous trajectory 
    def replayCartesianPath(self) -> None: 

        waypoints = self.poses

        # Interpolate cartesian path at resolution of 1 cm (eef_step). Disable jump threshold. Avoid
        # collisions with object you are picking up.
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            True)       # avoid_collisions
        
        self.group.execute(plan, wait=True)

     # initialize planning scene. Add tables and cubes to the scene
    def initializeScene(self):

        # first clear the scene
        self.scene.clear()

        # Add the table to the scene
        self.addBox(name="table", xpos=0.5, ypos=0.0,
                            zpos=-0.7293/2, size=(0.6096, 1.8288, 0.7293))
        
        # Add the Panda's frame so that it does not collide with it
        self.addBox(name="metal_frame", xpos=-0.1,
                            ypos=0, zpos=-0.4064/2, size=(0.5, 1.8288, 0.4064,))
        
        # Add all of the cubes to pick and place on table 
        for b in boxes:
            self.addBox(name = b[3], xpos=b[0], ypos=b[1],
                                zpos=b[2], size=(box_length, box_length, box_length))
            
    
     # Add a box to the planning scene 
    def addBox(self, name='box', xpos=0.0, ypos=0.0, zpos=0.0, size=(0.0, 0.0, 0.0)):

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = xpos
        box_pose.pose.position.y = ypos
        box_pose.pose.position.z = zpos
        box_name = name
        self.scene.add_box(box_name, box_pose, size=size)


    # Save the list of poses using pickle
    def saveTrajectory(self):

        # Serialize the list
        serializedObject = pickle.dumps(self.poses)

        # open the file
        file = open(self.trajectoryFileName, "wb")

        # Write the byte stream to the file
        file.write(serializedObject)

        # close the file
        file.close()

        print("\nPose list saved using pickle")


    def on_press(self, key):

        # first check if the key has attribute char
        if hasattr(key, 'char'):
            # move up (+z)
            if key.char == "w":
                self.cartesianPath(0, 0, self.del_cartesian)

            # move down (-z)
            elif key.char == "s":
                self.cartesianPath(0, 0, -self.del_cartesian)

            # move left (+y)
            elif key.char == "a":
                self.cartesianPath(0, self.del_cartesian, 0)

            # move right (-y)
            elif key.char == "d":
                self.cartesianPath(0, -self.del_cartesian, 0)

            # move forward (+x)
            elif key.char == "f":
                self.cartesianPath(self.del_cartesian, 0, 0)

            # move backward (-x)
            elif key.char == "b":
                self.cartesianPath(-self.del_cartesian, 0, 0)

            # close gripper
            elif key.char == "c":
                self.jointGoalGripper(0.00,0.00)

            # open gripper
            elif key.char == "o":
                self.jointGoalGripper(0.04, 0.04)

            # rotate end effector
            elif key.char == "e":
                self.rotateEEF()
            
            # go home
            elif key.char == "h":
                self.pandaHomePosition()
            
            # First, go home, then replay the trajectory using the poses list and cartesian path
            elif key.char == "r":
                self.pandaHomePosition()
                self.replayCartesianPath()
            
            # Save the current trajectory using pickle
            elif key.char == "u":
                self.saveTrajectory()

            
        # Record the pose of the end effector to the CSV file and the trajectory file
        self.record.savePose(self.csvFileName)

        # Add the pose to the list of waypoints
        current_position = self.group.get_current_pose().pose
        self.poses.append(current_position)

    def close_moveit(self)-> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():

    robot = pandaControl()

    # put panda in home position
   # robot.pandaHomePosition()

    # planning scene
    #robot.initializeScene()


    print("\nStarting keyboard listener...\n")

    print("Controls:")
    print("Move +z        [up]:         w")
    print("Move -z      [down]:         s")
    print("Move +y      [left]:         a")
    print("Move -y     [right]:         d")
    print("Move +x   [forward]:         f")
    print("Move -x  [backward]:         b\n")
    print("Open Gripper:                o")
    print("Close Gripper:               c\n")
    print("Rotate EEF:                  e\n")
    print("Replay trajectory:           r")
    print("Save trajectory:             u")
    print("Go Home:                     h\n")


    listener = keyboard.Listener(on_press = robot.on_press, on_release = None)
    listener.start()
    listener.join()

    robot.close_moveit

if __name__ == '__main__':
    main()




    