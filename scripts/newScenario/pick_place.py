import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import pi
import copy
import environment
import pickle

'''
    File: pick_place.py

    Pick up object in the environment for the work/school scenario
    
'''

class movePanda():

    def __init__(self) -> None:

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("pick_and_place", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.group_names = self.robot.get_group_names()
        self.eef_link = self.group.get_end_effector_link()

        # robot speed
        self.group.set_max_velocity_scaling_factor(0.05)
        self.group.set_max_acceleration_scaling_factor(0.05)
         # slow down the panda eef link
        self.group.limit_max_cartesian_link_speed(speed = 0.05, link_name = self.eef_link)
    
    # go to a certain pose
    def goToPose(self, x_pos=0.4, y_pos=0.1, z_pos=0.4, roll=0.0, pitch=0.0, yaw=0.0):

        quaternion = quaternion_from_euler(roll, pitch, yaw)

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        pose_goal.position.x = x_pos
        pose_goal.position.y = y_pos
        pose_goal.position.z = z_pos

        self.group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    
    # cartesian path for end effector
    def cartesianPath(self, x, y, z): 

        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.x += x
        wpose.position.y += y
        wpose.position.z += z
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            False)       # avoid_collisions

        self.group.execute(plan, wait=True)

    
    def cartesianPathPoses(self, poses) -> None: 

        # Interpolate cartesian path at resolution of 1 cm (eef_step). Disable jump threshold. Avoid
        # collisions with object you are picking up.
        (plan, fraction) = self.group.compute_cartesian_path(
            poses,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            True)       # avoid_collisions
        
        self.group.execute(plan, wait=True)

    # To open and close the gripper
    def openOrCloseGripper(self, mode = "open"):

        group_name = "panda_hand"
        group = moveit_commander.MoveGroupCommander(group_name)

        # Move panda fingers
        
        joint_goal = group.get_current_joint_values()

        if mode == "open":
            joint_goal[0] = 0.04
            joint_goal[1] = 0.04
        else:
            joint_goal[0] = 0.01
            joint_goal[1] = 0.01

        group.go(joint_goal, wait=True)
        # Calling stop() ensures that there is no residual movement
        group.stop()

    def attachBox(self, box_name='box'):
        touch_links = self.robot.get_link_names(group='panda_hand')
        touch_links.append('panda_hand_sc')
        self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)

    def detachBox(self, box_name='box'):
        eef_link = self.group.get_end_effector_link()
        self.scene.remove_attached_object(eef_link, name=box_name)
    
    # put panda in home position
    def goHome(self) -> None:
        joint_goal = self.group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = -0.785
        joint_goal[2] = 0
        joint_goal[3] = -2.356
        joint_goal[4] = 0
        joint_goal[5] = 1.571
        joint_goal[6] = 0.785

        self.group.go(joint_goal, wait=True)
        self.group.stop()

    # shut down Moveit
    def closeMoveit(self) -> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()

    def goToSavedPose(self):
         # Retrieve the file name
        filename = input("\nEnter the name or path of the pose file: ")

        # Open the file
        file = open(filename, "rb")

        # Unpickle the file
        savedPose = pickle.load(file)

        self.group.set_pose_target(savedPose)

        # Call the planner to compute the plan and execute it
        self.group.go(wait=True)



def main():

    try:
        print("\nStarting Pick and Place Task...\n\n")

        # Set up the environment
        environment.main()
        
        # Set up the moveit
        panda = movePanda()

        # go home first and open gripper
        panda.goHome()
        panda.openOrCloseGripper(mode="open")

        file0 = open("sanitizer_trajectory", "rb")
        file1 = open("place_trajectory", "rb")

        # Unpickle the file
        pickTrajectory = pickle.load(file0)
        placeTrajectory = pickle.load(file1)

        panda.cartesianPathPoses(pickTrajectory)

        panda.openOrCloseGripper(mode="close")

        panda.cartesianPath(x=0.0, y=0.0, z=0.1)

        panda.cartesianPathPoses(placeTrajectory)

        panda.openOrCloseGripper(mode="open")

        panda.goHome()

        # Close MoveIt
        panda.closeMoveit()

        print("\nPick and Place task complete\n\n")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
