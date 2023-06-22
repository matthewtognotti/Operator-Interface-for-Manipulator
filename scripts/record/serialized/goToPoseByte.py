import sys
import rospy
import moveit_commander
import pickle


# Use the poses saved by pandaRecord.py to send commands to the Panda

# First pose is to pick up the first box, second pose is to place the second box.
# First need to get the name of the two poses, then unpickle both poses.

# 1. Use goToPoseEueler to go to the pose
# 2. Close the gripper
# 3. Go to the second pose
# 4. Open Gripper
# 5. Repeat 1-4 using modified poses for predefined box locations

class pandaGoToPose():

    def __init__(self) -> None:

        # initiliaze moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        # initialize node
        rospy.init_node("panda_go_to_pose", anonymous=True)

        group_name = "panda_arm"

        # MoveGroupCommander is the python interface to the move_group
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # Get the name of the end effector link
        self.eef_link = self.group.get_end_effector_link()

    # Function to open and close the gripper by a specified amount
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

    # Go to pose in quaternion and xyz
    def goToPose(self, savedPose):

        self.group.set_pose_target(savedPose)

        # Call the planner to compute the plan and execute it
        self.group.go(wait=True)

    def closeMoveit(self) -> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():

    try:

        # Retrieve the file name
        filename = input("\nEnter the name or path of the pose file: ")

        # Open the file
        file = open(filename, "rb")

        # Unpickle the file
        savedPose = pickle.load(file)

        # Print the pose
        print(savedPose)

        # Go to the pose
        panda = pandaGoToPose()
        panda.goToPose(savedPose)

        # Close MoveIt
        panda.closeMoveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
