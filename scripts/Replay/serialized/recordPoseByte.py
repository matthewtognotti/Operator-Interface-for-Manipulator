import sys
import rospy
import moveit_commander
import pickle

# This script records the pose of the end effector using moveit and stores the pose in a file


class pandaRecord():
    def __init__(self) -> None:
        # initiliaze moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        # initialize node
        rospy.init_node("record_panda", anonymous=True)

        group_name = "panda_arm"

        # MoveGroupCommander is the python interface to the move_group
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # Get the name of the end effector link
        self.eef_link = self.group.get_end_effector_link()

    def savePose(self, fileName='modulename') -> None:
        # save the eef_pose as a pick module with the given name

        # Get the end effector pose in xyz and quaternion
        eef_pose = self.group.get_current_pose()

        # Serialize the pose
        serializedObject = pickle.dumps(eef_pose)

        # open the file
        file = open(fileName, "wb")

        # Write the byte stream to the file
        file.write(serializedObject)

        # close the file
        file.close()

        # print the pose
        print("\n Printing the pose data: \n")
        print(eef_pose)

    def closeMoveit(self) -> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():

    try:
        record = pandaRecord()

        # Retrieve file name from user
        fileName = input("\nEnter the file name or path to save the pose: ")

        # Save the eef pose into a file as a pickle module with that name
        record.savePose(fileName)

        # close moveit
        record.closeMoveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
