import sys
import rospy
import moveit_commander
import csv

# CSV VERSION

# This script records the pose of the end effector using moveit and stores the pose as a CSV file


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

        # Get the end effector position and orientation
        # eefPosition = self.group.get_current_pose().pose.position
        # eefOrientation = self.group.get_current_pose().pose.orientation

        eefPose = self.group.get_current_pose().pose

        print("\n")
        print(eefPose)
        print("\n")

        # Save this data in the CSV file named by the user
        headers = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        row_data = [eefPose.position.x,
                    eefPose.position.y,
                    eefPose.position.z,
                    eefPose.orientation.x,
                    eefPose.orientation.y,
                    eefPose.orientation.z,
                    eefPose.orientation.w]
        # Open the CSV file and write to it
        with open(fileName, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)
            writer.writerow(row_data)

        print("\nCSV file has been saved with name " + fileName + "\n")

    def closeMoveit(self) -> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():

    try:
        record = pandaRecord()

        # Retrieve file name from user
        fileName = input(
            "\nEnter the file name or path to save the CSV data: ")

        # Save the eef pose into the file
        record.savePose(fileName)

        # close moveit
        record.closeMoveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
