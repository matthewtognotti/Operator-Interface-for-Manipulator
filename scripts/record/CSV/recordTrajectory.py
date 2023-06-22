import sys
import rospy
import moveit_commander
import csv


# This script records the trajectory of the end effector using moveit

class pandaRecordTrajectory():
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

    def saveTrajectory(self, fileName='modulename') -> None:

        # Get the end effector position and orientation
        # eefPosition = self.group.get_current_pose().pose.position
        # eefOrientation = self.group.get_current_pose().pose.orientation


        # Save this data in the CSV file named by the user
        headers = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
   
        # Open the CSV file and write to it
        with open(fileName, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)

            #loop for trajectory data
            while True:
                eefPose = self.group.get_current_pose().pose

                row_data = [eefPose.position.x,
                eefPose.position.y,
                eefPose.position.z,
                eefPose.orientation.x,
                eefPose.orientation.y,
                eefPose.orientation.z,
                eefPose.orientation.w]

                writer.writerow(row_data)


    def closeMoveit(self) -> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():

    try:
        record = pandaRecordTrajectory()

        # Retrieve file name from user
        fileName = input(
            "\nEnter the file name or path to save the CSV data: ")

        # Save the eef pose into the file
        record.saveTrajectory(fileName)

        print("\nCSV file has been saved with name " + fileName + "\n")

        # close moveit
        record.closeMoveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
