import sys
import rospy
import moveit_commander
import csv


#  This script provides API to PandaTeleop to record the pose of the end effector 
#  and save in CSV

class pandaRecord():
    def __init__(self) -> None:
    
        group_name = "panda_arm"

        # MoveGroupCommander is the python interface to the move_group
        self.group = moveit_commander.MoveGroupCommander(group_name)

    # open the CSV file 
    def openCSV(self, fileName):

        # write the headers
        headers = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

        with open(fileName, 'w', newline='') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(headers)


    def savePose(self, fileName) -> None:

        # Get the end effector position and orientation
        # eefPosition = self.group.get_current_pose().pose.position
        # eefOrientation = self.group.get_current_pose().pose.orientation

        eefPose = self.group.get_current_pose().pose

        # print("\n")
        # print(eefPose)
        # print("\n")

        # Save this data in the CSV file named by the user
        row_data = [eefPose.position.x,
                    eefPose.position.y,
                    eefPose.position.z,
                    eefPose.orientation.x,
                    eefPose.orientation.y,
                    eefPose.orientation.z,
                    eefPose.orientation.w]
        
        # open file in append mode
        with open(fileName, 'a', newline='') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(row_data)