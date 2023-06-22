import sys
import rospy
import moveit_commander
from math import pi
import pickle
from pickAndPlace import movePanda
import geometry_msgs.msg
from time import sleep



# Edit a trajectory using the pose data from the teleop in keyboardPanda.py
# use APIs from PickandPlace.py

# length of box in lab
box_length = 0.0587375


# Coordinates of the boxes on table for picking and placing
boxes = [
    (0.35, 0.20, box_length/2, 'box 0'),
    (0.35, 0.26, box_length/2, 'box 1'),
    (0.35, 0.32, box_length/2, 'box 2'),
    (0.35, 0.38, box_length/2, 'box 3'),
    (0.35, 0.44, box_length/2, 'box 4'),
    (0.35, 0.50, box_length/2, 'box 5')
]



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
            
# This function edits the trajectory to pick up the next box
def pickAndPlace(self, pickTrajectory, placeTrajectory):
    pass

    # execute cartesian path using waypoints from the pose data to pick up the boxes

    # for b in boxes:
    #     self.cartesianPath(pickTrajectory)
    #     sleep(2)
    #     self.cartesianPath(placeTrajectory)
    #     sleep(2)
    #     # edit the last pose in the list to pick up the next box
    #     # move to the next box
    #     pickTrajectory[-1].position.y = b[1]
    #     placeTrajectory[-1].position.y -= 0.06


def main():
    
    # Retrieve the file names
    # filename0 = input("\nEnter the file path of the pick trajectory: ")
    # filename1 = input("\n\nEnter the file path of the place trajectory: ")
    # filename2 = input("\n\nEnter the file path of the new home trajectory: ")

    # Open the files
    file0 = open("PICKBOX1", "rb")
    file1 = open("PLACEBOX1", "rb")
    file2 = open("NEWHOMEPOSITION", "rb")

    # Unpickle the files
    pickTrajectory = pickle.load(file0)
    placeTrajectory = pickle.load(file1)
    homeTrajectory = pickle.load(file2)
    print("\n")

    # set up moveIt
    panda = movePanda()
    

    # EXAMPLE OF HOW TO EDIT THE POSE DATA STRUCTURE

    # Example: we can edit the coordinates of each pose in the pose data structure
    print("\nFirst pose y position:")
    print(pickTrajectory[0].position.y)
    print("\n")

    #last pose in the list to pick next box
    print("\nLast pose y position: ")
    print(pickTrajectory[-1].position.y)
    print("\n")

    # EXAMPLE END

    # add 9.5cm in y direction
    #poseList[-1].position.y += 0.095

    # make every y position negative
        
    # First, initialize the scene
    initializeScene(panda)
    
    # Then, put panda in home position
    panda.goHome()

    # trajectory for new home possition
    panda.cartesianPathPoses(homeTrajectory)
    
    # pick and place the boxes
    #panda.pickAndPlace(panda, pickTrajectory, placeTrajectory)

    # go to first pbox
     # Pick up the box
    panda.cartesianPathPoses(pickTrajectory)

    for i in range(len(boxes)):

        # # edit the pick trajectory to pick up the next box
        # pickTrajectory[-1].position.y = boxes[i][1]

        # # Pick up the box
        # panda.cartesianPathPoses(pickTrajectory)

        # close the gripper and attach box
        panda.openOrCloseGripper(mode = "close")
        panda.attachBox(box_name = boxes[i][3])

        # edit the place trajectory
        placeTrajectory[-1].position.y -= 0.06

        # Place the Box
        panda.cartesianPathPoses(placeTrajectory)

        # open the gripper and dettach the object
        panda.openOrCloseGripper(mode = "open")
        panda.detachBox(box_name = boxes[i][3])

        # reverse and edit the place trajectory to go to the next box
        placeReverse = placeTrajectory
        placeReverse.reverse()
        placeReverse[-1].position.y = boxes[i+1][1]
        panda.cartesianPathPoses(placeReverse)

    


if __name__ == '__main__':
    main()
