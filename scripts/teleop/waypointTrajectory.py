import pickle
from pickAndPlace import movePanda
from pandaGoHomeNew import pandaHome


# Edit a trajectory using the pose data from the teleop in keyboardPanda.py
# use APIs from PickandPlace.py

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

def main():
    
    # Retrieve the file namess

    # Open the files
    file0 = open("serializedTrajectories/PICK3", "rb")
    file1 = open("serializedTrajectories/PLACE3", "rb")

    # Unpickle the files
    pickTrajectory = pickle.load(file0)
    placeTrajectory = pickle.load(file1)

    # set up moveIt
    panda = movePanda()
    

    # EXAMPLE OF HOW TO EDIT THE POSE DATA STRUCTURE

    print("Example: we can edit the coordinates of each pose in the pose data structure")
    print("\nFirst pose y position:")
    print(pickTrajectory[0].position.y)
    print("\n")

    #last pose in the list to pick next box
    print("\nLast pose y position: ")
    print(pickTrajectory[-1].position.y)
    print("\n")

    # EXAMPLE END

        
    # First, initialize the scene
    initializeScene(panda)
    
    # Then, put panda in home position
    pandaHome.goHome(panda)
    
    # pick and place the boxes
    #panda.pickAndPlace(panda, pickTrajectory, placeTrajectory)

    
    first = True # Don't shift first trajectory
    
    for i in range(len(boxes)):
    
        # edit the pick trajectory, shifts the y positions in the poses of the end effector
        if first == False:
            for p in range(0, len(pickTrajectory)):
                pickTrajectory[p].position.y += 0.06
        panda.cartesianPathPoses(pickTrajectory)

        # attach box and close gripper
        panda.openOrCloseGripper(mode = "close")
        panda.attachBox(box_name = boxes[i][3])


        # go back to home by reversing the pick trajectory
        pickReverse = pickTrajectory.copy()
        pickReverse.reverse()
        panda.cartesianPathPoses(pickReverse)

        # place box using placeTrajectory. shifts the y positions in the poses of the end effector
        if first == False:
            for p in range(10, len(placeTrajectory)):
                placeTrajectory[p].position.y -= 0.06
        panda.cartesianPathPoses(placeTrajectory)

        # Dettach box and open gripper
        panda.openOrCloseGripper(mode = "open")
        panda.detachBox(box_name = boxes[i][3])

        # reverse and the place trajectory to go back to home
        placeReverse = placeTrajectory.copy()
        placeReverse.reverse()
        panda.cartesianPathPoses(placeReverse)
        first = False

if __name__ == '__main__':
    main()
