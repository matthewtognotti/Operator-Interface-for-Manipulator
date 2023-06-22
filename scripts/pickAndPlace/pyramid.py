from pickAndPlace import movePanda

import rospy
from math import pi


# Use APIs from PickAndPlace.py to move cubes into a pyramid

box_length = 0.0587375

pyramidCoordinates = [
    (0.5, -0.20, (box_length/2) + 0.115),
    (0.5, -0.26, (box_length/2) + 0.115),
    (0.5, -0.32, (box_length/2) + 0.115),
    (0.5, -0.20 - box_length/2, (box_length * 1.5) + 0.115),
    (0.5, -0.26 - box_length/2, (box_length * 1.5) + 0.115),
    (0.5, -0.26, (box_length * 2.5) + 0.115)
]

boxes = [
    (0.50, 0.20, box_length/2, 'box 0'),
    (0.50, 0.30, box_length/2, 'box 1'),
    (0.50, 0.40, box_length/2, 'box 2'),
    (0.40, 0.20, box_length/2, 'box 3'),
    (0.40, 0.30, box_length/2, 'box 4'),
    (0.40, 0.40, box_length/2, 'box 5')
]

# Assumes the panda is already above the current box to be placed

def placePyramid(panda, currentBox, pyramidPosition):

    # open the gripper
    panda.openOrCloseGripper(mode="open")

    # lower the arm
    panda.cartesianPath(x=0, y=0, z=-0.1)

    # close the gipper
    panda.openOrCloseGripper(mode="close")

    # attach the cube
    panda.attachBox(box_name = currentBox[3])

    # raise the arm
    panda.cartesianPath(x=0, y=0, z=0.25)

    # go slightly above the pyramid position
    panda.goToPose(x_pos=pyramidCoordinates[pyramidPosition][0], 
                   y_pos=pyramidCoordinates[pyramidPosition][1], 
                   z_pos=pyramidCoordinates[pyramidPosition][2]+0.1, 
                   roll=pi, pitch=0.0, yaw=pi/4)

    # lower the arm
    panda.cartesianPath(x=0, y=0, z=-0.095)

    # open the gripper
    panda.openOrCloseGripper(mode="open")

    # detach the object
    panda.detachBox(box_name = currentBox[3])

    # raise the arm
    panda.cartesianPath(x=0, y=0, z=0.25)


def main():

    try:
        print("\nStarting Pick and Place Task...\n\n")

        # Set up the moveit
        panda = movePanda()

        # first, go home position

        panda.goHome()

        # clear the scene
        panda.scene.clear()

        print("Adding scene and objects...\n")

        # Add the table to the scene
        print("Adding table to scene...\n")
        panda.addBox(name="table", xpos=0.5, ypos=0.0,
                            zpos=-0.7293/2, size=(0.6096, 1.8288, 0.7293))
        
        # Add all of the cubes to pick and place on table 
        for b in boxes:
            panda.addBox(name = b[3], xpos=b[0], ypos=b[1],
                                zpos=b[2], size=(box_length, box_length, box_length))
        
        # Add the Panda's frame so that it does not collide with it
        panda.addBox(name="metal_frame", xpos=-0.1,
                            ypos=0, zpos=-0.4064/2, size=(0.5, 1.8288, 0.4064,))

        # index for boxes list 
        position = 0
        # index for pyramid list 
        pyramidPosition = 0


        while len(boxes) > 0:

            currentBox = boxes[position]
            
            # go to the pose slightly above the current box 
            panda.goToPose(x_pos=currentBox[0], y_pos=currentBox[1], z_pos=currentBox[2] + 0.225, roll=pi, pitch=0.0, yaw=pi/4)

            while True:
                print(
                    "\nSelect Box: \n \t (c) Current \n \t (n) Next \n \t (p) Previous \n \t (a) Place all")
                userInput = input("Input: ").strip().lower()

                # Place current box
                if userInput == "c":
                    # pick up box and place in pyramid
                    placePyramid(panda, currentBox, pyramidPosition)
                    pyramidPosition += 1
                    boxes.pop(position)  # Remove box from list
                    if len(boxes) == 0:
                        break
                    position %= len(boxes)  # Stay in indices of list
                    break
                # Go to next box
                elif userInput == 'n':
                    position += 1
                    position %= len(boxes)
                    break
                # Go to previous box
                elif userInput == "p":
                    position -= 1
                    position %= len(boxes)
                    break
                # Place all boxes
                elif userInput == "a":
                    while len(boxes) > 0:
                        currentBox = boxes[0]
                        panda.goToPose(x_pos=currentBox[0], y_pos=currentBox[1], z_pos=currentBox[2] + 0.225, roll=pi, pitch=0.0, yaw=pi/4)
                        placePyramid(panda, currentBox, pyramidPosition)
                        pyramidPosition += 1
                        boxes.pop(0)
                    break
                else:
                    print("Invalid Input")

        # once all boxes are placed, go home
        panda.goHome()

        print("\nPyramid Complete")

        # Close MoveIt
        panda.closeMoveit()


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()

