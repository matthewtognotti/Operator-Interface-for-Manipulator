import sys
import rospy
import moveit_commander
import geometry_msgs.msg


'''
    File: envoronment.py

    Set up the environment for the robot scenario
    
'''

# Class to add objects to the environment in RViz using MoveIt
class environment():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()

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


def main():
    work_environment = environment()

    # First clear the scene
    work_environment.scene.clear()

    # Add the robot base
    work_environment.addBox(name="metal_frame", xpos=0, ypos=0, zpos= -0.25/2, size=(0.5, 0.5, 0.25))

    # Add the pick table to the scene
    work_environment.addBox(name="pick_table", xpos=0.0, ypos=0.25 + 0.6096/2, zpos=-0.7293/2, size=(1.8288, 0.6096, 0.7293))
        
    # Add the place table to the scene
    work_environment.addBox(name="place_table", xpos=0.0, ypos= -(0.25 + 0.6096/2), zpos=-0.7293/2, size=(1.8288, 0.6096, 0.7293))

if __name__ == '__main__':
    main()
