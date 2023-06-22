import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy
from math import pi
from tf.transformations import quaternion_from_euler


class move_panda():

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pandaTest", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.group_names = self.robot.get_group_names()
        self.eef_link = self.group.get_end_effector_link()

        
    # To open and close the gripper
    def joint_goal_gripper(self, joint0=0, joint1=0) -> None:
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


    def plan_cartesian_path(self, del_x=0.0, del_y=0.0, del_z=0.0) -> None:

        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.x += del_x
        wpose.position.y += del_y
        wpose.position.z += del_z
        waypoints.append(copy.deepcopy(wpose))

        # Interpolate cartesian path at resolution of 1 cm (eef_step). Disable jump threshold. Avoid
        # collisions with object you are picking up.
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            False)       # avoid_collisions

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    
    def execute_plan(self, plan)-> None:
        self.group.execute(plan, wait=True)

    
    def close_moveit(self)-> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()



def main():

    try:
        print("\nPress Ctrl-D to exit at any time\n")

        # Set up the moveit_commander
        panda = move_panda()

        # plan cartesian path
        cartesian_plan = panda.plan_cartesian_path(del_x=0.25, del_y=0, del_z=-0.25)[0]

        #execute cartesian path
        panda.execute_plan(cartesian_plan)

         #close the gripper
        panda.joint_goal_gripper(
                joint0=0.00, joint1=0.00)

        #open the gripper
        panda.joint_goal_gripper(joint0=0.04, joint1=0.04)

       
       
        # Close MoveIt
        panda.close_moveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()




