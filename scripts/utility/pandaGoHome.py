import rospy
import sys
import moveit_commander
from math import pi


class movePanda():

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.group_names = self.robot.get_group_names()
        self.eef_link = self.group.get_end_effector_link()

       # slow down panda
        self.group.set_max_velocity_scaling_factor(0.05)

    def goHome(self) -> None:
        joint_goal = self.group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -3*pi/4
        joint_goal[4] = 0
        joint_goal[5] = pi/2
        joint_goal[6] = pi/4

        self.group.go(joint_goal, wait=True)
        self.group.stop()

    # shut down Moveit
    def close_moveit(self) -> None:
        self.group.stop()
        self.group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


def main():

    try:
        print("\nPress Ctrl-D to exit at any time\n")

        # Set up the moveit_commander
        panda = movePanda()

       # Panda, go home!
        panda.goHome()

        # Close MoveIt
        panda.close_moveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
