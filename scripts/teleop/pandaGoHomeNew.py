import sys
import rospy
import moveit_commander


class pandaHome():

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()

        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

       # slow down panda
        self.group.set_max_velocity_scaling_factor(0.25)

    def goHome(self) -> None:
        joint_goal = self.group.get_current_joint_values()

        joint_goal = [-0.4404190601299195, -0.5776600058186037, 0.45451886521579515, -2.7625563637759094, 0.30345392554870804, 2.2101263998271756, 2.1197258424005994]

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
        panda = pandaHome()

       # Panda, go home!
        #panda.goHome()
        print(panda.group.get_current_joint_values())

        # Close MoveIt
        panda.close_moveit()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
