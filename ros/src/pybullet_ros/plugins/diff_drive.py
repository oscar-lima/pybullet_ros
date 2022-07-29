import rospy
from geometry_msgs.msg import Twist


class DiffDrive:
    def __init__(self, pybullet, robot, **kwargs):
        self.pb = pybullet
        self.robot = robot

        try:
            name = kwargs["name"]
            parameters = rospy.get_param("~" + name)
        except Exception:
            rospy.logerr("Not loading plugin {}".format(self.__class__.__name__))
            import traceback
            traceback.print_exc()
            self.no_execution = True
            return

        # joint information preparation
        self.revolute_joints_id_name = kwargs["rev_joints"]
        self.revolute_joints_name_id = {v: k for k, v in self.revolute_joints_id_name.items()}

        # a flag to monitor if anything went wrong
        self.no_execution = False

        cmd_vel_topic = parameters.get("cmd_vel", "~cmd_vel")
        self.cmd_vel_subscriber = rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        if (wheel_separation := parameters.get("wheel_separation", None)) is None:
            rospy.logwarn("No wheel_separation provided, using 1.0 as default")
            wheel_separation = 1.0
        self.wheel_separation = wheel_separation
        if (wheel_radius := parameters.get("wheel_radius", None)) is None:
            rospy.logwarn("No wheel_radius provided, using 1.0 as default")
            wheel_radius = 1.0
        self.wheel_radius = wheel_radius

        if (left_joints := parameters.get("left_joints", None)) is None:
            rospy.logerr("No left_joints provided")
            self.no_execution = True
            return
        if isinstance(left_joints, list) is False and type(left_joints) == str:
            left_joints = [left_joints]
        if (right_joints := parameters.get("right_joints", None)) is None:
            rospy.logerr("No right_joints provided")
            self.no_execution = True
            return
        if isinstance(right_joints, list) is False and type(right_joints) == str:
            right_joints = [right_joints]

        self.joint_indices = {"left": [], "right": []}
        for side in ["left", "right"]:
            for joint_name in eval(side + "_joints"):
                if joint_name not in self.revolute_joints_name_id:
                    rospy.logerr("Could not find joint {} in urdf".format(joint_name))
                    self.no_execution = True
                    return
                self.joint_indices[side].append(self.revolute_joints_name_id[joint_name])

        self.cmd_vel: Twist = Twist()

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def get_wheel_speeds(self):
        vx = self.cmd_vel.linear.x
        wz = self.cmd_vel.angular.z
        left = (vx - wz * self.wheel_separation / 2.0) / self.wheel_radius
        right = (vx + wz * self.wheel_separation / 2.0) / self.wheel_radius
        return {"left": left, "right": right}

    def execute(self):
        if not self.no_execution:
            speeds = self.get_wheel_speeds()
            for side in ["left", "right"]:
                indices = self.joint_indices[side]
                self.pb.setJointMotorControlArray(
                    bodyUniqueId=self.robot,
                    jointIndices=indices,
                    controlMode=self.pb.VELOCITY_CONTROL,
                    targetVelocities=[speeds[side] for _ in range(len(indices))]
                )
