
from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class Behavior(Node):

    def __init__(self, node_name="behavior"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(10))
        self.declare_parameter("plan_frequency", float(2))

        # NEW: Stuck Detection
        self.declare_parameter("stuck_timeout", 3.0)
        self.declare_parameter("stuck_distance", 0.05)

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.plan_frequency_ = self.get_parameter("plan_frequency").value

        # NEW: Stuck Detection
        self.stuck_timeout_ = self.get_parameter("stuck_timeout").value
        self.stuck_distance_ = self.get_parameter("stuck_distance").value

        # Handles: Topic Subscribers
        # Goal pose subscriber
        self.sub_goal_pose_ = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.callbackSubGoalPose_,
            10,
        )

        # Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry,
            "odom",
            self.callbackSubOdom_,
            10,
        )

        # Handles: Topic Publishers
        # Path request publisher
        self.pub_path_request_ = self.create_publisher(
            Path,
            "path_request",
            10,
        )

        # Handles: Timers
        self.timer = self.create_timer(
            1.0 / self.frequency_, self.callbackTimer_
        )

        self.timer_plan_ = self.create_timer(
            1.0 / self.plan_frequency_, self.callbackTimerPlan_
        )

        # Other Instance Variables
        self.received_goal_coords_ = False
        self.received_rbt_coords_ = False
        self.goal_reached_ = True

        # NEW: Stuck Detection
        self.stuck_start_time_ = None
        self.stuck_start_position_ = None
        self.is_stuck_ = False

    # Callbacks =============================================================

    # Goal pose subscriber callback.
    def callbackSubGoalPose_(self, msg: PoseStamped):
        self.received_goal_coords_ = True
        self.goal_reached_ = True

        self.goal_x_ = msg.pose.position.x
        self.goal_y_ = msg.pose.position.y

        # NEW: Reset stuck detection for new goal
        self.resetStuckDetection_()

        self.get_logger().info(
            f"Received New Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
        )

    # Odometry subscriber callback.
    def callbackSubOdom_(self, msg: Odometry):
        self.received_rbt_coords_ = True

        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

    # NEW: Stuck Detection
    def resetStuckDetection_(self):
        self.stuck_start_time_ = None
        self.stuck_start_position_ = None
        self.is_stuck_ = False

    # NEW: Stuck Detection
    def checkStuck_(self):
        now = self.get_clock().now()

        # Start a new monitoring window
        if self.stuck_start_time_ is None:
            self.stuck_start_time_ = now
            self.stuck_start_position_ = (
                self.rbt_x_,
                self.rbt_y_,
            )
            return

        # Calculate movement from the starting position
        movement = hypot(
            self.rbt_x_ - self.stuck_start_position_[0],
            self.rbt_y_ - self.stuck_start_position_[1],
        )

        # Calculate elapsed time in seconds
        elapsed = (
            now - self.stuck_start_time_
        ).nanoseconds / 1e9

        # Robot has moved sufficiently
        if movement >= self.stuck_distance_:
            self.stuck_start_time_ = now
            self.stuck_start_position_ = (
                self.rbt_x_,
                self.rbt_y_,
            )

            if self.is_stuck_:
                self.get_logger().info(
                    "Robot movement detected. Stuck condition cleared."
                )

            self.is_stuck_ = False
            return

        # Robot has not moved sufficiently within timeout
        if elapsed >= self.stuck_timeout_:
            if not self.is_stuck_:
                self.is_stuck_ = True

                self.get_logger().warn(
                    f"Robot STUCK! Movement < "
                    f"{self.stuck_distance_:.3f} m "
                    f"for {self.stuck_timeout_:.1f} seconds."
                )

    # Callback for timer.
    def callbackTimer_(self):
        if not self.received_rbt_coords_ or not self.received_goal_coords_:
            return

        dx = self.goal_x_ - self.rbt_x_
        dy = self.goal_y_ - self.rbt_y_

        goal_is_close = hypot(dx, dy) < 0.1

        if goal_is_close and not self.goal_reached_:
            self.get_logger().info(
                f"Reached Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
            )
            self.goal_reached_ = True

            # NEW: Stop monitoring when goal reached
            self.resetStuckDetection_()

        elif not goal_is_close and self.goal_reached_:
            self.get_logger().info(
                f"Going to Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
            )
            self.goal_reached_ = False

        # NEW: Check for stuck condition during navigation
        if not self.goal_reached_:
            self.checkStuck_()

    # Callback for publishing path requests
    def callbackTimerPlan_(self):
        if not self.received_goal_coords_ or not self.received_rbt_coords_:
            return

        # Create a new message for publishing
        msg_path_request = Path()
        msg_path_request.header.stamp = self.get_clock().now().to_msg()
        msg_path_request.header.frame_id = "map"

        # Starting pose: current robot position
        rbt_pose = PoseStamped()
        rbt_pose.header = msg_path_request.header
        rbt_pose.pose.position.x = self.rbt_x_
        rbt_pose.pose.position.y = self.rbt_y_
        rbt_pose.pose.orientation.w = 1.0

        # Destination pose: requested goal
        goal_pose = PoseStamped()
        goal_pose.header = msg_path_request.header
        goal_pose.pose.position.x = self.goal_x_
        goal_pose.pose.position.y = self.goal_y_
        goal_pose.pose.orientation.w = 1.0

        msg_path_request.poses.append(rbt_pose)
        msg_path_request.poses.append(goal_pose)

        # Publish the message
        self.get_logger().info(
            f"Sending Path Planning Request from Rbt @ "
            f"({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) "
            f"to Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})"
        )

        self.pub_path_request_.publish(msg_path_request)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Behavior())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
