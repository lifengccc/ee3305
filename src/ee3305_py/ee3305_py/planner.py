from heapq import heappush, heappop
from itertools import count
from math import hypot, floor, inf, sqrt, atan2, pi
from time import perf_counter

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        return self.g < other.g


class AStarNode(DijkstraNode):
    def __lt__(self, other):  # sort by f = g + h, break ties towards the node closer to the goal
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))
        self.declare_parameter("algorithm", "dijkstra")  # "dijkstra" or "astar"
        self.declare_parameter("heuristic_weight", float(1.0))  # A* only. 1.0 is optimal, >1.0 is weighted A* (faster but suboptimal)
        self.declare_parameter("smoothing", False)  # shortcut the grid path, then resample it. Works with both algorithms.

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value
        self.algorithm_ = self.get_parameter("algorithm").value
        self.smoothing_ = self.get_parameter("smoothing").value
        self.heuristic_weight_ = self.get_parameter("heuristic_weight").value
        if self.algorithm_ not in ("dijkstra", "astar"):
            self.get_logger().warn(f'Unknown algorithm "{self.algorithm_}". Using dijkstra.')
            self.algorithm_ = "dijkstra"

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        self.sub_path_request_ = self.create_subscription(
            Path, "path_request", self.callbackSubPathRequest_, 10
        )

        # Handles: Publishers
        self.pub_path_ = self.create_publisher(Path, "path", 10)

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):   
        
        if len(msg.poses) < 2:
            self.get_logger().warn("Path request needs robot and goal poses")
            return
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y

        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        
        self.costmap_ = list(msg.data)
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        if self.algorithm_ == "astar":
            self.astar_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)
        else:
            self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        c = floor((x - self.costmap_origin_x_) / self.costmap_resolution_)
        r = floor((y - self.costmap_origin_y_) / self.costmap_resolution_)

        return c, r

    # Converts cell column and cell row to world coordinates (cell center).
    def CRToXY_(self, c, r):
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_

        return x, y

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        return int(r * self.costmap_cols_ + c)

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        return c < 0 or r < 0 or c >= self.costmap_cols_ or r >= self.costmap_rows_

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):

        # Initializations ---------------------------------
        t0 = perf_counter()
        n_expanded = 0
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)

        if self.outOfMap_(rbt_c, rbt_r) or self.outOfMap_(goal_c, goal_r):
            self.get_logger().warn("Start or goal is outside the map!")
            return

        # Initialize nodes
        nodes = [
            DijkstraNode(i % self.costmap_cols_, i // self.costmap_cols_)
            for i in range(self.costmap_cols_ * self.costmap_rows_)
        ]

        # Initialize start
        start_node = nodes[self.CRToIndex_(rbt_c, rbt_r)]
        start_node.g = 0.0

        # Initialize open list
        # Open list entries are (g, tie-breaker, node) snapshots. Pushing the node alone would let a later
        # g update change the key of an entry already inside the heap and corrupt its ordering.
        tie = count()
        open_list = []
        heappush(open_list, (start_node.g, next(tie), start_node))

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)[2]

            # Skip if visited
            if node.expanded:
                continue
            node.expanded = True
            n_expanded += 1

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                self.publishNodePath_(
                    node, "Dijkstra", start_x, start_y, goal_x, goal_y, n_expanded, t0
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = node.c + dc
                nb_r = node.r + dr

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                nb_idx = self.CRToIndex_(nb_c, nb_r)

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_node.expanded:
                    continue

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                cost = self.costmap_[nb_idx]
                if cost < 0 or cost > self.max_access_cost_:
                    continue

                # Get the relative g-cost and push to open-list
                new_g = node.g + hypot(dc, dr) * (cost + 1)
                if new_g < nb_node.g:
                    nb_node.g = new_g
                    nb_node.parent = node
                    heappush(open_list, (nb_node.g, next(tie), nb_node))

        self.get_logger().warn(f"[Dijkstra] No Path Found! Expanded {n_expanded} nodes.")


    # Cost of travelling in a straight line between two cells, using the planner's step cost
    # (distance * (cell cost + 1)). Returns inf if the line touches an inaccessible cell.
    def lineCost_(self, c0, r0, c1, r1):
        length = hypot(c1 - c0, r1 - r0)
        n = max(1, int(length * 2))  # sample every half cell
        step = length / n
        cost = 0.0
        for i in range(1, n + 1):
            c = floor(c0 + 0.5 + (c1 - c0) * i / n)
            r = floor(r0 + 0.5 + (r1 - r0) * i / n)
            if self.outOfMap_(c, r):
                return inf
            cell = self.costmap_[self.CRToIndex_(c, r)]
            if cell < 0 or cell > self.max_access_cost_:
                return inf
            cost += step * (cell + 1)
        return cost

    # Any-angle shortcutting of a grid path. nodes are ordered start to goal and carry their g-costs.
    # A shortcut from node i to node j is only accepted if it is collision free AND does not cost more
    # than the grid path it replaces (g_j - g_i), so the smoothed path is never worse by the planner's own cost.
    def shortcutNodes_(self, nodes):
        keep = [nodes[0]]
        i = 0
        while i < len(nodes) - 1:
            best = i + 1  # the next grid cell is always valid
            for j in range(i + 2, len(nodes)):
                line_cost = self.lineCost_(nodes[i].c, nodes[i].r, nodes[j].c, nodes[j].r)
                if line_cost <= nodes[j].g - nodes[i].g:
                    best = j
                else:
                    break  # stop at the first failure to keep planning time bounded
            keep.append(nodes[best])
            i = best
        return keep

    # Resamples a polyline of (x, y) points at a fixed spacing so that the controller still gets dense waypoints.
    def resample_(self, pts, spacing):
        out = [pts[0]]
        carry = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            seg = hypot(x1 - x0, y1 - y0)
            d = spacing - carry
            while d < seg:
                out.append((x0 + (x1 - x0) * d / seg, y0 + (y1 - y0) * d / seg))
                d += spacing
            carry = seg - (d - spacing)
        if hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > 1e-9:
            out.append(pts[-1])
        return out

    # Total length, and total absolute heading change (rad), of a polyline. The latter measures jaggedness.
    def pathStats_(self, pts):
        length = 0.0
        turning = 0.0
        prev = None
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            d = hypot(x1 - x0, y1 - y0)
            if d < 1e-9:
                continue
            length += d
            h = atan2(y1 - y0, x1 - x0)
            if prev is not None:
                dh = (h - prev + pi) % (2 * pi) - pi
                turning += abs(dh)
            prev = h
        return length, turning

    # Backtracks from the goal node, optionally smooths, publishes the path, and logs stats so algorithms can be compared.
    def publishNodePath_(self, goal_node, name, start_x, start_y, goal_x, goal_y, n_expanded, t0):
        t_plan = perf_counter()

        # obtain the path from the nodes (start to goal).
        nodes = []
        n = goal_node
        while n is not None:
            nodes.append(n)
            n = n.parent
        nodes.reverse()

        raw_pts = [self.CRToXY_(n.c, n.r) for n in nodes]
        raw_len, raw_turn = self.pathStats_(raw_pts)
        pts = raw_pts

        if self.smoothing_:
            kept = self.shortcutNodes_(nodes)
            pts = self.resample_(
                [self.CRToXY_(n.c, n.r) for n in kept], self.costmap_resolution_
            )
        t_end = perf_counter()

        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"
        for x, y in pts:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg_path.poses.append(pose)
        self.pub_path_.publish(msg_path)

        length, turning = self.pathStats_(pts)
        msg = (
            f"[{name}{'+smooth' if self.smoothing_ else ''}] Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f}) | "
            f"expanded={n_expanded} g_cost={goal_node.g:.2f} length={length:.3f}m turning={turning:.2f}rad "
            f"search={(t_plan - t0) * 1000.0:.1f}ms"
        )
        if self.smoothing_:
            msg += (
                f" smooth={(t_end - t_plan) * 1000.0:.1f}ms "
                f"(raw length={raw_len:.3f}m turning={raw_turn:.2f}rad)"
            )
        self.get_logger().info(msg)

    # Heuristic: octile distance in cells (exact cost on an empty 8-connected grid).
    # Every step costs at least distance * (0 + 1), so this never overestimates (admissible and consistent).
    def heuristic_(self, c, r, goal_c, goal_r):
        dx = abs(c - goal_c)
        dy = abs(r - goal_r)
        return (dx + dy) + (sqrt(2.0) - 2.0) * min(dx, dy)

    # Runs A* based on the world coordinates. Same structure as dijkstra_(), plus a heuristic.
    def astar_(self, start_x, start_y, goal_x, goal_y):
        t0 = perf_counter()
        n_expanded = 0
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)

        if self.outOfMap_(rbt_c, rbt_r) or self.outOfMap_(goal_c, goal_r):
            self.get_logger().warn("Start or goal is outside the map!")
            return

        nodes = [
            AStarNode(i % self.costmap_cols_, i // self.costmap_cols_)
            for i in range(self.costmap_cols_ * self.costmap_rows_)
        ]

        start_node = nodes[self.CRToIndex_(rbt_c, rbt_r)]
        start_node.g = 0.0
        start_node.h = self.heuristic_weight_ * self.heuristic_(rbt_c, rbt_r, goal_c, goal_r)
        start_node.f = start_node.g + start_node.h

        # Entries are (f, h, tie-breaker, node) snapshots, so later updates to a node cannot corrupt the heap order.
        # Ties in f are broken towards the smaller h.
        tie = count()
        open_list = []
        heappush(open_list, (start_node.f, start_node.h, next(tie), start_node))

        while len(open_list) > 0:
            node = heappop(open_list)[3]

            if node.expanded:
                continue
            node.expanded = True
            n_expanded += 1

            if node.c == goal_c and node.r == goal_r:
                self.publishNodePath_(
                    node, "A*", start_x, start_y, goal_x, goal_y, n_expanded, t0
                )
                return

            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                nb_c = node.c + dc
                nb_r = node.r + dr

                if self.outOfMap_(nb_c, nb_r):
                    continue

                nb_idx = self.CRToIndex_(nb_c, nb_r)
                nb_node = nodes[nb_idx]

                if nb_node.expanded:
                    continue

                cost = self.costmap_[nb_idx]
                if cost < 0 or cost > self.max_access_cost_:
                    continue

                new_g = node.g + hypot(dc, dr) * (cost + 1)
                if new_g < nb_node.g:
                    nb_node.g = new_g
                    nb_node.h = self.heuristic_weight_ * self.heuristic_(nb_c, nb_r, goal_c, goal_r)
                    nb_node.f = nb_node.g + nb_node.h
                    nb_node.parent = node
                    heappush(open_list, (nb_node.f, nb_node.h, next(tie), nb_node))

        self.get_logger().warn(f"[A*] No Path Found! Expanded {n_expanded} nodes.")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
