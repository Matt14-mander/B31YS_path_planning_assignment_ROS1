#!/usr/bin/env python3
import rospy
import hashlib
import heapq
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math


class DijkstraPlanner:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dijkstra_planner')

        # Store map data
        self.map_data = None

        # Subscriber to receive the map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publisher to publish the planned path
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)

        # Wait for the map to be received
        rospy.loginfo("Waiting for map...")
        while self.map_data is None and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.loginfo("Map received!")

        # Inflate obstacles for safety
        self.inflate_obstacles(inflation_radius=3)  # 可按需要调整

        # Define world coordinates for start position in meters
        start_world = (0.0, 0.0)  # 示例起点

        # Get student's name from parameter
        student_name = rospy.get_param("~student_name", "default_student")
        rospy.loginfo("Student: %s", student_name)

        # Try to find a valid goal based on the student's name
        attempt = 0
        goal_grid = None
        while not goal_grid and attempt < 100:  # 限制尝试次数
            goal_grid = self.get_goal_from_name(student_name, attempt)
            if (not self.is_free(goal_grid)) or (not self.is_reachable(goal_grid)):
                rospy.logwarn(f"Generated goal attempt {attempt} is invalid. Trying again...")
                goal_grid = None
                attempt += 1

        if not goal_grid:
            rospy.logerr("Could not find valid goal after 100 attempts!")
            return

        rospy.loginfo(f"Goal position for {student_name}: {goal_grid}")

        # Convert world coordinates to grid cell coordinates
        start = self.world_to_grid(start_world, self.map_data.info)
        goal = goal_grid  # Already in grid coordinates

        rospy.loginfo(f"Planning from {start} to {goal}...")

        # Check that start/goal are free
        if not self.is_free(start):
            rospy.logwarn("Start is not in free space.")
            return

        # Build graph (adjacency list representation of the map)
        graph = self.build_graph()

        # Run Dijkstra's algorithm to find the path
        found, path = self.dijkstra(graph, start, goal)

        if found:
            rospy.loginfo(f"Path found with {len(path)} steps.")
            # Publish the found path
            self.publish_path(path)
        else:
            rospy.logwarn("No path found!")

        # Keep the node running
        rospy.spin()

    def map_callback(self, msg):
        """Callback function to receive the map data from the /map topic."""
        self.map_data = msg

    def get_goal_from_name(self, student_name, attempt=0):
        """
        Generate a goal position based on the student's name.
        Uses attempt number to avoid infinite loops with invalid goals.
        """
        hash_input = f"{student_name}_{attempt}"
        hash_value = int(hashlib.md5(hash_input.encode()).hexdigest(), 16)

        width = self.map_data.info.width
        height = self.map_data.info.height

        goal_x = (hash_value % width)
        goal_y = (hash_value // width) % height
        return (goal_x, goal_y)

    def is_reachable(self, goal):
        """
        Check if the goal is reachable from the start point using BFS on free cells.
        """
        start = self.world_to_grid((0.0, 0.0), self.map_data.info)  # Convert start to grid

        queue = deque([start])
        visited = set([start])

        while queue:
            current = queue.popleft()
            if current == goal:
                return True

            for nbr, _w in self.get_neighbors(current, diagonal=True):  # 兼容权重返回
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append(nbr)

        return False

    def world_to_grid(self, world, map_info):
        """Convert world (meters) to grid (cells)."""
        res = map_info.resolution
        origin = map_info.origin.position
        gx = int((world[0] - origin.x) / res)
        gy = int((world[1] - origin.y) / res)
        return (gx, gy)

    def grid_to_world(self, cell):
        """Convert grid (cells) to world (meters)."""
        res = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        wx = cell[0] * res + origin.x + res / 2.0
        wy = cell[1] * res + origin.y + res / 2.0
        return (wx, wy)

    def inflate_obstacles(self, inflation_radius=2):
        """
        Inflate obstacles by a given radius to create safety margins.
        Marks inflated cells as 99 (occupied-like).
        """
        if not self.map_data:
            return

        width = self.map_data.info.width
        height = self.map_data.info.height

        inflated_data = list(self.map_data.data)

        # Occupied (>0) treated as obstacle; unknown (-1) remains unchanged here
        occupied_cells = []
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if self.map_data.data[idx] > 0:
                    occupied_cells.append((x, y))

        for ox, oy in occupied_cells:
            for dy in range(-inflation_radius, inflation_radius + 1):
                for dx in range(-inflation_radius, inflation_radius + 1):
                    nx = ox + dx
                    ny = oy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if dx * dx + dy * dy <= inflation_radius * inflation_radius:
                            idx = ny * width + nx
                            if inflated_data[idx] == 0:
                                inflated_data[idx] = 99

        self.map_data.data = tuple(inflated_data)

    def is_free(self, cell):
        """
        Check if a cell is free (0). Treat unknown (-1) and inflated/occupied (>0) as not free.
        """
        width = self.map_data.info.width
        height = self.map_data.info.height
        x, y = cell
        if 0 <= x < width and 0 <= y < height:
            val = self.map_data.data[y * width + x]
            return val == 0
        return False

    def get_neighbors(self, cell, diagonal=False):
        """
        Get the valid neighbors of a given cell (up, down, left, right).
        
        :param cell: Grid cell coordinates (gx, gy)
        :return: List of valid neighboring cells
        """
        x, y = cell
        neighbors = []

        candidates = [(x+1,y,1.0), (x-1,y,1.0), (x,y+1,1.0), (x,y-1,1.0)]
        if diagonal:
            rt2 = math.sqrt(2)
            candidates += [(x+1,y+1,rt2), (x-1,y+1,rt2), (x+1,y-1,rt2), (x-1,y-1,rt2)]
        for nx, ny, cost in candidates:
            if self.is_free((nx, ny)):
                neighbors.append(((nx, ny), cost))

        return neighbors

    def build_graph(self, diagonal=True):
        """
        Build an adjacency list of free cells.
        Graph format: { node: [ (neighbor, weight), ... ] }
        """
        graph = {}
        width = self.map_data.info.width
        height = self.map_data.info.height

        for y in range(height):
            for x in range(width):
                if self.is_free((x, y)):
                    node = (x, y)
                    graph[node] = self.get_neighbors(node, diagonal=diagonal)
        return graph

    def backtrace(self, parent, start, goal):
        """Reconstruct path from start to goal using parent dict."""
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def dijkstra(self, graph, start, goal):
        """
        Dijkstra with a min-heap, supports weighted edges.
        graph: {u: [ (v, w), ... ] } or {u: [v1, v2, ...]} (defaults weight=1)
        """
       # Get ALL nodes (both keys and values)
        all_nodes = set(graph.keys())
        for neighbors in graph.values():
            for item in neighbors:
                if isinstance(item, tuple):
                    v = item[0]
                else:
                    v = item
            all_nodes.add(v)

        if start not in all_nodes or goal not in all_nodes:
            return False, []

        if start == goal:
            return True, [start]
        
        # Initialize distances for ALL nodes
        distances = {}
        for node in all_nodes:
            distances[node] = float('inf')
        distances[start] = 0
        
        # Keep track of unvisited nodes
        # unvisited = list(all_nodes)  # Use all nodes, not just graph.keys()
        
        # Keep track of parents for path reconstruction
        parent = {}

        heap = [(0.0, start)]
        while heap:
            d_u, u = heapq.heappop(heap)
            if d_u > distances[u]:
                continue

            if u == goal:
                path = self.backtrace(parent, start, goal)
                return (len(path) > 0, path)

            for item in graph.get(u, []):
                if isinstance(item, (tuple, list)):
                    v, w = item[0], float(item[1])
                else:
                    v, w = item, 1.0

                if w < 0:
                    # Dijkstra requires non-negative weights
                    continue

                alt = d_u + w
                if alt < distances[v]:
                    distances[v] = alt
                    parent[v] = u
                    heapq.heappush(heap, (alt, v))

        return False, []

    def publish_path(self, path_cells):
        """Publish the planned path as a Path message for RViz."""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for cell in path_cells:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            wx, wy = self.grid_to_world(cell)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


if __name__ == '__main__':
    try:
        DijkstraPlanner()
    except rospy.ROSInterruptException:
        pass
