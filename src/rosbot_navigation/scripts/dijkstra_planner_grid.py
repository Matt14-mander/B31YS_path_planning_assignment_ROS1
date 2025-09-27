#!/usr/bin/env python3

import rospy
import hashlib
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
        self.inflate_obstacles(inflation_radius=3)  # Adjust radius as needed 

        # Define world coordinates for start position in meters
        start_world = (0.0, 0.0)  # Example start position

        # Get student's name from parameter
        student_name = rospy.get_param("~student_name", "default_student")
        print(student_name)
        # Try to find a valid goal based on the student's name
        attempt = 0
        goal_grid = None
        while not goal_grid and attempt < 100:  # Limit attempts to avoid infinite loop
            goal_grid = self.get_goal_from_name(student_name, attempt)
            if not self.is_free(goal_grid) or not self.is_reachable(goal_grid):
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

        # Check that start is free
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
        """
        Callback function to receive the map data from the /map topic.
        """
        self.map_data = msg

    def get_goal_from_name(self, student_name, attempt=0):
        """
        Generate a goal position based on the student's name.
        Uses attempt number to avoid infinite loops with invalid goals.
        
        :param student_name: Name of the student
        :param attempt: Attempt number to modify hash input
        :return: Goal position in grid coordinates (gx, gy)
        """
        # Modify input for different attempts to avoid infinite loops
        hash_input = f"{student_name}_{attempt}"
        hash_value = int(hashlib.md5(hash_input.encode()).hexdigest(), 16)

        # Map the hash value to coordinates (ensure they are within the grid size)
        width = self.map_data.info.width
        height = self.map_data.info.height

        # Generate a valid position on the grid
        goal_x = (hash_value % width)
        goal_y = (hash_value // width) % height

        return (goal_x, goal_y)

    def is_reachable(self, goal):
        """
        Check if the goal is reachable from the start point using BFS
        
        :param goal: Goal position in grid coordinates
        :return: True if goal is reachable, False otherwise
        """
        start = self.world_to_grid((0.0, 0.0), self.map_data.info)  # Convert start to grid
        
        # Use BFS to check if goal is reachable
        queue = deque([start])
        visited = set([start])
        
        while queue:
            current = queue.popleft()
            
            if current == goal:
                return True  # Goal is reachable
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        return False  # Goal is not reachable

    def world_to_grid(self, world, map_info):
        """
        Convert world coordinates to grid coordinates.
        
        :param world: World coordinates (x, y) in meters
        :param map_info: The map's metadata (resolution, origin, etc.)
        :return: Corresponding grid cell coordinates (gx, gy)
        """
        res = map_info.resolution
        origin = map_info.origin.position
        gx = int((world[0] - origin.x) / res)
        gy = int((world[1] - origin.y) / res)
        return (gx, gy)

    def grid_to_world(self, cell):
        """
        Convert grid cell coordinates to world coordinates.
        
        :param cell: Grid cell coordinates (gx, gy)
        :return: Corresponding world coordinates (wx, wy) in meters
        """
        res = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        wx = cell[0] * res + origin.x + res / 2.0
        wy = cell[1] * res + origin.y + res / 2.0
        return (wx, wy)
        
        
    def inflate_obstacles(self, inflation_radius=2):
        """
        Inflate obstacles by a given radius to create safety margins.
        
        :param inflation_radius: Number of cells to inflate around obstacles
        """
        if not self.map_data:
            return
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        # Create a copy of the original map data
        inflated_data = list(self.map_data.data)
        
        # Find all occupied cells (value > 0)
        occupied_cells = []
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if self.map_data.data[idx] > 0:  # Occupied or unknown
                    occupied_cells.append((x, y))
        
        # Inflate around each occupied cell
        for occ_x, occ_y in occupied_cells:
            for dy in range(-inflation_radius, inflation_radius + 1):
                for dx in range(-inflation_radius, inflation_radius + 1):
                    new_x = occ_x + dx
                    new_y = occ_y + dy
                    
                    # Check if within bounds
                    if 0 <= new_x < width and 0 <= new_y < height:
                        # Check if within circular radius
                        distance = (dx**2 + dy**2)**0.5
                        if distance <= inflation_radius:
                            idx = new_y * width + new_x
                            # Only inflate if it was originally free space
                            if self.map_data.data[idx] == 0:
                                inflated_data[idx] = 99  # Mark as inflated obstacle
        
        # Update the map data
        self.map_data.data = tuple(inflated_data)

    def is_free(self, cell):
        """
        Check if a cell is free (not occupied).
        
        :param cell: Grid cell coordinates (gx, gy)
        :return: True if the cell is free, False otherwise
        """
        width = self.map_data.info.width
        height = self.map_data.info.height
        x, y = cell
        if 0 <= x < width and 0 <= y < height:
            value = self.map_data.data[y * width + x]
            return value == 0  # 0 indicates free space
        return False

    def get_neighbors(self, cell):
        """
        Get the valid neighbors of a given cell (up, down, left, right).
        
        :param cell: Grid cell coordinates (gx, gy)
        :return: List of valid neighboring cells
        """
        x, y = cell
        neighbors = [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]
        return [n for n in neighbors if self.is_free(n)]

    def build_graph(self):
        """
        Build an adjacency list representation of the grid map.
        Only includes free cells and their valid neighbors.
        
        :return: Graph (adjacency list), where keys are cells and values are lists of neighboring cells
        """
        graph = {}
        width = self.map_data.info.width
        height = self.map_data.info.height

        for y in range(height):
            for x in range(width):
                if self.is_free((x, y)):  # Only include free cells
                    node = (x, y)
                    graph[node] = self.get_neighbors(node)  # Get neighbors for each free cell

        return graph

    def backtrace(self, parent, start, goal):
        """
        Reconstruct the path from the start to the goal by backtracking from the goal.
        
        :param parent: Dictionary mapping each cell to its parent
        :param start: The start cell
        :param goal: The goal cell
        :return: List of cells representing the path from start to goal
        """
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()  # Reverse the path to get it from start to goal
        return path

    def dijkstra(self, graph, start, goal):
        """
        Perform Dijkstra's algorithm to find the shortest path from start to goal.
        
        :param graph: The adjacency list graph
        :param start: The start cell
        :param goal: The goal cell
        :return: A tuple (found, path), where 'found' is True if path is found,
                 and 'path' is the list of cells representing the path
        """
        # Get ALL nodes (both keys and values)
        all_nodes = set(graph.keys())
        for neighbors in graph.values():
            all_nodes.update(neighbors)
        
        # Initialize distances for ALL nodes
        distances = {}
        for node in all_nodes:
            distances[node] = float('inf')
        distances[start] = 0
        
        # Keep track of unvisited nodes
        unvisited = list(all_nodes)  # Use all nodes, not just graph.keys()
        
        # Keep track of parents for path reconstruction
        parent = {}
        
        while unvisited:
            # Find the unvisited node with minimum distance
            # This replaces the heap functionality
            current_node = None
            for node in unvisited:
                if current_node is None or distances[node] < distances[current_node]:
                    current_node = node
            
            # Remove current node from unvisited
            unvisited.remove(current_node)
            
            # If we reached the goal, we're done
            if current_node == goal:
                return True, self.backtrace(parent, start, current_node)
            
            # Check all neighbors of current node
            for neighbor in graph.get(current_node, []):
                if neighbor in unvisited:  # Only consider unvisited neighbors
                    # Calculate new distance through current node
                    new_distance = distances[current_node] + 1  # Each edge has weight 1
                    
                    # If this path is shorter, update it
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        parent[neighbor] = current_node
        
        # No path found
        return False, []

    def publish_path(self, path_cells):
        """
        Publish the planned path as a Path message for visualization in RViz.
        
        :param path_cells: List of grid cells representing the planned path
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        # Convert each grid cell in the path to a PoseStamped message
        for cell in path_cells:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            wx, wy = self.grid_to_world(cell)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # Publish the path to the /planned_path topic
        self.path_pub.publish(path)


if __name__ == '__main__':
    try:
        DijkstraPlanner()
    except rospy.ROSInterruptException:
        pass
