#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GraphVisualizer:
    def __init__(self):
        rospy.init_node('graph_visualizer')
        start_delay = rospy.get_param("~start_delay", 0.0)
        rospy.sleep(start_delay)
        self.map_data = None
        self.graph = {}
        self.node_pub = rospy.Publisher('/graph_nodes', Marker, queue_size=1)
        self.edge_pub = rospy.Publisher('/graph_edges', Marker, queue_size=1)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        rospy.loginfo("Waiting for map...")
        while self.map_data is None and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.loginfo("Map received!")

        self.build_graph()
        while not rospy.is_shutdown():
            self.publish_graph()
            rospy.loginfo("Graph markers published.")
            rospy.sleep(10.)

    def map_callback(self, msg):
        self.map_data = msg

    def is_free(self, x, y):
        width = self.map_data.info.width
        height = self.map_data.info.height
        index = y * width + x
        if 0 <= x < width and 0 <= y < height:
            return self.map_data.data[index] == 0
        return False

    def grid_to_world(self, x, y):
        res = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        wx = x * res + origin.x + res / 2.0
        wy = y * res + origin.y + res / 2.0
        return wx, wy

    def build_graph(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        for y in range(height):
            for x in range(width):
                if self.is_free(x, y):
                    node = (x, y)
                    neighbors = []
                    for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                        nx, ny = x+dx, y+dy
                        if self.is_free(nx, ny):
                            neighbors.append((nx, ny))
                    self.graph[node] = neighbors

    def publish_graph(self):
        # Marker for nodes
        node_marker = Marker()
        node_marker.header.frame_id = "map"
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.03
        node_marker.scale.y = 0.03
        node_marker.scale.z = 0.01
        node_marker.color.r = 0.0
        node_marker.color.g = 1.0
        node_marker.color.b = 0.0
        node_marker.color.a = 1.0

        # Marker for edges
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.01
        edge_marker.color.r = 1.0
        edge_marker.color.g = 0.0
        edge_marker.color.b = 0.0
        edge_marker.color.a = 0.7

        for node, neighbors in self.graph.items():
            wx, wy = self.grid_to_world(*node)
            point = Point()
            point.x = wx
            point.y = wy
            point.z = 0.0
            node_marker.points.append(point)

            for neighbor in neighbors:
                nwx, nwy = self.grid_to_world(*neighbor)
                p1 = Point(x=wx, y=wy, z=0.0)
                p2 = Point(x=nwx, y=nwy, z=0.0)
                edge_marker.points.append(p1)
                edge_marker.points.append(p2)

        rospy.sleep(1.0)
        self.node_pub.publish(node_marker)
        self.edge_pub.publish(edge_marker)
        rospy.loginfo("Graph markers published.")

if __name__ == "__main__":
    try:
        GraphVisualizer()
    except rospy.ROSInterruptException:
        pass

