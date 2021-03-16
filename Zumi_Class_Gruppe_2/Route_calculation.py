import networkx as nx
import osmnx as ox
import math
from ZumiRPC import Zumi
from Position import get_distance_between_points, direction_between_points, predict_point

class Navigator(object):
    def __init__(self,zumi,graph):
        """initialize with zumiobject and NetworkX graph"""
        self.zumi = zumi
        self.graph = graph

    def get_nearest_node(self,x,y):
        """get nearest node in graph to given point"""
        node_id = ox.distance.get_nearest_node(self.graph,(y,x),method ='euclidean')
        return node_id

    def get_nearest_edge(self,x,y):
        """get nearest edge in graph to given point
        TODO: Get nearest point on edge to given graph.
        necessity if graph is directional, since driving from nearest node to nearest point on edge might be against given edge direction
        """
        edge_id = ox.distance.get_nearest_edge(self.graph,(y,x))
        return edge_id

    def calc_route(self,x,y):
        """calculate route between current position and node nearest to destination point"""
        pos,direction = self.zumi.get_pos_and_dir()
        startnode = self.get_nearest_node(pos[0],pos[1])
        endnode = self.get_nearest_node(x,y)
        path = nx.shortest_path(self.graph,startnode,endnode,weight = 'distance')
        return path

    def get_nearest_point_on_edge(self,edge,x,y):
        """NOT TESTED YET
        Gives nearest point on an edge to given point"""
        old_x = edge[0]['x']
        old_y = edge[0]['y']
        new_x = edge[1]['x']
        new_y = edge[1]['y']
        angle = direction_between_points(new_x,new_y,old_x,old_y)
        dist = get_distance_between_points(new_x,new_y,old_x,old_y)
        sh_dist = 10000000
        sh_x = -1
        sh_y = -1
        def get_nearest_point(point_x,point_y):
            current_dist = get_distance_between_points(point_x,point_y,x,y)
            if(current_dist<sh_dist):
                sh_dist = current_dist
                sh_x = point_x
                sh_y = point_y
        get_nearest_point(old_x,old_y)
        get_nearest_point(new_x,new_y)
        for i in range(0,dist):
            point = predict_point([new_x,new_y],angle,dist)
            get_nearest_point(point[0],point[1])
        return sh_x,sh_y
        
    def drive_towards(self,x,y):
        """Calculates path towards destination point, drives zumi towards point and stops at graph node nearest to destination"""
        path = self.calc_route(x,y)
        for node_id in path:
            not_close_enough = True
            i = 0
            while(not_close_enough):
                node_x = self.graph.nodes[node_id]['x']
                node_y = self.graph.nodes[node_id]['y']
                distance = self.zumi.get_distance_to(node_x,node_y)
                """check if close enought to point, since drift and other errors might interfere"""
                if(distance>20):
                    if(i<2):
                        self.zumi.drive_towards(node_x,node_y)
                    else:
                        not_close_enough = False
                else:
                    not_close_enough = False
        return 
    


        


