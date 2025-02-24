import time
import heapq
import numpy as np
from .utils import *
from .parameter import *
from .quads import QuadTree



##似乎只能处理单体坐标
class NodeManager:
    ## 初始化 NodeManager 实例，设置四叉树存储节点，初始化可视化标志和前沿点集合。
    def __init__(self, plot=False):
        self.nodes_dict = QuadTree((0, 0), 1000, 1000)
        self.frontier = None

    def check_node_exist_in_dict(self, coords):
        key = (coords[0], coords[1])
        exist = self.nodes_dict.find(key)
        #print(f"[check_node_exist_in_dict] Looking for key={key}, found={exist}")
        #print(f"[{time.time()}][PID={os.getpid()}] check_node_exist_in_dict -> Looking for key={key}, found={exist}")
        return exist

    def add_node_to_dict(self, coords, frontiers, updating_map_info, map_info):
        key = (coords[0], coords[1])
        node = LocalNode(coords, frontiers, updating_map_info, map_info)
        self.nodes_dict.insert(point=key, data=node)
        
        #print(f"[add_node_to_dict] Inserted node with key={key}, node={node}")
        #print(f"[{time.time()}][PID={os.getpid()}] add_node_to_dict -> Inserted key={key}, node={node}")
        return node

    
    def frontiers_detect(self, map_info):
        height=map_info.map.shape[0]
        width=map_info.map.shape[1]
        obstacles=[]
        frontiers=[]
        for i in range(2, height-2):
            for j in range(2, width-2):
                if map_info.map[i][j]==0:
                   obstacles.append([i,j])
                elif map_info.map[i][j] == 205:
                    numFree = 0
                    temp1 = 0
                    if map_info.map[i+1][j] == 254:
                        temp1 += 1 if map_info.map[i+2][j] == 254 else 0
                        temp1 += 1 if map_info.map[i+1][j+1] == 254 else 0
                        temp1 += 1 if map_info.map[i+1][j-1] == 254 else 0
                        numFree += (temp1 > 0)
                    if map_info.map[i][j+1] == 254:
                        temp1 += 1 if map_info.map[i][j+2] == 254 else 0
                        temp1 += 1 if map_info.map[i+1][j+1] == 254 else 0
                        temp1 += 1 if map_info.map[i-1][j+1] == 254 else 0
                        numFree += (temp1 > 0)     
                    if map_info.map[i-1][j] == 254:
                        temp1 += 1 if map_info.map[i-1][j+1] == 254 else 0
                        temp1 += 1 if map_info.map[i-1][j-1] == 254 else 0
                        temp1 += 1 if map_info.map[i-2][j] == 254 else 0
                        numFree += (temp1 > 0)
                    if map_info.map[i][j-1] == 254:
                        temp1 += 1 if map_info.map[i][j-2] == 254 else 0
                        temp1 += 1 if map_info.map[i+1][j-1] == 254 else 0
                        temp1 += 1 if map_info.map[i-1][j-1] == 254 else 0
                        numFree += (temp1 > 0)     
                    if numFree > 0:
                        frontiers.append([i,j])
        if frontiers:
          frontiers = np.array(frontiers, dtype=np.int32).reshape(-1, 2)
          frontier_coords = get_coords_from_cell_position(frontiers, map_info)
          if frontier_coords.ndim ==1:
              frontier_coords = np.array([frontier_coords])
          fr_coords = set(map(tuple, frontier_coords))
        else:
          fr_coords = set()
        #print(f"[frontiers_detect] frontier_coords: {frontier_coords}")
        return fr_coords
    
    def remove_node_from_dict(self, node):
        for neighbor_coords in node.neighbor_set:
            if neighbor_coords != (node.coords[0], node.coords[1]):
                neighbor_node = self.nodes_dict.find(neighbor_coords)
                neighbor_node.data.neighbor_set.remove(node.coords.tolist())
        self.nodes_dict.remove(node.coords.tolist())
        
    ##更新节点图结构，包括添加新节点、更新现有节点的前沿和效用，以及更新节点的邻居关系。
    def update_graph(self, robot_location, frontiers, updating_map_info, map_info):
        node_coords, _ = get_updating_node_coords(robot_location, updating_map_info, map_info)
        #print(f"[update_graph] node_coords: {node_coords}")
        if self.frontier is None:
            new_frontier = frontiers
        else:
            new_frontier = frontiers - self.frontier
            new_out_range= []
            for frontier in new_frontier:
                if np.linalg.norm(robot_location - np.array(frontier).reshape(2)) > SENSOR_RANGE + FRONTIER_CELL_SIZE:
                    new_out_range.append(frontier)
            for frontier in new_out_range:
                new_frontier.remove(frontier)
        self.frontier = frontiers
        all_node_list = []
        global_frontiers = self.frontiers_detect(map_info)
        for coords in node_coords:
            node = self.check_node_exist_in_dict(coords)
            if node is None:
                #print(f"[update_graph] Adding new node at coords: {coords}")
                node = self.add_node_to_dict(coords, frontiers, updating_map_info, map_info)
            else:
                node = node.data
                if node.utility == 0 or np.linalg.norm(node.coords - robot_location) > 2 * SENSOR_RANGE:
                    pass
                else:
                    node.update_node_observable_frontiers(new_frontier, global_frontiers, updating_map_info, map_info)
            all_node_list.append(node)

        for node in all_node_list:
            if node.need_update_neighbor and np.linalg.norm(node.coords - robot_location) < (
                    SENSOR_RANGE + NODE_RESOLUTION):
                node.update_neighbor_nodes(updating_map_info, self.nodes_dict, map_info)

class LocalNode:
    def __init__(self, coords, frontiers, updating_map_info, map_info):
        self.coords = coords
        self.utility_range = UTILITY_RANGE
        self.utility = 0
        self.observable_frontiers = self.initialize_observable_frontiers(frontiers, updating_map_info, map_info)
        self.visited = 0

        self.neighbor_matrix = -np.ones((5, 5))
        self.neighbor_set = set()
        self.neighbor_matrix[2, 2] = 1
        self.neighbor_set.add((self.coords[0], self.coords[1]))
        self.need_update_neighbor = True

    def initialize_observable_frontiers(self, frontiers, updating_map_info, map_info):
        ##判断前沿点是否为空
        if len(frontiers) == 0:
            self.utility = 0
            return set()
        else:
            observable_frontiers = set()
            frontiers = np.array(list(frontiers)).reshape(-1, 2)
            dist_list = np.linalg.norm(frontiers - self.coords, axis=-1)
            new_frontiers_in_range = frontiers[dist_list < self.utility_range]
            #print(f"[initialize_observable_frontiers] frontiers: {frontiers}")
            #print(f"[initialize_observable_frontiers] new_frontiers_in_range: {new_frontiers_in_range}")
            
            for point in new_frontiers_in_range:
                if not in_local_map_range(point, updating_map_info):
                    continue
                print(f"[initialize_observable_frontiers] point: {point}")
                print(f"[initialize_observable_frontiers] self.coords: {self.coords}")
                collision = check_local_collision(self.coords, point, updating_map_info, map_info)
                #print(f"[initialize_observable_frontiers] collision: {collision}")
                if not collision:
                    observable_frontiers.add((point[0], point[1]))
            self.utility = len(observable_frontiers)
            if self.utility <= MIN_UTILITY:
                self.utility = 0
                observable_frontiers = set()
            return observable_frontiers

    def update_neighbor_nodes(self, updating_map_info, nodes_dict, map_info):
        for i in range(self.neighbor_matrix.shape[0]):
            for j in range(self.neighbor_matrix.shape[1]):
                if self.neighbor_matrix[i, j] != -1:
                    continue
                else:
                    center_index = self.neighbor_matrix.shape[0] // 2
                    if i == center_index and j == center_index:
                        self.neighbor_matrix[i, j] = 1
                        # self.neighbor_list.append(self.coords)
                        continue

                    neighbor_coords = np.around(np.array([self.coords[0] + (i - center_index) * NODE_RESOLUTION,
                                                self.coords[1] + (j - center_index) * NODE_RESOLUTION]), 1)
                    
                    if not in_global_map_range(neighbor_coords, map_info):
                        continue
                    #print(f"[update_neighbor_nodes] neighbor_coords: {neighbor_coords}")
                    neighbor_node = nodes_dict.find((neighbor_coords[0], neighbor_coords[1]))
                    if neighbor_node is None:
                        #cell = get_cell_position_from_coords(neighbor_coords, updating_map_info)
                        #if updating_map_info.map[cell[1], cell[0]] == 1:
                        #    self.neighbor_matrix[i, j] = 1
                        continue
                    else:
                        neighbor_node = neighbor_node.data
                        collision = check_collision(self.coords, neighbor_coords, map_info)
                        neighbor_matrix_x = center_index + (center_index - i)
                        neighbor_matrix_y = center_index + (center_index - j)
                        if not collision:
                            self.neighbor_matrix[i, j] = 1
                            self.neighbor_set.add((neighbor_coords[0], neighbor_coords[1]))

                            neighbor_node.neighbor_matrix[neighbor_matrix_x, neighbor_matrix_y] = 1
                            neighbor_node.neighbor_set.add((self.coords[0], self.coords[1]))

        if self.utility == 0:
            self.need_update_neighbor = False

    def update_node_observable_frontiers(self, new_frontiers, global_frontiers, updating_map_info, map_info):
        # remove frontiers observed
        frontiers_observed = []
        for frontier in self.observable_frontiers:
            if frontier not in global_frontiers:
                frontiers_observed.append(frontier)
        for frontier in frontiers_observed:
            self.observable_frontiers.remove(frontier)

        # add new frontiers in the observable frontiers
        if len(new_frontiers) > 0:
            new_frontiers = np.array(list(new_frontiers)).reshape(-1, 2)
            dist_list = np.linalg.norm(new_frontiers - self.coords, axis=-1)
            new_frontiers_in_range = new_frontiers[dist_list < self.utility_range]
            
            for point in new_frontiers_in_range:
                if not in_local_map_range(point, updating_map_info):
                    continue
                
                collision = check_local_collision(self.coords, point, updating_map_info, map_info)
                if not collision:
                    self.observable_frontiers.add((point[0], point[1]))

        self.utility = len(self.observable_frontiers)
        if self.utility <= MIN_UTILITY:
            self.utility = 0
            self.observable_frontiers = set()

    def set_visited(self):
        self.visited = 1
        self.observable_frontiers = set()
        self.utility = 0

