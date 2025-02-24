import numpy as np
import imageio
import os
from skimage.morphology import label
from .parameter import *

##将世界坐标（coords）转换为地图网格单元的位置（cell_position）。这在将连续的世界坐标映射到离散的网格单元时非常有用，例如在网格地图中标记障碍物或路径。
def get_cell_position_from_coords(coords, map_info, check_negative=True):
    coords = np.array(coords)  # 确保 coords 是 NumPy 数组
    single_cell = False
    if coords.flatten().shape[0] == 2:
        single_cell = True

    coords = coords.reshape(-1, 2)
    coords_x = coords[:, 0]
    coords_y = coords[:, 1]
    
    #print(f"[get_cell_position_from_coords] Input coords: {coords}")
    #print(f"[get_cell_position_from_coords] map_origin_x: {map_info.map_origin_x}, map_origin_y: {map_info.map_origin_y}, cell_size: {map_info.cell_size}")
    
    ##假设坐标原点是地图中心
    
    cell_x = (coords_x / map_info.cell_size) + map_info.map.shape[0]//2
    cell_y = (coords_y / map_info.cell_size) + map_info.map.shape[1]//2
    #print(f'[get_cell_position_from_coords] map_info_shape[0]: {map_info.map.shape[0]//2}, map_info_shape[1]: {map_info.map.shape[1]//2}')
    #print(f"[get_cell_position_from_coords] Calculated cell_x: {cell_x}, cell_y: {cell_y}")
    cell_position = np.around(np.stack((cell_x, cell_y), axis=-1)).astype(int)
    #print(f"[get_cell_position_from_coords] Rounded cell_position: {cell_position}")
    if check_negative:
        assert sum(cell_position.flatten() >= 0) == cell_position.flatten().shape[0], print(cell_position, coords, map_info.map_origin_x, map_info.map_origin_y)
    if single_cell:
        return cell_position[0]
    else:
        return cell_position

##将地图网格单元的位置（cell_position）转换为世界坐标（coords）。这在从网格地图获取具体位置时非常有用，例如在路径规划后将路径的网格单元转换为实际的移动坐标。
## cell_position是数组格式！
def get_coords_from_cell_position(cell_position, map_info):
    cell_position = cell_position.reshape(-1, 2)
    cell_x = cell_position[:, 0]
    cell_y = cell_position[:, 1]
    
    #print(f"[get_coords_from_cell_position] Input cell_position: {cell_position}")
    #print(f"[get_coords_from_cell_position] map_origin_x: {map_info.map_origin_x}, map_origin_y: {map_info.map_origin_y}, cell_size: {map_info.cell_size}")

    coords_x = (cell_x - map_info.map.shape[0]//2) * map_info.cell_size
    coords_y = (cell_y - map_info.map.shape[1]//2) * map_info.cell_size
    coords = np.stack((coords_x, coords_y), axis=-1)
    coords = np.around(coords, 1)
    #print(f"[get_coords_from_cell_position] Calculated coords: {coords}")

    if coords.shape[0] == 1:
        #print(f"[get_coords_from_cell_position] Output coords (single): {coords[0]}")
        return coords[0]
    else:
        #print(f"[get_coords_from_cell_position] Output coords (batch): {coords}")
        return coords

##该函数用于获取地图中所有标记为自由区域（FREE）的坐标。具体来说，它将地图中的自由单元格（free cells）转换为世界坐标系中的实际坐标。
def get_free_area_coords(map_info):
    free_indices = np.where(map_info.map == FREE)
    free_cells = np.asarray([free_indices[1], free_indices[0]]).T
    free_coords = get_coords_from_cell_position(free_cells, map_info)
    return free_coords

##该函数用于获取地图中与给定位置（location）连接的所有自由区域。具体来说，它生成一个二进制地图，标记出与指定位置连通的自由区域。
def get_free_and_connected_map(location, map_info):
    print(f"[get_free_and_connected_map] map_info.map unique values: {np.unique(map_info.map)}")
    
    # a binary map for free and connected areas
    free = (map_info.map == FREE).astype(float)
    
     # 检查是否存在 free 区域
    if free.sum() == 0:
        raise ValueError("No free areas in the map!")
    
    labeled_free = label(free, connectivity=2)
    # 检查 labeled_free 是否有效
    if labeled_free.size == 0:
        raise ValueError("No labeled regions found!")
    
    cell = get_cell_position_from_coords(location, map_info)
    
       # 检查 cell 是否在 labeled_free 的范围内
    if not (0 <= cell[1] < labeled_free.shape[0] and 0 <= cell[0] < labeled_free.shape[1]):
        raise ValueError(f"Cell {cell} is out of bounds for labeled_free with shape {labeled_free.shape}")
    
    label_number = labeled_free[cell[1], cell[0]]
    connected_free_map = (labeled_free == label_number)
    return connected_free_map


##该函数的主要目的是生成地图中所有可能的节点坐标，并根据是否需要检查连通性（check_connectivity）筛选出自由且连通的节点。
##这在节点网络构建和更新过程中至关重要，尤其是在动态环境中需要实时更新节点信息时。
def get_updating_node_coords(location, updating_map_info, map_info, check_connectivity=False):
    # 检查 updating_map_info 中是否有 FREE 区域
    if np.any(updating_map_info.map == FREE):
        print("Updating map contains free areas.")
    else:
        print("No free areas in the updating map.")
    ##传入世界坐标
    x_min = updating_map_info.map_origin_x
    y_min = updating_map_info.map_origin_y
    x_max = updating_map_info.map_origin_x + (updating_map_info.map.shape[1] - 1) * CELL_SIZE
    y_max = updating_map_info.map_origin_y + (updating_map_info.map.shape[0] - 1) * CELL_SIZE

    if x_min % NODE_RESOLUTION != 0:
        x_min = (x_min // NODE_RESOLUTION + 1) * NODE_RESOLUTION
    if x_max % NODE_RESOLUTION != 0:
        x_max = x_max // NODE_RESOLUTION * NODE_RESOLUTION
    if y_min % NODE_RESOLUTION != 0:
        y_min = (y_min // NODE_RESOLUTION + 1) * NODE_RESOLUTION
    if y_max % NODE_RESOLUTION != 0:
        y_max = y_max // NODE_RESOLUTION * NODE_RESOLUTION
    
    x_global_min = -(map_info.map.shape[1] * CELL_SIZE) / 2
    y_global_min = -(map_info.map.shape[0] * CELL_SIZE) / 2
    x_global_max = (map_info.map.shape[1] * CELL_SIZE) / 2
    y_global_max = (map_info.map.shape[0] * CELL_SIZE) / 2
    
    x_min = max(x_min, x_global_min)
    x_max = min(x_max, x_global_max)
    y_min = max(y_min, y_global_min)
    y_max = min(y_max, y_global_max)
        
    x_coords = np.arange(x_min, x_max + 0.1, NODE_RESOLUTION)
    y_coords = np.arange(y_min, y_max + 0.1, NODE_RESOLUTION)
    t1, t2 = np.meshgrid(x_coords, y_coords)
    nodes = np.vstack([t1.T.ravel(), t2.T.ravel()]).T
    nodes = np.around(nodes, 1)

    free_connected_map = None

    if not check_connectivity:

        indices = []
        nodes_cells = get_cell_position_from_coords(nodes, map_info).reshape(-1, 2)
        #print(f'[get_updating_node_coords] nodes_cells: {nodes_cells}')
        #print(f'[get_updating_node_coords] updating_map_info.map.shape:{updating_map_info.map.shape}')
        for i, (x_g, y_g) in enumerate(nodes_cells):
            [offset_x, offset_y] = get_cell_position_from_coords([updating_map_info.map_origin_x, updating_map_info.map_origin_y], map_info)
           
            x_l = x_g -offset_x
            y_l=  y_g -offset_y
            #assert 0 <= cell[1] < updating_map_info.map.shape[0] and 0 <= cell[0] < updating_map_info.map.shape[1]
            if updating_map_info.map[y_l, x_l] == FREE:
                indices.append(i)
        indices = np.array(indices, dtype=int)
        nodes = nodes[indices].reshape(-1, 2)

    else:
        free_connected_map = get_free_and_connected_map(location, updating_map_info)
        free_connected_map = np.array(free_connected_map)

        indices = []
        nodes_cells = get_cell_position_from_coords(nodes, map_info).reshape(-1, 2)
        for i, cell in enumerate(nodes_cells):
            assert 0 <= cell[1] < free_connected_map.shape[0] and 0 <= cell[0] < free_connected_map.shape[1]
            if free_connected_map[cell[1], cell[0]] == 1:
                indices.append(i)
        indices = np.array(indices,dtype=int)
        nodes = nodes[indices].reshape(-1, 2)
    #print(f'[get_updating_node_coords] nodes: {nodes}')
    #print(f'[get_updating_node_coords] free_connected_map: {free_connected_map}')
    return nodes, free_connected_map
##该函数用于检测地图中的前沿（frontiers）。
##前沿通常指的是已知区域与未知区域之间的边界，是机器人探索和扩展地图的重要依据。具体来说，该函数识别出地图中所有自由单元格（FREE）的邻近未知单元格（UNKNOWN），从而确定前沿点。
def get_frontier_in_map(map_info):
    x_len = map_info.map.shape[1]
    y_len = map_info.map.shape[0]  
    unknown = (map_info.map == UNKNOWN) * 1
    unknown = np.lib.pad(unknown, ((1, 1), (1, 1)), 'constant', constant_values=0)
    unknown_neighbor = unknown[2:][:, 1:x_len + 1] + unknown[:y_len][:, 1:x_len + 1] + unknown[1:y_len + 1][:, 2:] \
                       + unknown[1:y_len + 1][:, :x_len] + unknown[:y_len][:, 2:] + unknown[2:][:, :x_len] + \
                       unknown[2:][:, 2:] + unknown[:y_len][:, :x_len]
    free_cell_indices = np.where(map_info.map.ravel(order='F') == FREE)[0]
    frontier_cell_1 = np.where(1 < unknown_neighbor.ravel(order='F'))[0]
    frontier_cell_2 = np.where(unknown_neighbor.ravel(order='F') < 8)[0]
    frontier_cell_indices = np.intersect1d(frontier_cell_1, frontier_cell_2)
    frontier_cell_indices = np.intersect1d(free_cell_indices, frontier_cell_indices)

    x = np.linspace(0, x_len - 1, x_len)
    y = np.linspace(0, y_len - 1, y_len)
    t1, t2 = np.meshgrid(x, y)
    cells = np.vstack([t1.T.ravel(), t2.T.ravel()]).T
    frontier_cell = cells[frontier_cell_indices]

    frontier_coords = get_coords_from_cell_position(frontier_cell, map_info).reshape(-1, 2)
    if frontier_cell.shape[0] > 0 and FRONTIER_CELL_SIZE != CELL_SIZE:
        frontier_coords = frontier_coords.reshape(-1 ,2)
        frontier_coords = frontier_down_sample(frontier_coords)
    else:
        frontier_coords = set(map(tuple, frontier_coords))
    return frontier_coords

##该函数用于对前沿点进行下采样，减少前沿点的数量，避免前沿点过于密集，从而提升后续处理的效率和性能。下采样通过将前沿点划分到体素（voxel）中，并选择距离体素中心最近的点来实现。
def frontier_down_sample(data, voxel_size=FRONTIER_CELL_SIZE):
    voxel_indices = np.array(data / voxel_size, dtype=int).reshape(-1, 2)

    voxel_dict = {}
    for i, point in enumerate(data):
        voxel_index = tuple(voxel_indices[i])

        if voxel_index not in voxel_dict:
            voxel_dict[voxel_index] = point
        else:
            current_point = voxel_dict[voxel_index]
            if np.linalg.norm(point - np.array(voxel_index) * voxel_size) < np.linalg.norm(
                    current_point - np.array(voxel_index) * voxel_size):
                voxel_dict[voxel_index] = point

    downsampled_data = set(map(tuple, voxel_dict.values()))
    return downsampled_data

##用于检测是否在局部地图的范围中
def in_local_map_range(coords, map_info):
    x,y = coords
    min_x = map_info.map_origin_x
    min_y = map_info.map_origin_y
    max_x = map_info.map_origin_x + map_info.cell_size * map_info.map.shape[1]
    max_y = map_info.map_origin_y + map_info.cell_size * map_info.map.shape[0]
    return min_x <= x <= max_x and min_y <= y <= max_y


##检测是否在全局地图的范围中
def in_global_map_range(coords, map_info):
    x,y = coords
    min_x = map_info.map_origin_x - map_info.cell_size * map_info.map.shape[1] // 2
    min_y = map_info.map_origin_y - map_info.cell_size * map_info.map.shape[0] // 2
    max_x = map_info.map_origin_x + map_info.cell_size * map_info.map.shape[1] // 2
    max_y = map_info.map_origin_y + map_info.cell_size * map_info.map.shape[0] // 2
    return min_x <= x <= max_x and min_y <= y <= max_y

def check_collision(start, end, map_info):
    #print(f'[check_collision] start:{start[0],start[1]}, end: {end[0],end[1]}')
    # Bresenham line algorithm checking
    width = map_info.map.shape[1] * map_info.cell_size
    height = map_info.map.shape[0] * map_info.cell_size
    assert start[0] >= map_info.map_origin_x - width/2
    assert start[1] >= map_info.map_origin_y - height/2
    assert end[0] >= map_info.map_origin_x - width/2
    assert end[1] >= map_info.map_origin_y - height/2
    assert start[0] <= map_info.map_origin_x  + width/2, f'start:{start[0],start[1]}, map_info.map_origin_x:{map_info.map_origin_x}, width:{width}'
    assert start[1] <= map_info.map_origin_y  + height/2
    assert end[0] <= map_info.map_origin_x + width/2
    assert end[1] <= map_info.map_origin_y + height/2
    collision = False

    start_cell = get_cell_position_from_coords(start, map_info)
    end_cell = get_cell_position_from_coords(end, map_info)
    map = map_info.map
    
    x0 = start_cell[0]
    y0 = start_cell[1]
    x1 = end_cell[0]
    y1 = end_cell[1]
    #print(f'[check_collision] start:{start[0],start[1]}, end: {end[0],end[1]}')
    #print(f'[check_collision] start_cell:{start_cell[0],start_cell[1]}, end_cell: {end_cell[0],end_cell[1]}')
    #print(f'[check_collision] map_info{map_info.map_origin_x, map_info.map_origin_y, map_info.map_origin_x + map_info.cell_size * map_info.map.shape[1], map_info.map_origin_y + map_info.cell_size * map_info.map.shape[0]}')
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    error = dx - dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    dx *= 2
    dy *= 2

    while 0 <= x < map.shape[1] and 0 <= y < map.shape[0]:
        k = map.item(int(y), int(x))
        if x == x1 and y == y1:
            break
        if k == OCCUPIED:
            collision = True
            break
        if k == UNKNOWN:
            collision = True
            break
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return collision

def check_local_collision(start, end, local_map_info, map_info):
    
    assert start[0] >= local_map_info.map_origin_x,f'start[0]:{start[0], start[1]}, local_map_info.map_origin_x:{local_map_info.map_origin_x}'
    assert start[1] >= local_map_info.map_origin_y,f'start[1]:{start[0], start[1]}, local_map_info.map_origin_y:{local_map_info.map_origin_y}'
    assert end[0] >= local_map_info.map_origin_x,f'end[0]:{end[0], end[1]}, local_map_info.map_origin_x:{local_map_info.map_origin_x}'
    assert end[1] >= local_map_info.map_origin_y,f'end[1]:{end[0], end[1]}, local_map_info.map_origin_y:{local_map_info.map_origin_y}'
    assert start[0] <= local_map_info.map_origin_x + local_map_info.cell_size * local_map_info.map.shape[1], f'start[0]:{start[0], start[1]}, local_map_info.map_origin_x:{local_map_info.map_origin_x}, local_map_info.cell_size:{local_map_info.cell_size}, local_map_info.map.shape[1]:{local_map_info.map.shape[1]}' 
    assert start[1] <= local_map_info.map_origin_y + local_map_info.cell_size * local_map_info.map.shape[0], f'start[1]:{start[0], start[1]}, local_map_info.map_origin_y:{local_map_info.map_origin_y}, local_map_info.cell_size:{local_map_info.cell_size}, local_map_info.map.shape[0]:{local_map_info.map.shape[0]}'
    assert end[0] <= local_map_info.map_origin_x + local_map_info.cell_size * local_map_info.map.shape[1], f'end[0]:{end[0], end[1]}, local_map_info.map_origin_x:{local_map_info.map_origin_x}, local_map_info.cell_size:{local_map_info.cell_size}, local_map_info.map.shape[1]:{local_map_info.map.shape[1]}'
    assert end[1] <= local_map_info.map_origin_y + local_map_info.cell_size * local_map_info.map.shape[0], f'end[1]:{end[0], end[1]}, local_map_info.map_origin_y:{local_map_info.map_origin_y}, local_map_info.cell_size:{local_map_info.cell_size}, local_map_info.map.shape[0]:{local_map_info.map.shape[0]}'
    collision = False

    global_start_cell = get_cell_position_from_coords(start, map_info)
    global_end_cell = get_cell_position_from_coords(end, map_info)
    local_origin_cell = get_cell_position_from_coords([local_map_info.map_origin_x, local_map_info.map_origin_y], map_info)
    local_grid = local_map_info.map
    print(f'[check_local_collision] start:{start[0],start[1]}, end: {end[0],end[1]}')
    print(f'[check_local_collision] global_start_cell:{global_start_cell[0],global_start_cell[1]}, global_end_cell: {global_end_cell[0],global_end_cell[1]}')
    print(f'[check_local_collision] local_origin_cell:{local_origin_cell[0],local_origin_cell[1]}')
    
    start_cell= (global_start_cell[0] - local_origin_cell[0], global_start_cell[1] - local_origin_cell[1])
    end_cell = (global_end_cell[0] - local_origin_cell[0], global_end_cell[1] - local_origin_cell[1])
    
    ##起始点和终止点的全局世界坐标
    
    x0 = start_cell[0]
    y0 = start_cell[1]
    x1 = end_cell[0]
    y1 = end_cell[1]
    
  
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    error = dx - dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    dx *= 2
    dy *= 2

    while 0 <= x < local_grid.shape[1] and 0 <= y < local_grid.shape[0]:
        k = local_grid.item(int(y), int(x))
        if x == x1 and y == y1:
            break
        if k == OCCUPIED:
            collision = True
            break
        if k == UNKNOWN:
            collision = True
            break
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return collision

def make_gif(path, n, frame_files, rate):
    with imageio.get_writer('{}/{}_explored_rate_{:.4g}.gif'.format(path, n, rate), mode='I', duration=0.5) as writer:
        for frame in frame_files:
            image = imageio.imread(frame)
            writer.append_data(image)
    print('gif complete\n')

    # Remove files
    for filename in frame_files[:-1]:
        os.remove(filename)

