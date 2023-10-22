from PIL import Image
import numpy as np
import networkx as nx
from itertools import permutations

def get_neighbours(x: int, y: int, adjacency_8: bool) -> np.array:
    """Return the 4/8 neighbours of a coordinate.

    Args:
        x (int): Row index
        y (int): Column index
        adjacency_8 (bool): Decide if we want full 8 or 4 adjacency model

    Returns:
        np.array: Returns list of neighbours
    """
    adjacent_coords = []
    
    # Define possible offsets for neighbors based on adjacency setting
    if adjacency_8:
        offsets = [(-1, -1), (-1, 0), (-1, 1),
                   (0, -1),           (0, 1),
                   (1, -1),  (1, 0),  (1, 1)]
    else:
        offsets = [(0, -1), (-1, 0), (0, 1), (1, 0)]

    # Calculate adjacent coordinates
    for dx, dy in offsets:
        adjacent_coords.append((x + dx, y + dy))

    return adjacent_coords

def find_section_from_color(section_colors, color_array):
    """
    Find the section key corresponding to a given color array.

    Args:
        section_colors: Dictionary mapping section names to RGB color arrays.
        color_array: RGB color array to find the corresponding section for.

    Returns:
        The section key (section name) or None if no match is found.
    """
    for section, rgb in section_colors.items():
        if all(rgb == color_array):
            return section
    return None  # Return None if no match is found

def get_nodes(image:np.array, map_colors: dict, other_colors: dict)->(nx.Graph,list):
    graph = nx.Graph()
    wrong_colors = []
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            color = image[row,col]
            if any(all(color==curr_color) for curr_color in map_colors.values()):
                section = find_section_from_color(map_colors, color)
                graph.add_node((row,col), section = section)
            elif not any(all(color==curr_color) for curr_color in other_colors.values()):
                wrong_colors.append((row,col))
    return graph,wrong_colors


def draw_path(image: np.array, paths: list, filepath = ''):
    """Draws a given path on the map

    Args:
        image (np.array): Starting image
        paths (list): A list of paths where each path is a list of nodes
        filepath (str, optional): Filepath where to save the resulting image, if '' then it will only show the image without saving. Defaults to ''.
    """
    im = image.copy()
    colors = [[c,c,c] for c in range(0,240,240//len(paths))] #Define the colors for the different sections of the path
    for i,path in enumerate(paths):
        for node_coords in path[1:-1]:
            im[node_coords[0], node_coords[1], :] = colors[i]
    im = Image.fromarray(im)
    im = im.resize((1000,1000), resample=Image.BOX)
    if filepath == '':
        im.show()
    else:
        im.save(fp=filepath)

def hamiltonian_path(graph: nx.Graph, coordinates:np.array) -> (np.array,float):    
    """Given a list of nodes finds the shortest path along the graph that visits all these nodes, for more than 11 nodes
    it will compute an approximation. That the path will always start from the first element of the coordinates and end in the last

    Args:
        graph (nx.Graph): Networkx graph used to run the subroutines
        int (np.array): List of coordinates to visit

    Returns:
        (np.array,int): Returns a numpy array containing the order in which the nodes are visited and the length of the path
    """
    adj_mat = np.empty((len(coordinates),len(coordinates)), dtype = np.float16)
    for i,node_coords in enumerate(coordinates):
        lengths = nx.single_source_dijkstra_path_length(graph,node_coords)
        lengths = [lengths[key] for key in coordinates] #Assign distances
        adj_mat[i,:] = lengths #Could also just fill upper diag and optimize search
        adj_mat[i,i] = 10000 #Avoid finding only self loops (arbitrary value here)
    adj_mat[0,len(coordinates)-1] = adj_mat[len(coordinates)-1,0] = 10000 #Avoid considering edge from start to end node

    if len(coordinates)<=11: #For 11 items takes 1.5 seconds to brute force
        perms = list(permutations(range(1,len(coordinates)-1))) 
        best_perm = []
        best_perm_len = adj_mat[0,0]*len(coordinates)

        for perm in perms:
            perm = (0,)+perm+(len(coordinates)-1,)
            curr_len = sum([adj_mat[perm[i],perm[i+1]] for i in range(len(perm)-1)])
            if curr_len<best_perm_len:
                best_perm = perm
                best_perm_len = curr_len
        best_perm = list(best_perm)
        best_perm = np.array(coordinates)[best_perm]
        best_perm = [tuple(best_perm[i]) for i in range(len(best_perm))]
        return best_perm,best_perm_len
    
    else: #Greedy algorithm that takes shortest outgoing edge
        graph = nx.Graph()
        graph.add_nodes_from(coordinates)

        visited = [0]*len(coordinates)
        flat_indices = np.argsort(adj_mat, axis=None)[:-len(coordinates)-2:2] #Drop diagonal, upper triangular and corners
        row_indices, col_indices = np.unravel_index(flat_indices, adj_mat.shape)
        i = 0
        
        while len(graph.edges) < len(coordinates)-1:#Chech smallest edge weight at each iteration, need to avoid that edge connect start and end index
            row_index = row_indices[i]
            col_index = col_indices[i] 
            if visited[row_index] == 2 or visited[col_index] == 2 or (visited[0] == 1 and min(row_index, col_index) == 0) or (visited[-1] == 1 and max(row_index, col_index) == len(coordinates)-1):
                i += 1
                continue
            else:    
                graph.add_edge(coordinates[min(row_index,col_index)],coordinates[max(row_index,col_index)], weight = adj_mat[row_index,col_index])
                
                if len(graph.edges)<len(coordinates)-1 and nx.has_path(graph, coordinates[0], coordinates[-1]): 
                    graph.remove_edge(coordinates[min(row_index,col_index)],coordinates[max(row_index,col_index)])
                    i += 1
                    continue

                try: #Important: need to avoid to add cycles
                    cycle = nx.find_cycle(graph, orientation="ignore")
                except nx.NetworkXNoCycle: 
                    visited[row_index] += 1
                    visited[col_index] += 1
                    i += 1
                    continue
                
                i += 1
                graph.remove_edge(coordinates[min(row_index,col_index)],coordinates[max(row_index,col_index)])
        
        path = nx.dijkstra_path(graph, coordinates[0],coordinates[-1])
        path_len = nx.dijkstra_path_length(graph, coordinates[0],coordinates[-1])

        return path, path_len

def create_graph(image: np.array,  section_colors: dict, walkable_colors: dict, other_colors:dict, scale = 1) -> nx.Graph:
    """Given an image of an indoor space returns a networkx graph representing the image

    Args:
        image (np.array): Input image
        section_colors (dict): Dicitonary of the various sections of the supermarkets.
        walkable_colors (dict): Colors of the areas where people are able to walk, ideally add different color for the entrance and exit
        other_colors (dict): Colors like walls and the area outside of the supermarket, these colors won't be checked
        scale (int, optional): How many meters does a pixel represent, distances will be measured in meters. Defaults to 1.

    Returns:
        nx.Graph: Graph representing the supermarket
    """
    graph,wrong_coords = get_nodes(image, {**section_colors,**walkable_colors}, other_colors)
    assert len(wrong_coords)==0, f'The image contains some colors not specified on the color list at coordinates:{wrong_coords}' 
    assert len(section_colors)>0, 'You need at least one color for the section_colors'
    
    for node_coords,node_data in graph.nodes(data=True):
            row = node_coords[0]
            col = node_coords[1]
            if node_data['section'] not in walkable_colors.keys(): #Case where color is inside section list
                neigbours = get_neighbours(row,col,False)
                for n_row, n_col in neigbours:
                    if graph.has_node((n_row,n_col)): #Avoid corner case neighbour is a wall
                        neighbour_section = graph.nodes[(n_row,n_col)]['section']
                        if neighbour_section in walkable_colors.keys(): #Check if neighbour is walkable
                            graph.add_edge(node_coords,(n_row,n_col), weight = (2*scale)**(0.5)*0.6) #Needs to be longer than half of cross edge to avoid shortcut
            else: #Case where color is start, end, path (Need only to consider internal edges between them)
                neigbours = get_neighbours(row,col,True)
                for n_row, n_col in neigbours:
                    if graph.has_node((n_row,n_col)): #Avoid corner case neighbour is a wall
                        neighbour_section = graph.nodes[(n_row,n_col)]['section']
                        if neighbour_section in walkable_colors.keys(): #Check if neighbour is walkabe
                            if abs(n_row-row)+abs(n_col-col) == 2: #Cross edge
                                graph.add_edge(node_coords,(n_row,n_col), weight = (2*scale)**(0.5))
                            else:
                                graph.add_edge(node_coords,(n_row,n_col), weight = scale)
    return graph