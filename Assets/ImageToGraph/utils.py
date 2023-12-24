from PIL import Image
import numpy as np
import networkx as nx
from itertools import permutations
import json 
import random
import os
from gtts import gTTS
from playsound import playsound
import time

imagetograph_path = os.path.dirname(os.path.abspath(__file__))


def say(text: str, to_file:bool=True):
    """Say or save to audio file a give text

    Args:
        text (str): Text to be spoken
        to_file (bool, optional): Decide if to save the file or speak it. Defaults to True.
    """
    tts = gTTS(text) 
    filename = f'{time.time()}.mp3'
    tts.save(filename)
    if not to_file:
        playsound(filename)
        os.remove(filename)
        

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

def map_evenly(coords, items):
    """For each section receives the items that go into that section and the coordinates
    of that section, afterwards evenly and randomly spreads the items across the various 
    coordinates.

    Args:
        coords (list): List of all the available coords in the supermarket.
        items (list): List of all the items to be place in there.

    Returns:
        dict: Returns a mapping between each coordinate and item to be placed there.
    """
    # Shuffle the first array to ensure random distribution
    random.shuffle(items)

    # Determine the amount of items each node gets
    steps = np.linspace(0,len(items), len(coords)+1)
    steps = list(map(round, steps))

    # Create a dictionary to store the mappings
    mapping = {}

    # Map elements from the second array to the first array
    for i in range(len(coords)):
       #print(f'{steps[i]}:{steps[i+1]}')
       mapping.update({coords[i]:items[int(steps[i]):int(steps[i+1])]})

    return mapping


def get_nodes(image:np.array, map_colors: dict, other_colors: dict)->(nx.Graph,list):
    """Given an image and the colors for the various parts of the floorplan, return the graph
    containing only the nodes at first, with already the items placed inside.

    Args:
        image (np.array): Input Image.
        map_colors (dict): Colors that define the floorplan (both section and walkable).
        other_colors(dict): Colors that are not used in the graph, used for assertion.

    Returns:
        (nx.Graph, list): The graph containing all the nodes with the items inside.
        List of coordinates containing colors that are not supposed to be used.
    """
    graph = nx.Graph()
    wrong_colors = []
    section_coordinates = {}
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            color = image[row,col]
            if any(all(color==curr_color) for curr_color in map_colors.values()): #Assign section and item to node
                section = find_section_from_color(map_colors, color)
                try:
                    section_coordinates[section].append((row,col))
                except KeyError:
                    section_coordinates.update({section:[(row,col)]})
                graph.add_node((row,col), section = section)
            elif not any(all(color==curr_color) for curr_color in other_colors.values()):
                wrong_colors.append((row,col))
    
    #Now evenly add the items to the correct section and then to the nodes
    with open(os.path.join(imagetograph_path, 'supermarket_items.json')) as f:
        items = json.load(f)
    mapping = {}
    for key in section_coordinates.keys():
        try:
            item_names = items[key]
        except KeyError:
            continue
        coords = section_coordinates[key]
        mapping.update(map_evenly(coords,item_names))
    nx.set_node_attributes(graph, mapping, 'item')
    return graph,wrong_colors

def draw_path(image: np.array, paths: list, filepath = '', star_pos_color = False, only_return = False):
    """Draws a given path on the map.

    Args:
        image (np.array): Starting image.
        paths (list): A list of paths where each path is a list of nodes.
        filepath (str, optional): Filepath where to save the resulting image, if '' then it will only show the image without saving. Defaults to ''.
        star_pos_color (bool, optional): If true, draws the first pixel and its neighbourhood in red to know where you are located
    """
    im = image.copy()
    colors = [[c,c,c] for c in range(0,240,240//len(paths))] #Define the colors for the different sections of the path
    for i,path in enumerate(paths):
        for node_coords in path[1:-1]: #Doesn't color the start and the end
            im[node_coords[0], node_coords[1], :] = colors[i]
        if i == 0 and star_pos_color:
            start_neighbourhood = get_neighbours(path[0][0], path[0][1], True)
            for node in start_neighbourhood:
                if np.all(im[node[0], node[1], :] == 255) or np.all(im[node[0], node[1], :] == 0): #Color only if walkable or paht
                    im[node[0], node[1], :] = [255,0,0] #Colors in red the neighbourhood of the starting pixel 
            im[path[0][0], path[0][1], :] = [255,0,0]
    
    im = im[40:-40, 40:-40]
    im = Image.fromarray(np.uint8(im))
    im = im.resize((300,300), resample=Image.BOX)
    
    if only_return:
        return im
    else:
        if len(filepath) == 0:
            im.show(title="path")
        else:
        #if len(filepath) != 0:
            im.save(fp=filepath)
        return im


def get_coordinates(shopping_list: list, graph: nx.Graph, with_start=True)->list:
    """Given a list of items, returns the corresponding coordinates where they are located.

    Args:
        shopping_list (list): List of items to buy.
        graph (nx.Graph): Supermarket Graph.
        with_start (bool, optional): Defines if we want to include the entrance and exit of the.
        supermarket at the start/end of the path. Defaults to True.

    Returns:
        list: List of coordinates in the same order as the shopping list
    """
    coordinates = [(0,0)]*len(shopping_list)
    start_coord = (0,0)
    end_coord = (0,0)
    for coord,attributes in graph.nodes(data = True): 
        if attributes['section'] == 'Start':
            start_coord = coord
            continue
        if attributes['section'] == 'End':
            end_coord = coord
            continue
        try:
            shelf = attributes['item']
        except KeyError:
            continue
        for item in shelf:
            if item in shopping_list:
                idx = shopping_list.index(item)
                coordinates[idx] = coord
    if with_start:
        coordinates.insert(0,start_coord)
        coordinates.append(end_coord)

    return coordinates

def permute_third_list(first_list, second_list, third_list):
    """
    Permutes the third list in the same way as the second list, based on the permutation of the first list.

    Args:
    first_list (list): The original list.
    second_list (list): The list that is a permutation of the original list.
    third_list (list): The list to be permuted based on the permutation of the second list.

    Returns:
    list: The permuted third list.
    """
    # Check if the lengths of all lists are equal
    assert len(first_list) == len(second_list) or len(second_list) == len(third_list), "All lists must be of equal length"

    # Create a dictionary to store the indices of elements in the second list
    indices = {element: i for i, element in enumerate(second_list)}

    # Use the indices from the second list's elements to reorder the third list
    permuted_third_list = [third_list[indices[element]] for element in first_list]

    return permuted_third_list

def hamiltonian_path(graph: nx.Graph, items:np.array, origin: tuple = None) -> (np.array,np.array,float):    
    """Given a list of items finds the shortest path along the supermarket that visits all these items, for more than 11 nodes
    it will compute an approximation. That the path will always start from the first element of the coordinates and end in the last

    Args:
        graph (nx.Graph): Networkx graph used to run the subroutines
        int (np.array): List of items to visit

    Returns:
        (np.array,np.array,int): Returns two numpy arrays containing items and nodes in the correct order and path length
    """
    coordinates = get_coordinates(items, graph)
    if origin is not None:
        coordinates[0] = origin

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
        items = permute_third_list(coordinates[1:-1],best_perm[1:-1],items)
        return items,best_perm,best_perm_len
    
    else: #Greedy algorithm that takes shortest outgoing edge
        graph = nx.Graph() #This will be a path graph built from the adj matrix
        graph.add_nodes_from(coordinates)

        visited = [0]*len(coordinates) #Keeps track of which nodes where visited in the algorith(all nodes except start and finish at most 2 times)
        flat_indices = np.argsort(adj_mat, axis=None)[:-len(coordinates)-2:2] #Drop diagonal, upper triangular and corners
        row_indices, col_indices = np.unravel_index(flat_indices, adj_mat.shape)
        i = 0
        
        while len(graph.edges) < len(coordinates)-1:#Check smallest edge weight at each iteration, need to avoid that edge connect start and end index
            row_index = row_indices[i]
            col_index = col_indices[i] 
            if visited[row_index] == 2 or visited[col_index] == 2 or (visited[0] == 1 and min(row_index, col_index) == 0) or (visited[-1] == 1 and max(row_index, col_index) == len(coordinates)-1):
                i += 1
                continue
            else:    
                graph.add_edge(coordinates[min(row_index,col_index)],coordinates[max(row_index,col_index)], weight = adj_mat[row_index,col_index])
                
                #Important: need to avoid to add cycles or edges that create a path between start and finish
                if len(graph.edges)<len(coordinates)-1 and nx.has_path(graph, coordinates[0], coordinates[-1]): 
                    graph.remove_edge(coordinates[min(row_index,col_index)],coordinates[max(row_index,col_index)])
                    i += 1
                    continue

                try: 
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
        items = permute_third_list(coordinates[1:-1],path[1:-1],items)

        return items,path, path_len

def create_graph(image: np.array, scale = 1) -> nx.Graph:
    """Given an image of an indoor space returns a networkx graph representing the image

    Args:
        imagedir (np.array): Input imge as array
        scale (int, optional): How many meters does a pixel represent, distances will be measured in meters. Defaults to 1.

    Returns:
        nx.Graph: Graph representing the supermarket
    """
    """
        Static colors in the image, should follow pattern:
        section_colors (dict): Dicitonary of the colors of the various sections of the supermarkets.
        walkable_colors (dict): Colors of the areas where people are able to walk, ideally add different color for the entrance and exit
        other_colors (dict): Colors like walls and the area outside of the supermarket, these colors won't be checked
    """
    section_colors = {'Meat': [255,126,121],'Bakery': [252,168,78],'Dairy' : [250,225,80],
                    'Frozen' : [78,235,239],'Seafood' : [80,156,218], 'Produce' : [148,214,105],
                    'Tools': [172,165,142],'Drinks' : [132,92,85],'Grocery' : [124,124,124]}

    walkable_colors = {'Path': [255,255,255],'Start': [237,0,255],'End': [255,0,137]}

    other_colors = {'Wall': [70,70,70],'Background' : [155,173,183]}


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

def directions_to_next_item(graph: nx.Graph, image: np.array, current_location: tuple, destination: tuple):
    """Draws the directions to the given point

    Args:
        graph (nx.Graph): Input graph
        image (np.array): Image related to the graph
        current_location (tuple): Coordinates of the shopper location
        destination (tuple): Destination of the shopper
    """
    path = [nx.dijkstra_path(graph, current_location, destination)]
    draw_path(image, path,star_pos_color=True)
