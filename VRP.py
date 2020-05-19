#import random
import time
import math

#######################################################
############  Vehicle Routing Problem  ################
#######################################################


############################################
#######    Create Distance Matrix  #########
###########################################


# def distance_formula(p1, p2):
#     #Eucledian distance.
#     dist = (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2
#     return dist

def distance_formula(p1, p2):
    #Eucledian distance.
    #dist = ((p1[0] - p2[0])*111)**2 + (110*cos(3.14*(p1[1] - p2[1])/180)**2
    lat = (p1[0] - p2[0])*110.57
    deg_2_rad = 3.14*(p1[1] - p2[1])/180
    log = 111.320*math.cos(deg_2_rad)
    dist = math.sqrt(lat**2 + log**2)
    return dist


#Name of a File containing Latitude and Longitude of locations.
#First point is the location of Head Quaters.
def get_distance_matrix(file = 'points.txt'):
    with open(file) as f:
        lat_long_file = f.readlines()
    lat_long = []
    for i in range(len(lat_long_file)):
        if len(lat_long_file[i].strip().split('\t')) == 2:
            temp_lat, temp_long = lat_long_file[i].strip().split('\t')
            lat_long.append([float(temp_lat), float(temp_long.split('\n')[0])])
        else:
            print("Else {}: ".format(i), lat_long_file[i].strip().split('\t'))

    distance_matrix = []
    for i in lat_long:
        temp_node_distance  = []
        for j in lat_long:
            temp_node_distance.append(distance_formula(i, j))
        distance_matrix.append(temp_node_distance)
    return distance_matrix

#Return True if node has benn visited before.
def is_node_visited(node, visited):
    for i in visited:
        if node == i:
            return True
    return False

def get_vehicles_distance(data, vehicles_route):
    vehicles_distance = {}
    for i in vehicles_route:
            vehicles_distance[i] = 0
            for j in range(1, len(vehicles_route[i])):
                vehicles_distance[i] += data[vehicles_route[i][j]][vehicles_route[i][j-1]]
    return vehicles_distance

def get_S_F(data):
    #It will save min and max values along with their index.    
    #Assign maximum possible distance value.
    min = 20000
    min_index = 0
    #Assign minimum possible distance.
    max = -1
    max_index = 0
    #Assuming "HQ" will always be positioned as first.
    for i, distance in enumerate(data[0]):
        if distance < min and distance != 0:
            min = distance
            min_index = i
        
        if distance > max:
            max = distance
            max_index = i
    out = {}
    out['min_value'] = min
    out['min_index'] = min_index
    out['max_value'] = max
    out['max_index'] = max_index
    return out

def calculate_G(data, min_index, max_index, num_vehicles):
    #Get a 'Y' as in paper.
    Y = data[min_index][max_index]
    #Calculate G
    G = Y / (num_vehicles - 1)
    return G

def initial_vehicle_index(data, min_index, max_index, num_vehicles, G): 
    #It saves index of Vehicles Start node.
    veh_start_node =  {}
    #Assign an index of node for First vehicle.
    veh_start_node['v0'] = [0, min_index]
    veh_start_node['v' + str(1)] = [0, max_index]

    #It stores nodes have been assigned to vehicles already.
    #Make sure two vehicles are not initialised from a same starting position.
    assign_nodes = [0]
    assign_nodes.append(min_index)
    assign_nodes.append(max_index)
    
    for i in range(2, num_vehicles):
        #Assign possible distance from a node.
        prev_error = 10000
        #Increase G linearly for every new vehicle.
        veh_G = (i-1) * G
        for j, k in enumerate(data[min_index]):
            if k == veh_G and (not is_node_visited(j, assign_nodes)):
                veh_start_node['v' + str(i)] = [0, j]
                assign_nodes.append(j)
                break
            elif abs(veh_G - k) < prev_error and (not is_node_visited(j, assign_nodes)):
                prev_error = abs(veh_G - k)
                veh_start_node['v' + str(i)] = [0, j]
        assign_nodes.append(veh_start_node['v'+str(i)][1])
        #print("assign_nodes: ", assign_nodes)
    return veh_start_node

#Return minimum distance node which is not visited yet.
def min_distance_node(data, visited):
    #maximum distance possible.
    min = 2000
    min_index = -1
    for i, value in enumerate(data):
        if value < min and (not is_node_visited(i, visited)):
            min = value
            min_index = i
    return min_index

def select_successor(data, vehicles_start, max_distance, vehicles_route = {}, visited_node = [0], vehicles_distance = {}):
    #Initializing visited nodes and vehicles routes.
    if not vehicles_route:
        visited_node = [0]
        for i in vehicles_start:
            visited_node.append(vehicles_start[i][1])
            vehicles_route[i] = vehicles_start[i]

    #print("Initial Successor: ", vehicles_route)
    if not vehicles_distance:
        vehicles_distance = get_vehicles_distance(data, vehicles_route)

    #print("Initial:", vehicles_distance)

    while True:
        impossible_node = True
        for i in vehicles_route:
            next_node = min_distance_node(data[vehicles_route[i][-1]], visited_node)
            if next_node == -1:
                return vehicles_route
            #Check node distance from "HQ" is less than Max limit.
            if vehicles_distance[i] + data[vehicles_route[i][-1]][next_node] + data[0][next_node] < max_distance:
                #Add distance vehicle has covered so far...
                vehicles_distance[i] += data[vehicles_route[i][-1]][next_node]
                #Add next node to be visited by this vehicle.
                vehicles_route[i].append(next_node)
                #Add this node in visited list.
                visited_node.append(next_node)
                impossible_node = False
        if impossible_node:
            print("Some nodes cannot be spanned because of Distance Limit.")
            print("You should increase distance limit.")
            #print("Nodes that are not been visited...")
            return vehicles_route
                
def select_successor_rearrange(data, vehicles_start, max_distance, vehicles_route = {}, visited_node = [0]):

    #Initializing visited nodes and vehicles routes.
    if not vehicles_route:
        for i in vehicles_start:
            visited_node.append(vehicles_start[i][1])
            vehicles_route[i] = vehicles_start[i]
        #print("select_successor_rearange: Vehicles route have been initialized...")

    #Calculating Vehicles distance.
    vehicles_distance = get_vehicles_distance(data, vehicles_route)
    #print("Distance from regroup func: ", vehicles_distance)

    while True:
        #It stores last visit locations of all vehicles.
        last_visit = []
        for i in vehicles_route:
            last_visit.append(vehicles_route[i][-1])
        next_node = []
        vehicle_distance = []
        vehicle_num = 0
        min_vehicle_distance = 2000000
        for i, j in enumerate(last_visit):
            next_node.append(min_distance_node(data[j], visited_node))
            vehicle_distance.append(data[j][next_node[-1]])
            if min_vehicle_distance > vehicle_distance[-1]:
                vehicle_num = i
                min_vehicle_distance = vehicle_distance[-1]
                
        #If all node have been visited exit from this function.
        if next_node[0] == -1:
            return vehicles_route
        for i, key in enumerate(vehicles_route):
            if vehicle_num == i:
                if vehicles_distance[key] + data[vehicles_route[key][-1]][next_node[i]] + data[0][next_node[i]] < max_distance:
                    #Add distance vehicle has covered so far...
                    vehicles_distance[key] += data[vehicles_route[key][-1]][next_node[i]]
                    #Add next node to be visited by this vehicle.
                    vehicles_route[key].append(next_node[i])
                    #Add this node in visited list.
                    visited_node.append(next_node[i])
                break

def calcultate_distance(data, vehicles_route):
    vehicle_distance = {}
    for i, key in enumerate(vehicles_route):
        vehicle_distance[key] = 0
        for j in range(1, len(vehicles_route[key])):
            vehicle_distance[key] += data[vehicles_route[key][j-1]][vehicles_route[key][j]]
    return vehicle_distance


###############################################
#########   Start of Execution  ###############
###############################################

start_time = time.ctime(time.time())
print("Program Starts at:", start_time)

data = {}
data['distance_matrix'] = [
    [0, 4, 2, 3, 8, 12, 11, 14, 17],
    [4, 0, 6, 3, 4, 6, 8, 9, 13],
    [2, 6, 0, 5, 7, 13, 8, 11, 12],
    [3, 3, 5, 0, 1, 8, 2, 7, 9],
    [8, 4, 7, 1, 0, 4, 1, 3, 5],
    [12, 6, 13, 8, 4, 0, 4, 3, 6],
    [11, 8, 8, 2, 1, 4, 0, 2, 4],
    [14, 9, 11, 7, 3, 3, 2, 0, 3],
    [17, 13, 12, 9, 5, 6, 4, 3, 0]
]

# To read data from a file uncomment below line.
# data['distance_matrix'] = get_distance_matrix('points.txt')

#Name of a file containing Latitude and Longitude of locations.
#First point me be the location of Head Quaters.
data['num_vehicles'] = 8
data['HQ'] = 0
data['Max_distance'] = 100000
data['release_regroup'] = 8

b = get_S_F(data['distance_matrix'])
G = calculate_G(data['distance_matrix'], b["min_index"], 
                b['max_index'], data['num_vehicles'])
print("G is: ", G)

vehicles_start = initial_vehicle_index(data['distance_matrix'], b["min_index"],
                b['max_index'], data['num_vehicles'], G)

#Initial_vehicle_route
initial_route = select_successor(data['distance_matrix'], vehicles_start, data['Max_distance'])

# Append HQ node in each vehicle node.
for i in initial_route:
    initial_route[i].append(0)

print("Initial Route: ")
print(initial_route)
distance = get_vehicles_distance(data['distance_matrix'], initial_route)
print("Initial Vehicles distance: ")
print(distance)

T_distance = 0
for i in distance:
    T_distance += distance[i]

print("Total Initial Distance: ", T_distance, "Km")
print("Total Initial Distance: ", T_distance*0.621371, "miles")

#Pop HQ as an ending position from initial_route.
for i in initial_route:
    initial_route[i].pop()


###########################
######  Regrouping  #######
###########################

#Initialized list for visited nodes.
visited_node = list(range(0, len(data['distance_matrix'][0])))

#Release last nodes from each vehicle route.
for key in initial_route:
    for j in range(data['release_regroup']):
        if len(initial_route[key]) > 2:
            try:
                b = initial_route[key].pop()
                visited_node.remove(b)
            except:
                pass
            #print(key, "Node is released: ")

regroup = select_successor_rearrange(data['distance_matrix'], initial_route, data['Max_distance'],
                     initial_route, visited_node = visited_node)

#Now add HQ as a destination in each Vehicle route.
for i in regroup:
    regroup[i].append(0)

print("Final routes for Vehicles: ")
print(regroup)

distance = get_vehicles_distance(data['distance_matrix'], regroup)
print("Final distance for Each Vehicle in Kilometers: ")
print(distance)

m_distance = distance
for i in m_distance:
    m_distance[i] = m_distance[i]*0.621371
print("Final distance for Each Vehicle in miles: ")
print(m_distance)

T_distance = 0
for i in distance:
    T_distance += distance[i]

print("Total distance after Regrouping: ", T_distance, "Km")
print("Total distance after Regrouping: ", T_distance*0.621371, "miles")

end_time = time.ctime(time.time())
print("Program Ends at: ", end_time)