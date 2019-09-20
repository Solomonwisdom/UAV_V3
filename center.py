#!/usr/bin/env python
# _*_ coding:utf-8 _*_
import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
from copy import deepcopy
from flight import Flight
from core import SaPhase1, SaPhase2
import math
# import matplotlib.pyplot as plt

NUM_OF_FLIGHT = 3
SPEED_OF_FLIGHT = 10
NUM_OF_POINT = 30

MAX_WEIGHT = 1000000

# id(start from 0) : [x(float),y(float)]
POINT = {}
# [[dij]] dis from pi to pj
DIST = []
# id(start from 0) : [id(int),start(int),end(int),dt(float)]
MISSION = {}
# id(start from 0) : flight object
FLIGHT = {}

# [...[mission,mission...]...] to finish
MISSION_A = []
# [...[mission,mission...]...] to start
MISSION_B = []
# [...[td,td...]...] td = {id:point_id,td:{get:[i,i...],put:[i,i...]}} i = mission_id
TODO_LIST = []
# [...[p,p...]...] p = [x,y]
PATH = []

# figure size
WI = 5000
BR = 3000

def initialize_point():
    global POINT
    f = open(os.path.join(BASE_DIR, "data/point.txt"))
    lines = f.read().splitlines()
    for line in lines:
        temp = line.split(" ")
        POINT[int(temp[0])] = [float(temp[1]), float(temp[2])]
    f.close()

def initialize_dist():
    global DIST
    DIST = [[float(0) for i in range(NUM_OF_POINT)] for j in range(NUM_OF_POINT)]
    f = open(os.path.join(BASE_DIR, "data/route.txt"))
    lines = f.read().splitlines()
    for line in lines:
        temp = line.split(" ")
        DIST[int(temp[0])][int(temp[1])] = float(temp[2])
        DIST[int(temp[1])][int(temp[0])] = float(temp[2])
    f.close()

def initialize_mission(content):
    global MISSION
    MISSION.clear()
    lines = content.splitlines()
    id = 0
    for line in lines:
        temp = line.split(" ")
        MISSION[id] = [id,int(temp[0]),int(temp[1]),float(temp[2])]
        id += 1

def initialize_flight():
    global FLIGHT
    for i in range(NUM_OF_FLIGHT):
        FLIGHT[i] = Flight(deepcopy(POINT),deepcopy(DIST))

# read from data file
def load_file():
    initialize_point()
    initialize_dist()
    initialize_flight()

# prepare for the center
def init_center():
    global MISSION_A
    global MISSION_B
    global TODOLIST
    global PATH
    MISSION_A = []
    MISSION_B = []
    TODO_LIST = []
    PATH = []
    for i in range(NUM_OF_FLIGHT):
        MISSION_A.append([])
        MISSION_B.append([])
        TODO_LIST.append([])
        PATH.append([])

# dist -> cost
def generate_distance(position):
    cost = {}
    # current point & N points --> N+1 * N+1
    len_of_content = NUM_OF_POINT + 1
    for i in range(NUM_OF_FLIGHT):
        content = [[float(0) for j in range(len_of_content)] for k in range(len_of_content)]
        # 0 vs 1-N
        for j in range(1, len_of_content):
            c = math.sqrt(pow(position[i][0] - POINT[j-1][0], 2) + pow(position[i][1] - POINT[j-1][1], 2)) / SPEED_OF_FLIGHT
            content[0][j] = c
            content[j][0] = c
        # 1-N * 1-N
        for j in range(1, len_of_content):
            for k in range(j+1, len_of_content):
                c = DIST[j-1][k-1] / SPEED_OF_FLIGHT
                content[j][k] = c
                content[k][j] = c
        cost[i] = content
    return deepcopy(cost)

# current_cost
def generate_cost_current(content, flight_id):
    # current todo_list = []
    if len(TODO_LIST[flight_id]) == 0:
        return 0.0
    else:
        # route time
        time_all = 0
        # mission time
        cost_all = 0
        for i in range(len(TODO_LIST[flight_id])):
            point_id = TODO_LIST[flight_id][i]["point"]
            if i == 0:
                # current position to point 0
                time_all += content[0][point_id + 1]
            else:
                # point i-1 to point i
                time_all += content[TODO_LIST[flight_id][i - 1]["point"] + 1][point_id + 1]
            # cost += flight_time * num_of_mission(finished now)
            if "put" in TODO_LIST[flight_id][i]["todo"].keys():
                cost_all += time_all * len(TODO_LIST[flight_id][i]["todo"]["put"])
        return cost_all

# generate point set
def generate_route(a, b):
    route = []
    for i in range(len(a)):
        # add the end point
        route.append(a[i][2])
    for i in range(len(b)):
        # add the start point
        route.append(b[i][1])
    return deepcopy(list(set(route)))


# handle each mission
def handle_each_mission(ID):
    position = []
    # prepare for the algorithm
    for j in range(NUM_OF_FLIGHT):
        position.append(FLIGHT[j].get_position(MISSION[ID][3]))
        temp_route_done = FLIGHT[j].get_route_done()
        MISSION_A[j], MISSION_B[j], TODO_LIST[j] = FLIGHT[j].update_mission_todolist()
        # append the route_done to the path
        for k in range(len(temp_route_done)):
            PATH[j].append([POINT[temp_route_done[k]][0], POINT[temp_route_done[k]][1]])
    # append the current position to the path
    for j in range(NUM_OF_FLIGHT):
        PATH[j].append(deepcopy(position[j]))
    # generate cost matrix
    cost = generate_distance(deepcopy(position))
    # store the temp result
    current_cost = {}
    new_cost = {}
    delta_cost = {}
    flight_time = {}
    mission_b_temp = {}
    todo_list_temp = {}
    # start the algorithm
    for j in range(NUM_OF_FLIGHT):
        # get the current cost
        current_cost[j] = generate_cost_current(deepcopy(cost[j]), j)
        b_temp = deepcopy(MISSION_B[j])
        b_temp.append(MISSION[ID])
        a_temp = deepcopy(MISSION_A[j])
        # generate points without duplicates based on mission a+b
        route1 = generate_route(deepcopy(a_temp), deepcopy(b_temp))
        sa_phase1 = SaPhase1(deepcopy(a_temp), deepcopy(b_temp), deepcopy(cost[j]))
        """
        A + B' = route1 + todolist1 + A' + cost1 + time1
        """
        min_route1, cost_all1, time_all1, todo_list1, a_new = sa_phase1.min_cost(deepcopy(route1))
        # generate route_phase_2 based on a_new
        route2 = generate_route(deepcopy(a_new), deepcopy([]))
        sa_phase2 = SaPhase2(deepcopy(a_new), deepcopy(cost[j]), min_route1[-1])
        """
        A' = route2 + todolist2 + cost2 + time2
        """
        min_route2, cost_all2, time_all2, todo_list2 = sa_phase2.min_cost(deepcopy(route2))
        # cost time over all
        cost_time_all = cost_all1 + cost_all2
        # flight time over all
        flight_time_all = time_all1 + time_all2
        # overall todolist
        todo_list_all = todo_list1 + todo_list2
        new_cost[j] = cost_time_all
        # delta time cost
        delta_cost[j] = new_cost[j] - current_cost[j]
        flight_time[j] = flight_time_all
        mission_b_temp[j] = deepcopy(b_temp)
        todo_list_temp[j] = deepcopy(todo_list_all)
    index = -1
    min_delta = MAX_WEIGHT
    # get the min delta
    for j in range(NUM_OF_FLIGHT):
        if delta_cost[j] < min_delta:
            index = j
            min_delta = delta_cost[j]
    print(index)
    # update the B and todolist in the center
    MISSION_B[index] = deepcopy(mission_b_temp[index])
    TODO_LIST[index] = deepcopy(todo_list_temp[index])
    # call the flight to update
    for j in range(NUM_OF_FLIGHT):
        FLIGHT[j].update_from_center(deepcopy(MISSION_B[j]), deepcopy(TODO_LIST[j]))
    print(MISSION_A)
    print(MISSION_B)
    print(TODO_LIST)
    print("\n\n")

# from pykafka import KafkaClient


# handle missions
def handle_mission(event, context):
    global POINT
    global DIST
    if len(POINT) == 0 or len(DIST) == 0:
        load_file()
    init_center()
    # client = KafkaClient("kafka.kubeless:9092")
    # topic = client.topics["uav_output"]
    # producer = topic.get_producer()

    data = event['data']
    if type(data)==bytes: 
        data = str(data, encoding='utf-8')
    initialize_mission(data)
    # handle every mission
    for i in range(len(MISSION)):
        # handle each mission
        handle_each_mission(i)
    # # append the route to the path
    # for i in range(NUM_OF_FLIGHT):
    #     temp_route = FLIGHT[i].get_route()
    #     for j in range(len(temp_route)):
    #         PATH[i].append([POINT[temp_route[j]][0],POINT[temp_route[j]][1]])
    result = {}
    for i in range(NUM_OF_FLIGHT):
        result[i] = FLIGHT[i].get_route()
    result_json = json.dumps(result)
    # producer.produce(bytes(result_json,encoding='utf-8'))
    # producer.stop()
    return bytes(result_json,encoding='utf-8')
    

# display paths
def display_path():
    print(PATH)
    # prepare to draw a figure
    color = ['green', 'red', 'blue', 'black', 'purple']
    for i in range(len(PATH)):
        # prepare to draw a sub_figure
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set_title('FLIGHT_ROUTE')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        for j in range(len(PATH[i]) - 1):
            # x and y axises
            x = []
            y = []
            x.append(PATH[i][j][0])
            y.append(PATH[i][j][1])
            x.append(PATH[i][j+1][0])
            y.append(PATH[i][j+1][1])
            ax.plot(x, y, color = color[i % 5])
        ax.axis([0, WI, 0, BR])
        plt.savefig("data/result" + str(i) + ".jpg")
