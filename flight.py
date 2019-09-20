#!/usr/bin/env python
# _*_ coding:utf-8 _*_

from copy import deepcopy
import math

SPEED_OF_FLIGHT = 10

class Flight(object):

    def __init__(self,point,distance):
        # POINT {}
        self._point = point
        # DIST [[]]
        self._distance = distance
        # current position
        self._position = [2490.0, 1529.0]
        # mission A
        self._mission_a = []
        # mission B
        self._mission_b = []
        # new route
        self._route = []
        # route finished
        self._route_done = []
        # current todolist
        self._todo_list = []

    """
    time_interval + position + route -> position' + route_done
    """
    def get_position(self,time_interval):
        self._route_done.clear()
        # has nothing to do and stands in place
        if len(self._route)==0:
            return deepcopy(self._position)
        time_spent = 0
        # length from current position to route[0]
        len_start = math.sqrt(pow(self._point[self._route[0]][0]-self._position[0],2)+pow(self._point[self._route[0]][1]-self._position[1],2))
        time_spent += len_start / SPEED_OF_FLIGHT
        # has not come to start
        if time_spent >= time_interval:
            self._position[0] = self._position[0] + (self._point[self._route[0]][0] - self._position[0]) * time_interval * SPEED_OF_FLIGHT / len_start
            self._position[1] = self._position[1] + (self._point[self._route[0]][1] - self._position[1]) * time_interval * SPEED_OF_FLIGHT / len_start
            return deepcopy(self._position)
        # has come to start
        self._route_done.append(self._route[0])
        index = 0
        len_phase = 0
        for i in range(len(self._route)-1):
            len_phase = self._distance[self._route[i]][self._route[i+1]]
            time_spent += len_phase / SPEED_OF_FLIGHT
            # has not finished the phase from index to index+1
            if time_spent >= time_interval:
                break
            index += 1
            # has come to index+1
            self._route_done.append(self._route[index])
        # has finished all the route
        if index==len(self._route)-1:
            self._position = deepcopy(self._point[self._route[-1]])
            return deepcopy(self._position)
        # stops between index and index+1
        self._position[0] = self._point[self._route[index + 1]][0] - (self._point[self._route[index + 1]][0] - self._point[self._route[index]][0]) * (time_spent - time_interval) * SPEED_OF_FLIGHT / len_phase
        self._position[1] = self._point[self._route[index + 1]][1] - (self._point[self._route[index + 1]][1] - self._point[self._route[index]][1]) * (time_spent - time_interval) * SPEED_OF_FLIGHT / len_phase
        return deepcopy(self._position)

    """
    route_done + todo_list + A + B -> todo_list' + A' + B'
    """
    def update_mission_todolist(self):
        for i in range(len(self._route_done)):
            # route should match todo_list
            assert(self._route_done[i]==self._todo_list[0]["point"])
            temp = self._todo_list[0]
            # the head is finished
            self._todo_list.remove(temp)
            # find missions finished
            if "put" in temp["todo"].keys():
                # to store the missions have been finished
                finish = []
                for j in range(len(self._mission_a)):
                    if self._mission_a[j][0] in temp["todo"]["put"]:
                        finish.append(self._mission_a[j])
                # remove the missions from A
                for j in range(len(finish)):
                    self._mission_a.remove(finish[j])
            # find missions started
            if "get" in temp["todo"].keys():
                # to store the missions have been started
                start = []
                for j in range(len(self._mission_b)):
                    if self._mission_b[j][0] in temp["todo"]["get"]:
                        start.append(self._mission_b[j])
                # remove the missions from B and add them to A
                for j in range(len(start)):
                    self._mission_b.remove(start[j])
                    self._mission_a.append(start[j])
        return deepcopy(self._mission_a),deepcopy(self._mission_b),deepcopy(self._todo_list)

    """
    B + todo_list -> B + todo_list + route
    """
    def update_from_center(self,mission_b,todo_list):
        # copy from the center
        self._route.clear()
        self._mission_b = mission_b
        self._todo_list = todo_list
        # calculate the route
        for i in range(len(self._todo_list)):
            self._route.append(self._todo_list[i]["point"])

    """
    get route_done to display route
    """
    def get_route_done(self):
        return deepcopy(self._route_done)

    """
    get route to display route
    """
    def get_route(self):
        return deepcopy(self._route)