import os
import sys
#path environment
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from plexe import Plexe, ACC, CACC

import random

#global variables
VEHICLE_LENGTH = 4
DISTANCE = 6  # inter-vehicle distance
LANE_NUM = 12
PLATOON_SIZE = 1
SPEED = 16.6  # m/s
V2I_RANGE = 200 
PLATOON_LENGTH = VEHICLE_LENGTH * PLATOON_SIZE + DISTANCE * (PLATOON_SIZE - 1)
MAX_ACCEL = 2.6
#DECEL = SPEED**2/(2*(V2I_RANGE-25))  
#DECEL = 3.5
STOP_LINE = 15.0

def communicate(plexe, topology):
    """ 
    Performs data transfer between vehicles, i.e., fetching data from
    leading and front vehicles to feed the CACC algorithm
    :param plexe: API instance
    :param topology: a dictionary pointing each vehicle id to its front
    vehicle and platoon leader. each entry of the dictionary is a dictionary
    which includes the keys "leader" and "front"
    """
    for vid, l in topology.items():
        if "leader" in l.keys():
            # get data about platoon leader
            ld = plexe.get_vehicle_data(l["leader"])
            # pass leader vehicle data to CACC
            plexe.set_leader_vehicle_data(vid, ld)
            # pass data to the fake CACC as well, in case it's needed
            plexe.set_leader_vehicle_fake_data(vid, ld)
        if "front" in l.keys():
            # get data about platoon leader
            fd = plexe.get_vehicle_data(l["front"])
            # pass front vehicle data to CACC
            plexe.set_front_vehicle_data(vid, fd)
            # compute GPS distance and pass it to the fake CACC
            distance = traci.get_distance(plexe, vid, l["front"])
            plexe.set_front_vehicle_fake_data(vid, fd, distance)


def add_single_platoon(plexe, topology, step, lane,ADD_PLATOON_STEP):
    for i in range(PLATOON_SIZE):
        vid = "v.%d.%d.%d" %(step/ADD_PLATOON_STEP, lane, i)
        routeID = "route_%d" %lane   # route 0~11, one-to-one map with lane
        traci.vehicle.add(vid, routeID, departPos=str(100-i*(VEHICLE_LENGTH+DISTANCE)), departSpeed=str(5), departLane=str(lane%3), typeID="vtypeauto")        
        plexe.set_path_cacc_parameters(vid, DISTANCE, 2, 1, 0.5)
        plexe.set_cc_desired_speed(vid, SPEED)
        plexe.set_acc_headway_time(vid, 1.5)
        plexe.use_controller_acceleration(vid, False)
        plexe.set_fixed_lane(vid, lane%3, False)
        traci.vehicle.setSpeedMode(vid, 0)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
            traci.vehicle.setColor(vid, (255,255,255,255))  # red
            topology[vid] = {}
        else:
            plexe.set_active_controller(vid, CACC)
            traci.vehicle.setColor(vid, (200,200,0, 255)) # yellow
            topology[vid] = {"front": "v.%d.%d.%d" %(step/ADD_PLATOON_STEP, lane, i-1), "leader": "v.%d.%d.0" %(step/ADD_PLATOON_STEP, lane)}



def add_platoons(plexe, topology, step,state, ADD_PLATOON_PRO = 0.3, ADD_PLATOON_STEP = 600):
    random.setstate(state)
    for lane in range(LANE_NUM):    # lane 0~11
        if random.random() < ADD_PLATOON_PRO:
            add_single_platoon(plexe, topology, step, lane,ADD_PLATOON_STEP)
    
    return random.getstate()


def compute_leaving_time(distance_veh_i,speed_veh_i):
    distance = 400 + PLATOON_LENGTH + STOP_LINE - distance_veh_i
    speed = speed_veh_i + 0.00001
    return distance*1.0/speed