from .utils import *
from .vehicle_data import *
from math import sqrt

conflict_matrix = {
    0:[], 
    1:[4, 8, 10, 11], 
    2:[4, 5, 7, 11], 
    3:[], 
    4:[1, 2, 7, 11], 
    5:[2, 7, 8, 10],
    6:[],
    7:[2, 4, 5, 10],
    8:[1, 5, 10, 11],
    9:[],
    10:[1, 5, 7, 8],
    11:[1, 2, 4, 8]
    }

class Intelligent_Intersection:


    def __init__(self, seed = 42,ADD_PLATOON_PRO = 0.3, ADD_PLATOON_STEP = 600):
        self.seed= seed
        random.seed(self.seed)
        self.state = random.getstate()

        self.ADD_PLATOON_PRO = ADD_PLATOON_PRO 
        self.ADD_PLATOON_STEP = ADD_PLATOON_STEP
        
        self.step = 0
        self.topology = {}
        self.serving_list = []  # each element: [veh, route, leaving_time, priority]
        self.serving_list_veh_only = []  # each element: veh

        self.veh_data = VehicleData()

    def start(self,sumo_cmd,number_steps):
        traci.start(sumo_cmd)
        # plexe é chamado após cada traci.simulationStep() 
        plexe = Plexe()
        traci.addStepListener(plexe)

        while self.step < number_steps:  # total steps     
            traci.simulationStep()

            if self.step % self.ADD_PLATOON_STEP == 0:  # add new platoon every X steps
                self.state= add_platoons(plexe, self.topology, self.step,self.state,self.ADD_PLATOON_PRO, self.ADD_PLATOON_STEP)
                     
            self.check_all_leaders()

            self.smart_traffic_light(plexe)

            if self.step % 10 == 1:
                # simulate vehicle communication every 0.1s
                communicate(plexe, self.topology)
                self.veh_data.general_data_collect(self.step,self.topology)
            self.step += 1

        traci.close()
    
    def check_all_leaders(self):
        # check all leaders to decide whether add to serving list or delete from topology
        deleted_veh = []
        for key, value in list(self.topology.items()):            
            if value == {}:  # if it is a leader
                odometry = traci.vehicle.getDistance(key)
                # for the first time V2I communication
                if (not key in self.serving_list_veh_only) and (400-V2I_RANGE <= odometry < 400-V2I_RANGE+100): 
                    # add to serving list and initialize by simply setting leaving_time=0, priority=1.
                    self.serving_list.append([key, int(key.split(".")[2]), 0, 1])
                    self.serving_list_veh_only.append(key)
                # record the platoon which has almost finished the route
                if odometry > 800:
                    deleted_veh.append(key)

        # delete vehcles from the list which has already passed the intersection           
        self.del_veh_topology_and_intersection(deleted_veh)

    def del_veh_topology_and_intersection(self,deleted_veh):
        # delete the platoon which has almost finished the route
        for veh in deleted_veh:
            veh_time = veh.split(".")[1]
            veh_route = veh.split(".")[2]
            for i in range(PLATOON_SIZE):                
                veh_id = "v." + veh_time + "." + veh_route + "." + str(i)
                del self.topology[veh_id]     

        # delete vehcles from the list which has already passed the intersection
        self.serving_list[:] = [element for element in self.serving_list if traci.vehicle.getDistance(element[0]) < 400 + PLATOON_LENGTH + STOP_LINE]  
        self.serving_list_veh_only = [element[0] for element in self.serving_list]  

    def smart_traffic_light(self,plexe):
        desired_speed_v2i=-1
        max_leaving_time_v2i = -1
        # update leaving_time for priority 0 and update speed for priority 1
        for i in range(len(self.serving_list)):  # serving_list element = [veh, route, leaving_time, priority]
            veh_i = self.serving_list[i][0]  # veh_ID = "v.time.route.num"
            route_i = int(veh_i.split(".")[2])
            priority = self.serving_list[i][3]

            distance_veh_i = traci.vehicle.getDistance(veh_i)
            speed_veh_i = traci.vehicle.getSpeed(veh_i)

            if priority == 0: # for priority=0, only need to update leaving_time
                self.serving_list[i][2] = compute_leaving_time(distance_veh_i,speed_veh_i)                
            else:
                max_leaving_time_v2i = self.find_max_leaving_time(i,route_i)
                # if no conflict any more, reset the veh to run as expected.
                if max_leaving_time_v2i == 0.00001:
                    desired_speed_v2i = self.find_desired_speed_no_conflict(i,distance_veh_i,speed_veh_i)
                # otherwise, adjust the speed to make sure the veh arrives after max_leaving_time
                else:
                    desired_speed_v2i = self.find_desired_speed_with_conflict(i,distance_veh_i,speed_veh_i,max_leaving_time_v2i)
                
                plexe.set_cc_desired_speed(veh_i, desired_speed_v2i)
            
            self.veh_data.range_intersection_data_collect(self.step,veh_i,desired_speed_v2i,self.serving_list[i][2],max_leaving_time_v2i)


    def find_max_leaving_time(self,i:int,route_i,max_leaving_time = 0.00001):
        # find the max leaving time in conflict routes  
        # initialize the time by a small value            
        for j in range(i):
            veh_j = self.serving_list[j][0]
            route_j = int(veh_j.split(".")[2])
            leaving_time_j = self.serving_list[j][2]
            if (route_j in conflict_matrix[route_i]) and (leaving_time_j > max_leaving_time):
                max_leaving_time = leaving_time_j
        
        return max_leaving_time

    def find_desired_speed_no_conflict(self,i:int,distance_veh_i,speed_veh_i):
        # if no conflict any more, reset the veh to run as expected.
        self.serving_list[i][3] = 0
        distance = 400 + PLATOON_LENGTH + STOP_LINE - distance_veh_i
        desired_speed = sqrt(2 * MAX_ACCEL * distance + (speed_veh_i)**2)
        
        self.serving_list[i][2] = (desired_speed - speed_veh_i) / MAX_ACCEL
            #serving_list[i][2] = compute_leaving_time(veh_i)
        return desired_speed
        
    
    def find_desired_speed_with_conflict(self,i,distance_veh_i,speed_veh_i,max_leaving_time):
         # otherwise, adjust the speed to make sure the veh arrives after max_leaving_time
        distance_to_stop_line = 400 - STOP_LINE - distance_veh_i
        # print("max_leaving_time: ", max_leaving_time)
        # print("distance_to_stop_line: ", distance_to_stop_line)
        current_speed = speed_veh_i + 0.00001  # add a small number to avoid division by zero
        decel = 2 * (current_speed * max_leaving_time - distance_to_stop_line)/(max_leaving_time **2)
        desired_speed = current_speed - decel * max_leaving_time
        #desired_speed = (distance_to_stop_line) / max_leaving_time

        self.serving_list[i][2] = (distance_to_stop_line + PLATOON_LENGTH + 2*STOP_LINE)/current_speed

        return desired_speed
        """
        traci.vehicle.getSpeed(veh_i) + 0.00001  # add a small number to avoid division by zero
        arrive_time = distance_to_stop_line / speed
        if arrive_time < max_leaving_time:
            desired_speed = speed * distance_to_stop_line/(V2I_RANGE-STOP_LINE)
            if speed - desired_speed > DECEL * 0.01:
                desired_speed = speed - DECEL * 0.01
            traci.vehicle.slowDown(veh_i, desired_speed, 0.01)
            #traci.vehicle.setSpeed(veh_i, desired_speed)
        elif arrive_time > max_leaving_time + 0.5:
            #desired_speed = speed + DECEL*0.01
            #if desired_speed > SPEED:
            #    desired_speed = SPEED
            #traci.vehicle.slowDown(veh_i, desired_speed, 0.01) 
            reset_veh(plexe, veh_i)
            #traci.vehicle.setSpeed(veh_i, desired_speed)                   
        serving_list[i][2] = (distance_to_stop_line + PLATOON_LENGTH + 2*STOP_LINE)*1.0/speed
        """