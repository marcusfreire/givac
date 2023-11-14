from .intelligent_intersection_with_modification import *

class Intelligent_Intersection_DynamicPlatoons(Intelligent_Intersection_MOD):

    def __init__(self,NUM_VEH_PLATOON, seed = 42,ADD_PLATOON_PRO = 0.3, ADD_PLATOON_STEP = 600):
        super().__init__(seed,ADD_PLATOON_PRO = ADD_PLATOON_PRO, ADD_PLATOON_STEP = ADD_PLATOON_STEP)

        self.mod_step_length = 10
        self.step_lenght = 0.01
        self.NUM_VEH_PLATOON = NUM_VEH_PLATOON

        self.DISTANCE_MAX = self.NUM_VEH_PLATOON * VEHICLE_LENGTH + (self.NUM_VEH_PLATOON-1)*DISTANCE
        self.dic_leader = {i:[] for i in range(LANE_NUM)}

    def start(self,sumo_cmd,number_steps,number_steps_communication_for_second=0.1,step_lenght = 0.01):
        '''Função para executar o SUMO atraves do TRACI 
        Tamanho padrão do step_lenght = 0.01 do SUMO, ou seja 10 ms, então para cada 1 segundo
        o simulador executa 100 passos com o tamanho do step padrão 0.01
        Segundo a documentação do SUMO o intervalo do step_lenght  [0.001 and 1.0] 
        Ex.: https://sumo.dlr.de/docs/Simulation/Basic_Definition.html#Defining_the_Time_Step_Length'''
        self.step_lenght = step_lenght
        if (self.step_lenght > number_steps_communication_for_second):
            # raise the ValueError
            raise ValueError("The value of number_steps_communication_for_second must be greater than step_length") 
         
        self.mod_step_length = number_steps_communication_for_second/step_lenght
                
        traci.start(sumo_cmd)
          
        # plexe é chamado após cada traci.simulationStep() 
        plexe = Plexe()
        traci.addStepListener(plexe)
        while self.step < number_steps:  # total steps     
            traci.simulationStep()

            if self.step % self.mod_step_length == 1:
                # simulate vehicle communication every number_steps_for_communication  
                self.verificarveiculos()              
                self.veh_data.general_data_collect(self.step,self.topology)
                communicate(plexe, self.topology)
                self.radar_distance(plexe)
                
                self.check_all_leaders()

                self.smart_traffic_light(plexe)
            
            if self.step % 100 == 5:
                # at 1 second, let the joiner get closer to the platoon
                self.get_distance_between_leaders(plexe,distance_max = self.DISTANCE_MAX)
    #             imprimirTopologia(topology)

            if self.step % self.ADD_PLATOON_STEP == 0:  # add new platoon every X steps
                self.state= self.add_platoons(plexe, self.topology, self.step,self.state,self.ADD_PLATOON_PRO, self.ADD_PLATOON_STEP)   

            self.step += 1

        traci.close()
    
    def check_all_leaders(self):
        # check all leaders to decide whether add to serving list or delete from topology
        deleted_veh = []
        for key, value in list(self.topology.items()):            
            if 'ISleader' in self.topology[key]:  # if it is a leader
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
            self.excluir_platoon_topology(veh,int(veh_route))

        # delete vehcles from the list which has already passed the intersection
        self.serving_list[:] = [element for element in self.serving_list if traci.vehicle.getDistance(element[0]) < 400 + self.topology[element[0]]["distance"] + STOP_LINE]
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
                self.serving_list[i][2] = self.compute_leaving_time(veh_i,self.topology)                
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


    def find_desired_speed_no_conflict(self,i:int,distance_veh_i,speed_veh_i):
        # if no conflict any more, reset the veh to run as expected.
        self.serving_list[i][3] = 0
        veh_i = self.serving_list[i][0]
        PLATOON_LENGTH = self.topology[veh_i]["distance"]
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
        veh_i = self.serving_list[i][0]
        PLATOON_LENGTH = self.topology[veh_i]["distance"]
        self.serving_list[i][2] = (distance_to_stop_line + PLATOON_LENGTH + 2*STOP_LINE)/current_speed

        return desired_speed

    def join_platoon(self,plexe, front_join_id, joiner_id,laneID):
        
        if 'leader' in self.topology[front_join_id]:
            LEADER = self.topology[front_join_id]["leader"]
        else:
            LEADER = front_join_id
            self.topology[LEADER]["second"] = joiner_id
        
        #Adicionar o ultimo carro adicionado ao plator
        if self.topology[joiner_id]["tail"]!= None: #Verificar se ja é líder de outro pelotão
            #Reiniciar a liderança do segundo pelotão
            tailID = self.topology[joiner_id]["tail"]
            while tailID != joiner_id:
                distance = get_distance(plexe, LEADER, tailID)
                if distance < self.DISTANCE_MAX:
                    self.topology[tailID]["leader"] = LEADER
                    tailID = self.topology[tailID]["front"]
                else:
                    tailID_TEMP= self.topology[tailID]["front"]
                    self.topology[joiner_id]["tail"] = tailID_TEMP
                    self.topology[tailID] = {"ISleader": True, "tail": None, "second": None, "distance": VEHICLE_LENGTH}
                    plexe.set_active_controller(tailID, ACC)
                    traci.vehicle.setColor(tailID, (255,255,255))  # white
                    
                    
                    self.dic_leader[laneID].append(tailID)
                    self.dic_leader[laneID] = self.order_vet_veh(self.dic_leader[laneID],laneID)
                    
                    tailID = tailID_TEMP
                    
                
            self.topology[LEADER]["tail"] = self.topology[joiner_id]["tail"]
        else:
            self.topology[LEADER]["tail"] = joiner_id
            
        self.topology[joiner_id] = {"leader": LEADER, "front": front_join_id} 
        
        plexe.set_active_controller(joiner_id, CACC)
        plexe.set_path_cacc_parameters(joiner_id, distance=DISTANCE)
        
        traci.vehicle.setColor(joiner_id, (200,200,0))  # Amarelo
        
        self.dic_leader[laneID].remove(joiner_id)
        #Atualizar distância entre o Lider e o último carro
        self.calc_platoon_size(plexe,LEADER,self.topology)

    def get_distance_between_leaders(self,plexe,distance_max = 25):
        
        for laneID,leaders in self.dic_leader.items():
            qtd_lead = len(leaders)
            if qtd_lead > 1:
                main_leader = leaders[0]
                if self.topology[main_leader]["tail"] != None:# Atualizando distância do plator
                    self.calc_platoon_size(plexe,main_leader,self.topology)
                
                for leadID in leaders[1:]:
    #                 print("main_leader:{} \t leadID:{}".format(main_leader,leadID))
                    if self.topology[leadID]["tail"] != None:# Atualizando distância do plator
                        self.calc_platoon_size(plexe,leadID,self.topology)
                
                    distance = get_distance(plexe, main_leader, leadID)
                    
    #                 print("[{}-{}]: {}".format(main_leader,leadID,distance))
                    if distance < distance_max:
                        if self.topology[main_leader]["tail"]== None: # Lider sem pelotão
                            self.join_platoon(plexe, main_leader, leadID,laneID)
                        else:
                            self.join_platoon(plexe, self.topology[main_leader]["tail"], leadID,laneID)
    #                     imprimirTopologia(topology)
                    else:
                        main_leader = leadID                   
            elif ((qtd_lead == 1) and (self.topology[leaders[0]]["tail"] != None)):
                self.calc_platoon_size(plexe,leaders[0],self.topology)

    @staticmethod
    def order_vet_veh(vet,laneID):
        vet_temp = [int(i.split(".")[1]) for i in vet]
        vet_temp.sort()
        return ["v.%d.%d.%d" %(i, laneID, 0) for i in vet_temp]
    
    @staticmethod
    def calc_platoon_size(plexe,vLeadID,topology):
        #Calcular a distância entre o 1º e ultimo carro, dessa forma adiciona o
        # tamanho dos dois carros e mais 1 metro de segruança.
        # esse será o tamanho de nosso pelotão
        distance = get_distance(plexe, vLeadID, topology[vLeadID]["tail"])
        topology[vLeadID]["distance"] = distance + 2* VEHICLE_LENGTH + 1

    def excluir_platoon_topology(self,leadID,laneID):
    
        if self.topology[leadID]["tail"] != None: #Verificar se é lider unitario        
            tailID = self.topology[leadID]["tail"]
            while tailID != leadID:
                tailID_TEMP= self.topology[tailID]["front"]
                del self.topology[tailID]
                tailID = tailID_TEMP
        del self.topology[leadID]
        self.dic_leader[laneID].remove(leadID)
        return self.topology

    def radar_distance(self,plexe):
        min_dist = 1e6
        for values in self.dic_leader.values():
            for leadID in values:
                if self.topology[leadID]["tail"]!= None: # verificar se tem um pelotão
                    radar = plexe.get_radar_data(self.topology[leadID]["second"])
                    if radar[RADAR_DISTANCE] < min_dist:
                        min_dist = radar[RADAR_DISTANCE]
    
    @staticmethod
    def imprimirTopologia(topology):
        print("Step:",traci.simulation.getTime())
        for i in topology.keys():
            print('\t',i,':',topology[i])

    def add_single_platoon(self,plexe, topology, step, lane):
        for i in range(PLATOON_SIZE):
            vid = "v.%d.%d.%d" %(step/self.ADD_PLATOON_STEP, lane, i)
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
                #Para a formação de pelotão dinâmica
                topology[vid] = {"ISleader": True, "tail": None, "second": None, "distance": VEHICLE_LENGTH}
                self.dic_leader[lane].append(vid)
                self.dic_leader[lane] = self.order_vet_veh(self.dic_leader[lane],lane)
            else:
                plexe.set_active_controller(vid, CACC)
                traci.vehicle.setColor(vid, (200,200,0, 255)) # yellow
                topology[vid] = {"front": "v.%d.%d.%d" %(step/self.ADD_PLATOON_STEP, lane, i-1), "leader": "v.%d.%d.0" %(step/self.ADD_PLATOON_STEP, lane)}

    def add_platoons(self,plexe, topology, step,state, ADD_PLATOON_PRO = 0.3, ADD_PLATOON_STEP = 600):
        random.setstate(state)
        for lane in range(LANE_NUM):    # lane 0~11
            if random.random() < ADD_PLATOON_PRO:
                self.add_single_platoon(plexe, topology, step, lane)
        
        return random.getstate()
    
    @staticmethod
    def compute_leaving_time(veh,topology):
        PLATOON_LENGTH = topology[veh]["distance"]
        distance = 400 + PLATOON_LENGTH + STOP_LINE - traci.vehicle.getDistance(veh)
        speed = traci.vehicle.getSpeed(veh) + 0.00001
        return distance*1.0/speed