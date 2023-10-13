from .intelligent_intersection import *

class Intelligent_Intersection_MOD(Intelligent_Intersection):

    def __init__(self, seed = 42,ADD_PLATOON_PRO = 0.3, ADD_PLATOON_STEP = 600):
        super().__init__(seed,ADD_PLATOON_PRO = ADD_PLATOON_PRO, ADD_PLATOON_STEP = ADD_PLATOON_STEP)

        self.mod_step_length = 10
        self.step_lenght = 0.01

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
                
                self.check_all_leaders()

                self.smart_traffic_light(plexe)

            if self.step % self.ADD_PLATOON_STEP == 0:  # add new platoon every X steps
                self.state= add_platoons(plexe, self.topology, self.step,self.state,self.ADD_PLATOON_PRO, self.ADD_PLATOON_STEP)   

            self.step += 1

        traci.close()

    def verificarveiculos(self):
        deleted_veh = []
        for key in list(self.topology.keys()):
            try:
                speed_teste = traci.vehicle.getSpeed(key)           
            except:
                print("ERRO: car completed the route:",key)
                deleted_veh.append(key)
        self.del_veh_topology_and_intersection(deleted_veh)