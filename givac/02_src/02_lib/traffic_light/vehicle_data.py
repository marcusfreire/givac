import pandas as pd
from .utils import *
# from sklearn.metrics import mean_squared_error

class VehicleData:
    """
    Class to store data of a vehicle.
    """
    def __init__(self):
        self.dic = {}
        self.dic_v2i = {}
        self.dic_paket_losses={}
        self.add_variable_dic(self.dic,self.dic_v2i,self.dic_paket_losses)




    @staticmethod
    def add_variable_dic(dic,dic_v2i,dic_paket_losses):
        dic['step']=[]
        dic['key']=[]
        dic['route']=[]
        dic['odometry']=[]
        dic['speed']=[]
        dic['timeLoss']=[]
        dic['current_time']=[]
        
        dic_v2i['step']=[]
        dic_v2i['key']=[]
        dic_v2i['desired_speed']=[]
        dic_v2i['distance_to_stop_line']=[]
        dic_v2i['max_leaving_time']=[]
        dic_v2i['leaving_time']=[]

        dic_paket_losses['step']=[]
        dic_paket_losses['key']=[]
        dic_paket_losses['odo']=[]
        dic_paket_losses['odo_real']=[]
        dic_paket_losses['speed_predicted']=[]
        dic_paket_losses['speed_real']=[]
        # dic_paket_losses["rmse"]=[]
        dic_paket_losses['leaving_time']=[]
        dic_paket_losses["distance_to_stop_line"]=[]
        dic_paket_losses["distance_to_stop_line_real"]=[]
        dic_paket_losses["timeDelta"]=[]


    def general_data_collect(self,step,topology):          
        #Usando Dicionário
        for key in list(topology.keys()):
                self.add_step_dic(step,key)

    def add_step_dic(self,step,key):
        self.dic["step"].append(step)
        odometry = traci.vehicle.getDistance(key)
        self.dic["key"].append(key)
        self.dic["route"].append(int(key.split(".")[2]))
        self.dic["odometry"].append(odometry)
        self.dic["speed"].append(traci.vehicle.getSpeed(key))
        self.dic["timeLoss"].append(traci.vehicle.getTimeLoss(key))
        self.dic["current_time"].append(traci.simulation.getTime())


    def range_intersection_data_collect(self,step,veh_i,desired_speed_v2i,leaving_time_v2i,max_leaving_time_v2i):
        self.dic_v2i["step"].append(step)
        self.dic_v2i["key"].append(veh_i)
        self.dic_v2i["desired_speed"].append(desired_speed_v2i)
        self.dic_v2i["distance_to_stop_line"].append(400 - STOP_LINE - traci.vehicle.getDistance(veh_i))
        self.dic_v2i["max_leaving_time"].append(max_leaving_time_v2i)
        self.dic_v2i["leaving_time"].append(leaving_time_v2i)

    def merge_veh_data(self,type_merge = 'left',keys = ['key','step'], packet_losses=False):
        df_dados = pd.DataFrame(self.dic)
        df_v2i = pd.DataFrame(self.dic_v2i)
        df = pd.merge(left=df_dados,right=df_v2i,how=type_merge,on=keys)
        df['duration'] = df['current_time'] - df.groupby('key')['current_time'].transform('first')

        if packet_losses:
            df_paket_losses = pd.DataFrame(self.dic_paket_losses)
            df_paket_losses = df_paket_losses[['key','step','speed_predicted','timeDelta']]
            df = pd.merge(left=df,right=df_paket_losses,how=type_merge,on=keys)
        
        return df

    def packet_losses_data_collect(self,step,veh_i,odo,speed,leaving_time,timeDelta):
        odo_real = traci.vehicle.getDistance(veh_i)
        speed_real = traci.vehicle.getSpeed(veh_i)

        distance = 400 + PLATOON_LENGTH + STOP_LINE - odo
        distance_real = 400 + PLATOON_LENGTH + STOP_LINE - odo_real

        self.dic_paket_losses['step'].append(step)
        self.dic_paket_losses['key'].append(veh_i)
        self.dic_paket_losses['odo'].append(odo)
        self.dic_paket_losses['odo_real'].append(odo_real)
        self.dic_paket_losses['speed_predicted'].append(speed)
        self.dic_paket_losses['speed_real'].append(speed_real)
        self.dic_paket_losses['leaving_time'].append(leaving_time)
        
        self.dic_paket_losses["distance_to_stop_line"].append(distance)
        self.dic_paket_losses["distance_to_stop_line_real"].append(distance_real)

        self.dic_paket_losses['timeDelta'].append(timeDelta)

        # self.dic_paket_losses["rmse"].append(mean_squared_error([speed_real], [speed], squared=False))