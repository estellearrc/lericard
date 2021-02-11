import numpy as np
from numpy.linalg import norm, det
import time
try:
    import matplotlib.pyplot as plt
except:
    pass

class Logs:
    def __init__(self, mission, *args):
        self.file = "Logs_mission_" + str(time.time()) + ".csv"
        self.__data_dic = {}
        self.keys_in_order = []
        with open(self.file, "w") as f:
            f.write(mission + "\n")
            for name in args:
                f.write(name + ",")
                self.__data_dic[name] = None
                self.keys_in_order.append(name)
            f.write("\n")
    
    
    @property
    def data_dic(self):
        return self.__data_dic

    
    def update(self, key, value):
        if key in self.keys_in_order:
            self.__data_dic[key] = value
        else:
            print("Warning in logs writing : " + key + " is not defined")
            pass
    
    
    def write_data(self):
        with open(self.file, "a") as f:
            for key in self.keys_in_order:
                f.write(str(self.data_dic[key]) + ",")
            f.write("\n")


