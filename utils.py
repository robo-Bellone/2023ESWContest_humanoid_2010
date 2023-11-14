import numpy as np
import time

def load_from_file(file_name):
    print(f'loading {file_name}...\n')
    with open(f'{file_name}.txt', "r") as f:
        data = []
        for l in f:
            data.append(float(l))
        return data
        
def load_from_dat(file_name):
    print(f'loading {file_name}...\n')
    with open(f'{file_name}', "r") as f:
        data = []
        for l in f:
            print(l)
            data.append(l)
        return data

'''
get_data_from_file - get single value of data from config file you wrote before

    input
        file_name: str data
        data_name: str data


    output
        data: data which you wrote, type is automatically evaluated.
'''
def get_data_from_file(file_name, data_name):
    print(f'loading {data_name} from {file_name}')
    with open(f'{file_name}.txt', "r") as f:
        data = []
        for l in f:
            if l.split("=")[0] == data_name:
                data = i.split("=")[1]
                print(f"{data_name} = {data}")
                if type(data) == type("a"):
                    return data
                return eval(data)

        print(f"Can't find data name with {data_name}. abort")
        return -1

'''
color_getter - get color from config file with matching camera name

    input
        cam_name: str data label the camera name (ex: logi, labtop, '' means default)
        color: str data of color which you want to get

    output
        lower: list with 3 elements which you want to cut low
        upper: list with 3 elements which you want to cut high
'''
def color_getter(cam_name, color):
    data = load_from_file(f'config_{cam_name}_{color}')
    lower = [data[0], data[1], data[2]]
    upper = [data[3], data[4], data[5]]
    return lower, upper
    
def get_color_from_dat(list):
    data = load_from_dat('Cts5_v1.dat')
    lower = []
    upper = []
    for i in list:
        lower.append((int(data[i].split(',')[2]), int(data[i].split(',')[4]), int(data[i].split(',')[6])))
        upper.append((int(data[i].split(',')[1]), int(data[i].split(',')[3]), int(data[i].split(',')[5])))
        
    return lower[0], upper[0], lower[1], upper[1]

class Utilization:

    nt_set = 0
    set_time = 1
    def __init__(self, flag, call_time):
        name = flag
        self.set_time = call_time
        self.nt_set = time.time()
    def is_ready(self):
        if time.time() - self.nt_set > self.set_time:
            return True
        else:
            return False
    def call(self):
        self.nt_set = time.time()


    

        
