import json,re
import numpy as np
import pandas as pd


data1 = []
data2 = []
print("Hello")
with open("/home/developer/Ossama_Repo/MasterThesisOssama/MATLAB_files/Ossama_Experm/Acceleration_Iden/03_accelerationDeceleration_forward_backward.log", "r") as log_file:
    log_string = log_file.read()
    for i in range(5*2):
        response_string = log_string.split("Chassis (0): ")[i+1].strip()
        s = [float(s) for s in re.findall(r'-?\d+\.?\d*', response_string)]
        if s[1]==1:
            data1.append(s)
        elif s[1]==2:
            data2.append(s)
        
data1 = np.array(data1,dtype=object)
data2 = np.array(data2,dtype=object)


df1 = pd.DataFrame(data1)
df2 = pd.DataFrame(data2)

#df1.to_csv('Data1.csv')
#df2.to_csv('Data2.csv') 
