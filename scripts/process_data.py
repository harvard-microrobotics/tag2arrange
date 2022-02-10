import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
def main():
    base_folder="/home/ranstlouis/Documents/data/"

    df = pd.read_csv(os.path.join(base_folder,'cycle_data.csv'))
    #print(df)
    arrangements = df['arrangement'].unique()
    measured = np.array([])
    actual = np.array([])
    for arrange in arrangements:
        df_arrange = df.loc[df['arrangement'] == arrange]  
        ori_deg = df_arrange['ori_z'].values.mean() * 180 / np.pi
        measured = np.concatenate((measured, np.array([ori_deg])))     
        actual = np.concatenate((actual,np.array([arrange]))) 
        error = actual - measured
        
    #print(actual,measured,error)
    plt.plot(actual,error,'*')
    plt.xlabel("Arrangement Values")
    plt.ylabel("Error (degrees)")
    plt.show()
 

if __name__ == '__main__':
    main()