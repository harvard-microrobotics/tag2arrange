import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import sys

from matplotlib import rcParams
import seaborn as sns



def main():
    sns.set_style("ticks")
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams["font.serif"] = "Arial"
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14    
    base_folder="/home/ranstlouis/Documents/data/"

    df = pd.read_csv(os.path.join(base_folder,'cycle_data.csv'))
    #print(df)
    arrangements = df['arrangement'].unique()
    measured = np.array([])
    actual = np.array([])
    
    df['ori_z_deg'] = np.rad2deg(df['ori_z'])
    actual =  df['arrangement'].values
    measured = df['ori_z_deg'].values
    df['error'] = actual - measured

    #sns.lineplot(data=df, x="arrangement", y='error', err_style='bars', marker="o", linestyle="")


    #for arrange in arrangements:
    #    df_arrange = df.loc[df['arrangement'] == arrange]  
    #    ori_deg = df_arrange['ori_z'].values.mean() * 180 / np.pi
    #    measured = np.concatenate((measured, np.array([ori_deg])))     
    #    actual = np.concatenate((actual,np.array([arrange]))) 
    #    error = actual - measured

    group =df.groupby(["arrangement"])
    err=group['error'].mean()
    err_std=group['error'].std()
    actual=group['arrangement'].mean()
    print("Average Accuracy Error: %f degrees" % err.mean())
    print("Max Accuracy Error: %f degrees" % err.max())
    print("Average Precision Error: %f degrees" % err_std.mean())
    print("Max Precision Error: %f degrees" % err_std.max())
    #print(actual,measured,error)
    #fig = plt.figure(figsize = (3.48, 2))
    plt.errorbar(actual[1:],err[1:],yerr=err_std[1:],marker='*', linestyle="", capsize=4,markersize=10)
    plt.xlabel("Arrangement Values")
    plt.ylabel("Error (degrees)")
    plt.title("Arrangement Accuracy and Precision Test")
    
    sns.despine()
    ax= plt.gca()
    plt.ylim([0,5])
    
    plt.axhline(y=4, color='b', linestyle='--')
    plt.text(70, 4.2, "Critical Spec Line", ha='left', va="top")
    
    plt.text(ax.get_xlim()[0]+5, ax.get_ylim()[1]-0.05, "N=10", ha='left', va="top")
    out_filename = "arrange_test"
    #plt.savefig(os.path.join(base_folder,out_filename+'.png'),dpi=300)
    #plt.savefig(os.path.join(base_folder,out_filename+'.svg'))
    #plt.savefig(os.path.join(base_folder,out_filename+'.pdf'))
    plt.show()
    
 

if __name__ == '__main__':
    main()