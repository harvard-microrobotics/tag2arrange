import yaml
import pandas as pd
import os
import sys 
import numpy as np
import pickle
from pypmml import Model

base_folder="/home/ranstlouis/catkin_ws/src/tag2arrange/config/items"
#Load csv 
def main():
	file = 'items.csv'
	df = load_convert(file) 
	df['Aspect Ratio'] = df['Length (mm)'] / df['Width (mm)']
	#Create data to predict
	data = [df['Aspect Ratio'],df['Height (mm)'],df['Object Shape']] 
	headers = ['Aspect Ratio','Height','Object Shape']
	data_df = pd.concat(data, axis=1, keys=headers)
	arranges = predict(data_df) #[45]#
	#Create yaml from prediction and item csv
	num_items = df.shape[0] 	
	item_dict = {}
	for i in range(num_items):
		item_info= df.iloc[i]
		tag_id = item_info['Tag ID']
		name = item_info['Name']
		arrange = arranges[i]
		height = item_info['Height (mm)']		
		item_dict[str(tag_id)] = {'name': name,'arrange': float(arrange),'height': float(height)/2/1000}
	item_list = {'items':{'items':item_dict}}
	with open(os.path.join(base_folder,'objects.yaml'), 'w') as file:
		item_file=yaml.dump(item_list, file)
	
def load_convert(file):
	'''
		Convert the csv to a pandas dataframe
	'''
	#base_folder="/home/ranstlouis/catkin_ws/src/tag2arrange/config/items"
	df = pd.read_csv(os.path.join(base_folder,file))
	return df
def predict(data_test):
	'''
		Load the trained classifier and predict the arrangement value
	'''
	file = os.path.join(base_folder,'SVC.pmml')	
	model = Model.fromFile(file)
	result = model.predict(data_test)
	print(result)
	x = result['predicted_Arrangement']
	y = (np.array(x)-1)/4 * 90
	return y
if __name__ == '__main__':
    main()