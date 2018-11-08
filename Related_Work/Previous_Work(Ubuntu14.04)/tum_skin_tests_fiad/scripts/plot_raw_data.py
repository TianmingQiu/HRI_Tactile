#!/usr/bin/env python
import os,sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
from sklearn.decomposition import PCA
import numpy as np
import math
import scipy.io as sio
from sklearn.svm import SVC
from sklearn import svm
import matplotlib.pyplot as plt

range_bool=np.zeros((1,7)) # the value equals 0 or 1, which represents whether the skincell is touched
accx_array= None
accy_array= None
accz_array= None
force_1_array= None
force_2_array= None
force_3_array= None
range_array= None

#other global variables
Num_skincell=0
schalter=False
i=0
hand=False
state=False
begin_time=rospy.Time()
t=1
q=0
range_dim17=None

def callback_accx(data):
	global accx_array
	if (schalter==True):
		if accx_array== None:
			accx_array=data.data
		else:
			accx_array=np.concatenate((accx_array,data.data))
	
def callback_accy(data):
	global accy_array
	if (schalter==True):
		if accy_array== None:
			accy_array=data.data
		else:
			accy_array=np.concatenate((accy_array,data.data))

def callback_accz(data):
	global accz_array
	if (schalter==True):
		if accz_array== None:
			accz_array=data.data
		else:
			accz_array=np.concatenate((accz_array,data.data))


def callback_force_1(data):
	global force_1_array
	if (schalter==True):
		if force_1_array== None:
			force_1_array=data.data
		else:
			force_1_array=np.concatenate((force_1_array,data.data))


def callback_force_2(data):
	global force_2_array
	if (schalter==True):
		if force_2_array== None:
			force_2_array=data.data
		else:
			force_2_array=np.concatenate((force_2_array,data.data))

def callback_force_3(data):
	global force_3_array
	if (schalter==True):
		if force_3_array== None:
			force_3_array=data.data
		else:
			force_3_array=np.concatenate((force_3_array,data.data))


def callback_range(data):
	global i
	global range_array
	global schalter
	global hand
	global state
	global begin_time
	global t
	global Num_skincell
	global q

	temp_range=data.data
	Num_skincell=len(temp_range)

	# hand represents whether hand is touching the skincell
	if any(j>=0.1 for j in temp_range):
		hand=True
	else:
		hand=False
	# t controls the state whether begin_time should be updated
	if t==0:
		if state==True and hand==False:
			begin_time=rospy.get_rostime()
			t=t+1
	elif t==1:
		if state==True and hand==True:
			t=t-1
	# reset the variables for next prediction
	'''if q==0:
		if state==False and hand==True:
			i=0
			accx_array= None
			accy_array= None
			accz_array= None
			force_1_array= None
			force_2_array= None
			force_3_array= None
			range_array= None

			accx_reshaped=None
			accy_reshaped=None
			accz_reshaped=None
			force_1_reshaped=None
			force_2_reshaped=None
			force_3_reshaped=None
			range_reshaped=None
			q=q+1
	elif q==1:
		if state==False and hand==False:
			q=q-1'''	
	
	# state represnets whether the programm is gathering data
	if state==True:
		if hand==True:
			schalter=True
		elif hand==False and rospy.get_rostime()<= rospy.Duration(0.5)+begin_time:
			schalter=True
		else: 
			schalter=False
			state=False
	else:
		if hand==True:
			schalter=True
			state=True
		else:
			schalter=False
			state=False
	# gather range data into an array		
	if (schalter==True):
		if range_array == None:
			range_array=data.data	
		else:
			range_array=np.concatenate((range_array,data.data))
		i=i+1
	print i
	print schalter


def callback_accx(data):
	global accx_array
	if (schalter==True):
		if accx_array== None:
			accx_array=data.data
		else:
			accx_array=np.concatenate((accx_array,data.data))
	
def callback_accy(data):
	global accy_array
	if (schalter==True):
		if accy_array== None:
			accy_array=data.data
		else:
			accy_array=np.concatenate((accy_array,data.data))

def callback_accz(data):
	global accz_array
	if (schalter==True):
		if accz_array== None:
			accz_array=data.data
		else:
			accz_array=np.concatenate((accz_array,data.data))


def callback_force_1(data):
	global force_1_array
	if (schalter==True):
		if force_1_array== None:
			force_1_array=data.data
		else:
			force_1_array=np.concatenate((force_1_array,data.data))


def callback_force_2(data):
	global force_2_array
	if (schalter==True):
		if force_2_array== None:
			force_2_array=data.data
		else:
			force_2_array=np.concatenate((force_2_array,data.data))

def callback_force_3(data):
	global force_3_array
	if (schalter==True):
		if force_3_array== None:
			force_3_array=data.data
		else:
			force_3_array=np.concatenate((force_3_array,data.data))


def callback_range(data):
	global i
	global range_array
	global schalter
	global hand
	global state
	global begin_time
	global t
	global Num_skincell
	global q

	temp_range=data.data
	Num_skincell=len(temp_range)

	# hand represents whether hand is touching the skincell
	if any(j>=0.1 for j in temp_range):
		hand=True
	else:
		hand=False
	# t controls the state whether begin_time should be updated
	if t==0:
		if state==True and hand==False:
			begin_time=rospy.get_rostime()
			t=t+1
	elif t==1:
		if state==True and hand==True:
			t=t-1
	# reset the variables for next prediction
	if q==0:
		if state==False and hand==True:
			i=0
			accx_array= None
			accy_array= None
			accz_array= None
			force_1_array= None
			force_2_array= None
			force_3_array= None
			range_array= None

			accx_reshaped=None
			accy_reshaped=None
			accz_reshaped=None
			force_1_reshaped=None
			force_2_reshaped=None
			force_3_reshaped=None
			range_reshaped=None
			q=q+1
	elif q==1:
		if state==False and hand==False:
			q=q-1	
	
	# state represnets whether the programm is gathering data
	if state==True:
		if hand==True:
			schalter=True
		elif hand==False and rospy.get_rostime()<= rospy.Duration(0.5)+begin_time:
			schalter=True
		else: 
			schalter=False
			state=False
	else:
		if hand==True:
			schalter=True
			state=True
		else:
			schalter=False
			state=False
	# gather range data into an array		
	if (schalter==True):
		if range_array == None:
			range_array=data.data	
		else:
			range_array=np.concatenate((range_array,data.data))
		i=i+1
	print i
	print schalter

def data_reshape(x_before):
	length=len(list(x_before))
	length=length/Num_skincell
	x_reshaped=np.array(list(x_before)).reshape(length,Num_skincell)
	return x_reshaped

def plot_array_eachcell(accx_array,accy_array,accz_array,force_1_array,force_2_array,force_3_array,range_array):
	# x-y-z-f1-f2-f3-r
	accx_array_reshape=data_reshape(accx_array)
	accy_array_reshape=data_reshape(accy_array)
	accz_array_reshape=data_reshape(accz_array)
	force_1_array_reshape=data_reshape(force_1_array)
	force_2_array_reshape=data_reshape(force_2_array)
	force_3_array_reshape=data_reshape(force_3_array)
	range_array_reshape=data_reshape(range_array)
	
	shape_accx=accx_array_reshape.shape
	shape_accy=accy_array_reshape.shape
	shape_accz=accz_array_reshape.shape
	shape_f1=force_1_array_reshape.shape
	shape_f2=force_2_array_reshape.shape
	shape_f3=force_3_array_reshape.shape
	shape_range=range_array_reshape.shape

	if (shape_accx==shape_accy==shape_accz==shape_f1==shape_f2==shape_f3==shape_range):
		shape_min=shape_accx[0]
		shape_all=shape_accx
	else:
		shape_0=[shape_accx[0],shape_accy[0],shape_accz[0],shape_f1[0],shape_f2[0],shape_f3[0]]
		shape_min=shape_0[shape_0.index(min(shape_0))]
		shape_all=(shape_min,Num_skincell)
	
	accx_array_reshape=accx_array_reshape[0:shape_min-30,:]
	accy_array_reshape=accy_array_reshape[0:shape_min-30,:]
	accz_array_reshape=accz_array_reshape[0:shape_min-30,:]
	force_1_array_reshape=force_1_array_reshape[0:shape_min-30,:]
	force_2_array_reshape=force_2_array_reshape[0:shape_min-30,:]
	force_3_array_reshape=force_3_array_reshape[0:shape_min-30,:]
	range_array_reshape=range_array_reshape[0:shape_min-30,:]
	
	cell_1=np.zeros((shape_min-30,7))
	cell_1[:,0]=accx_array_reshape[:,0]
	cell_1[:,1]=accy_array_reshape[:,0]
	cell_1[:,2]=accz_array_reshape[:,0]
	cell_1[:,3]=force_1_array_reshape[:,0]
	cell_1[:,4]=force_2_array_reshape[:,0]
	cell_1[:,5]=force_3_array_reshape[:,0]
	cell_1[:,6]=range_array_reshape[:,0]

	cell_2=np.zeros((shape_min-30,7))
	cell_2[:,0]=accx_array_reshape[:,1]
	cell_2[:,1]=accy_array_reshape[:,1]
	cell_2[:,2]=accz_array_reshape[:,1]
	cell_2[:,3]=force_1_array_reshape[:,1]
	cell_2[:,4]=force_2_array_reshape[:,1]
	cell_2[:,5]=force_3_array_reshape[:,1]
	cell_2[:,6]=range_array_reshape[:,1]
	
	cell_3=np.zeros((shape_min-30,7))
	cell_3[:,0]=accx_array_reshape[:,2]
	cell_3[:,1]=accy_array_reshape[:,2]
	cell_3[:,2]=accz_array_reshape[:,2]
	cell_3[:,3]=force_1_array_reshape[:,2]
	cell_3[:,4]=force_2_array_reshape[:,2]
	cell_3[:,5]=force_3_array_reshape[:,2]
	cell_3[:,6]=range_array_reshape[:,2]

	cell_4=np.zeros((shape_min-30,7))
	cell_4[:,0]=accx_array_reshape[:,3]
	cell_4[:,1]=accy_array_reshape[:,3]
	cell_4[:,2]=accz_array_reshape[:,3]
	cell_4[:,3]=force_1_array_reshape[:,3]
	cell_4[:,4]=force_2_array_reshape[:,3]
	cell_4[:,5]=force_3_array_reshape[:,3]
	cell_4[:,6]=range_array_reshape[:,3]

	cell_5=np.zeros((shape_min-30,7))
	cell_5[:,0]=accx_array_reshape[:,4]
	cell_5[:,1]=accy_array_reshape[:,4]
	cell_5[:,2]=accz_array_reshape[:,4]
	cell_5[:,3]=force_1_array_reshape[:,4]
	cell_5[:,4]=force_2_array_reshape[:,4]
	cell_5[:,5]=force_3_array_reshape[:,4]
	cell_5[:,6]=range_array_reshape[:,4]

	cell_6=np.zeros((shape_min-30,7))
	cell_6[:,0]=accx_array_reshape[:,5]
	cell_6[:,1]=accy_array_reshape[:,5]
	cell_6[:,2]=accz_array_reshape[:,5]
	cell_6[:,3]=force_1_array_reshape[:,5]
	cell_6[:,4]=force_2_array_reshape[:,5]
	cell_6[:,5]=force_3_array_reshape[:,5]
	cell_6[:,6]=range_array_reshape[:,5]

	cell_7=np.zeros((shape_min-30,7))
	cell_7[:,0]=accx_array_reshape[:,6]
	cell_7[:,1]=accy_array_reshape[:,6]
	cell_7[:,2]=accz_array_reshape[:,6]
	cell_7[:,3]=force_1_array_reshape[:,6]
	cell_7[:,4]=force_2_array_reshape[:,6]
	cell_7[:,5]=force_3_array_reshape[:,6]
	cell_7[:,6]=range_array_reshape[:,6]
	
	print shape_min 
	axis = np.arange(0,shape_min-30,1)
	print axis
	print len(axis),len(cell_1[:,0])

	plt.figure(1)

	plt.subplot(2,2,1)	
	plt.plot(axis,cell_1[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_1[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_1[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_1[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_1[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_1[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_1')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)
	
	plt.subplot(2,2,2)	
	plt.plot(axis,cell_2[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_2[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_2[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_2[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_2[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_2[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_2')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)

	plt.subplot(2,2,3)	
	plt.plot(axis,cell_3[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_3[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_3[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_3[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_3[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_3[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_3')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)

	plt.subplot(2,2,4)	
	plt.plot(axis,cell_4[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_4[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_4[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_4[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_4[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_4[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_4')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)

	plt.figure(2)

	plt.subplot(2,2,1)	
	plt.plot(axis,cell_5[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_5[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_5[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_5[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_5[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_5[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_5')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)
	
	plt.subplot(2,2,2)	
	plt.plot(axis,cell_6[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_6[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_6[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_6[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_6[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_6[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_6')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)

	plt.subplot(2,2,3)	
	plt.plot(axis,cell_7[:,0],'r',label="x") # accelerometer x
	plt.plot(axis,cell_7[:,1],'b',label="y") # accelerometer y
	plt.plot(axis,cell_7[:,2],'g',label="z") # accelerometer z
	plt.plot(axis,cell_7[:,3],'c',label="f_1") # force 1
	plt.plot(axis,cell_7[:,4],'y',label="f_2") # force 2
	plt.plot(axis,cell_7[:,5],'k',label="f_3") # force 3
	plt.title('Skincell_7')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=1, borderaxespad=0.)

	plt.show()
	
	

if __name__ == '__main__':
	
	rospy.init_node('data_preprocess', anonymous=True)
	rospy.Subscriber("accx_topic",Float32MultiArray,callback_accx)
	rospy.Subscriber("accy_topic",Float32MultiArray,callback_accy)
	rospy.Subscriber("accz_topic",Float32MultiArray,callback_accz)
	rospy.Subscriber("force_1_detail_topic",Float32MultiArray,callback_force_1)
	rospy.Subscriber("force_2_detail_topic",Float32MultiArray,callback_force_2)
	rospy.Subscriber("force_3_detail_topic",Float32MultiArray,callback_force_3)
	rospy.Subscriber("range_topic",Float32MultiArray,callback_range)
	
	rospy.spin()
	
	plot_array_eachcell(accx_array,accy_array,accz_array,force_1_array,force_2_array,force_3_array,range_array)

		



