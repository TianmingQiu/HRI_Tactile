#!/usr/bin/env python
import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
from sklearn.decomposition import PCA
import numpy as np
import math
import scipy.io as sio
from sklearn.svm import SVC
from sklearn import svm

#global variables for training dataset
X_slap=np.zeros((30,16))
X_push=np.zeros((30,16))
X_tap3=np.zeros((30,16))
X_poke=np.zeros((30,16))
X_scratch=np.zeros((30,16))
X_punch=np.zeros((30,16))
X=np.zeros((120,16))
y=np.zeros((1,120))

#global variabkes for feature descriptor
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
	
### This part subscribes topics and push the messages' data into an array###


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



### This part reshapes the data in to an N*Num_skincell array for data processing###
def data_reshape(x_before):
	length=len(list(x_before))
	length=length/Num_skincell
	x_reshaped=np.array(list(x_before)).reshape(length,Num_skincell)
	return x_reshaped



### This part builds the feature descriptor. Hjorth parameter Activity/Mobility/Complexity,linear correlateion coefficien and Non linear correlation coefficien. It returns a 1*16 feature vector.###


def activity_fv(data):	
	size_data=data.shape
	Activity=np.zeros((1,size_data[1]))
	for i in range(0,size_data[1]):
		Activity[:,i]=sum((data[:,i]-np.mean(data[:,i]))*(data[:,i]-np.mean(data[:,i])))/size_data[0]
	return Activity

def mobility_fv(data):
	Activity=activity_fv(data)
	size_data=data.shape
	Mobility=np.zeros((1,size_data[1]))
	Activity_diff=np.zeros((1,size_data[1]))
	
	for i in range(0,size_data[1]):
		Activity_diff[:,i]=sum((np.diff(data[:,i])-np.mean(np.diff(data[:,i])))*(np.diff(data[:,i])-np.mean(np.diff(data[:,i]))))/(size_data[0]-1)

	for i in range(0,size_data[1]):
		Mobility[:,i]=math.sqrt(Activity_diff[:,i]/Activity[:,i])
	return Mobility

def complexity_fv(data):
	size_data=data.shape
	Mobility=mobility_fv(data)
	Complexity=np.zeros((1,size_data[1]))
	Mobility_diff=np.zeros((1,size_data[1]))
	Activity_diff=np.zeros((1,size_data[1]))
	Activity_diff_diff=np.zeros((1,size_data[1]))
	for i in range(0,size_data[1]):
		Activity_diff[:,i]=sum((np.diff(data[:,i])-np.mean(np.diff(data[:,i])))*(np.diff(data[:,i])-np.mean(np.diff(data[:,i]))))/(size_data[0]-1)
	
	for i in range(0,size_data[1]):
		Activity_diff_diff[:,i]=sum((np.diff(np.diff(data[:,i]))-np.mean(np.diff(np.diff(data[:,i]))))*(np.diff(np.diff(data[:,i]))-np.mean(np.diff(np.diff(data[:,i])))))/(size_data[0]-2)

	for i in range(0,size_data[1]):
		Mobility_diff[:,i]=math.sqrt(Activity_diff_diff[:,i]/Activity_diff[:,i])

	for i in range(0,size_data[1]):
		Complexity[:,i]= Mobility_diff[:,i]/Mobility[:,i]
	return Complexity
	
#linear correlation between two accelermeter sensors
def lcorr_fv(data_1,data_2):
	size_data=data_1.shape 
	Lcorr=np.zeros((1,size_data[1]))
	for i in range(0,size_data[1]):
		Lcorr[:,i]=sum((data_1[:,i]-np.mean(data_1[:,i]))*(data_2[:,i]-np.mean(data_2[:,i])))/(math.sqrt(sum((data_1[:,i]-np.mean(data_1[:,i]))*(data_1[:,i]-np.mean(data_1[:,i]))))*math.sqrt(sum((data_2[:,i]-np.mean(data_2[:,i]))*(data_2[:,i]-np.mean(data_2[:,i])))))
	return Lcorr

#Non-linear-Correlation
def ncorr_fv(data_1,data_2):
	size_data=data_1.shape
	Ncorr=np.zeros((1,size_data[1]))
	for i in range(0,size_data[1]):
		temp_array_1=data_1[:,i]
		temp_array_2=data_2[:,i]
		temp_1 = temp_array_1.argsort()
		temp_2 = temp_array_2.argsort()
		ranks_1 = np.empty(len(temp_array_1), int)
		ranks_2 = np.empty(len(temp_array_2), int)
		ranks_1[temp_1] = np.arange(len(temp_array_1))
		ranks_2[temp_2] = np.arange(len(temp_array_2))
		Ncorr[:,i]=1-(6*(sum((ranks_1-ranks_2)*(ranks_1-ranks_2)))/(size_data[0]*(size_data[0]*size_data[0]-1)))
	return Ncorr
	
def lc_all(data_1,data_2,data_3,data_7):
	data_7[data_7>=0.1]=1
	data_7[data_7<0.1]=0
	size_data_7=data_7.shape
	data_7_bool=np.zeros((1,7))
	for i in range(size_data_7[1]):
		if any(j==1 for j in data_7[:,i]):
			data_7_bool[:,i]=1

	Lc=np.zeros((1,3))
	Lc[:,0]=sum(sum(data_7_bool*lcorr_fv(data_1,data_2)))/sum(sum(data_7_bool))
	Lc[:,1]=sum(sum(data_7_bool*lcorr_fv(data_1,data_3)))/sum(sum(data_7_bool))
	Lc[:,2]=sum(sum(data_7_bool*lcorr_fv(data_2,data_3)))/sum(sum(data_7_bool))
	return Lc

def nc_all(data_1,data_2,data_3,data_7):
	data_7[data_7>=0.1]=1
	data_7[data_7<0.1]=0
	size_data_7=data_7.shape
	data_7_bool=np.zeros((1,7))
	for i in range(size_data_7[1]):
		if any(j==1 for j in data_7[:,i]):
			data_7_bool[:,i]=1
	Nc=np.zeros((1,3))
	Nc[:,0]=sum(sum(data_7_bool*ncorr_fv(data_1,data_2)))/sum(sum(data_7_bool))
	Nc[:,1]=sum(sum(data_7_bool*ncorr_fv(data_1,data_3)))/sum(sum(data_7_bool))
	Nc[:,2]=sum(sum(data_7_bool*ncorr_fv(data_2,data_3)))/sum(sum(data_7_bool))
	return Nc

def axyz_all(data_1,data_7):
	data_7[data_7>=0.1]=1
	data_7[data_7<0.1]=0
	size_data_7=data_7.shape
	data_7_bool=np.zeros((1,7))
	for i in range(size_data_7[1]):
		if any(j==1 for j in data_7[:,i]):
			data_7_bool[:,i]=1
	Ax=np.zeros((1,3))
	Ax[:,0]=sum(sum(data_7_bool*activity_fv(data_1)))/sum(sum(data_7_bool))
	Ax[:,1]=sum(sum(data_7_bool*mobility_fv(data_1)))/sum(sum(data_7_bool))
	Ax[:,2]=sum(sum(data_7_bool*complexity_fv(data_1)))/sum(sum(data_7_bool))
	return Ax

def f_all(data_1,data_2,data_3,data_7):
	data_7[data_7>=0.1]=1
	data_7[data_7<0.1]=0
	size_data_7=data_7.shape
	data_7_bool=np.zeros((1,7))
	for i in range(size_data_7[1]):
		if any(j==1 for j in data_7[:,i]):
			data_7_bool[:,i]=1
	Act=np.zeros((1,1))
	Act_1=data_7_bool*activity_fv(data_1)
	Act_2=data_7_bool*activity_fv(data_2)
	Act_3=data_7_bool*activity_fv(data_3)
	Act[0,0]=sum(sum(Act_1+Act_2+Act_3))/(sum(sum(data_7_bool))*3)
	return Act

def feature_vector_fct(data_1,data_2,data_3,data_4,data_5,data_6,data_7):
	Feature_vector=np.array(lc_all(data_1,data_2,data_3,data_7))
	Feature_vector=np.append(Feature_vector, np.array(nc_all(data_1,data_2,data_3,data_7)), axis=1)
	Feature_vector=np.append(Feature_vector, np.array(axyz_all(data_1,data_7)), axis=1)
	Feature_vector=np.append(Feature_vector, np.array(axyz_all(data_2,data_7)), axis=1)
	Feature_vector=np.append(Feature_vector, np.array(axyz_all(data_3,data_7)), axis=1)
	Feature_vector=np.append(Feature_vector, np.array(f_all(data_4,data_5,data_6,data_7)), axis=1)
	return Feature_vector

def training_data_prepare():
	global X

	for i in range(30):
		X_push[i,:]=np.loadtxt('push_'+str(i+1)+'.dat')
		X_tap3[i,:]=np.loadtxt('tap3_'+str(i+1)+'.dat')
		#X_poke[i,:]=np.loadtxt('poke_'+str(i+1)+'.dat')
		X_punch[i,:]=np.loadtxt('punch_'+str(i+1)+'.dat')
		#X_slap[i,:]=np.loadtxt('slap_'+str(i+1)+'.dat')
		X_scratch[i,:]=np.loadtxt('scratch_'+str(i+1)+'.dat')

	X[0:30,:]=X_push
	X[30:60,:]=X_tap3
	X[60:90,:]=X_scratch
	X[90:120,:]=X_punch
	#X[120:150,:]=X_slap
	y[:,0:30]=1 #push
	y[:,30:60]=2 #tap3
	y[:,60:90]=3 #scratch
	y[:,90:120]=4 #punch
	#y[:,120:150]=5 #slap
	
	y_flat=np.ravel(y)
	clf = svm.LinearSVC()
	clf.fit(X, y_flat) 
	return clf


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

	#reshape the data array for feature descriptor
	if state==False:
		accx_reshaped=data_reshape(accx_array)
		accy_reshaped=data_reshape(accy_array)
		accz_reshaped=data_reshape(accz_array)
		force_1_reshaped=data_reshape(force_1_array)
		force_2_reshaped=data_reshape(force_2_array)
		force_3_reshaped=data_reshape(force_3_array)
		range_reshaped=data_reshape(range_array)
	
		shape_accx=accx_reshaped.shape
		shape_accy=accy_reshaped.shape
		shape_accz=accz_reshaped.shape
		shape_f1=force_1_reshaped.shape
		shape_f2=force_2_reshaped.shape
		shape_f3=force_3_reshaped.shape
		shape_range=range_reshaped.shape
		
		
		if (shape_accx==shape_accy==shape_accz==shape_f1==shape_f2==shape_f3==shape_range):
			shape_all=shape_accx
		else:
			shape_0=[shape_accx[0],shape_accy[0],shape_accz[0],shape_f1[0],shape_f2[0],shape_f3[0]]
			shape_min=shape_0[shape_0.index(min(shape_0))]
			shape_all=(shape_min,Num_skincell)
			
			#reshape the arrays into the same shape as the smallest array
			accx_reshaped=accx_reshaped[0:shape_min-30,:]
			accy_reshaped=accy_reshaped[0:shape_min-30,:]
			accz_reshaped=accz_reshaped[0:shape_min-30,:]
			force_1_reshaped=force_1_reshaped[0:shape_min-30,:]
			force_2_reshaped=force_2_reshaped[0:shape_min-30,:]
			force_3_reshaped=force_3_reshaped[0:shape_min-30,:]
			range_reshaped=range_reshaped[0:shape_min-30,:]
			
		
		fv_touch=feature_vector_fct(accx_reshaped,accy_reshaped,accz_reshaped,force_1_reshaped,force_2_reshaped,force_3_reshaped,range_reshaped)

		np.savetxt('slap_8.dat',fv_touch)



