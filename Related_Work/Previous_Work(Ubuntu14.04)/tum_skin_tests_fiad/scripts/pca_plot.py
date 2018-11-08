#!/usr/bin/env python
import rospy
import numpy as np
import math
import scipy.io as sio
from sklearn.svm import SVC
from sklearn import svm
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt


X_push=np.zeros((30,16))
X_tap=np.zeros((30,16))
X_scratch=np.zeros((30,16))
X_punch=np.zeros((30,16))
X=np.zeros((120,16))
y=np.zeros((1,120))

def plot_2D_feature():
	global X

	for i in range(30):
		X_push[i,:]=np.loadtxt('push_'+str(i+1)+'.dat')
		X_tap[i,:]=np.loadtxt('tap_'+str(i+1)+'.dat')
		X_scratch[i,:]=np.loadtxt('scratch_'+str(i+1)+'.dat')
		X_punch[i,:]=np.loadtxt('punch_'+str(i+1)+'.dat')
		
	X[0:30,:]=X_push
	X[30:60,:]=X_tap
	X[60:90,:]=X_scratch
	X[90:120,:]=X_punch

	y[:,0:30]=1 #push
	y[:,30:60]=2 #tap
	y[:,60:90]=3 #scratch
	y[:,90:120]=4 #punch

	

	X = PCA(n_components=2).fit_transform(X)
	X_1=PCA(n_components=2).fit_transform(X_push)
	X_2=PCA(n_components=2).fit_transform(X_tap)
	X_3=PCA(n_components=2).fit_transform(X_scratch)
	X_4=PCA(n_components=2).fit_transform(X_punch)

	
	print X_1,'\n',X_2,'\n',X_3,'\n',X_4

	plt.figure(1)	
	plt.plot(X_1[0:30,0],X_1[0:30,1],'or')
	plt.plot(X_2[0:30,0],X_2[0:30,1],'ob')
	plt.plot(X_3[0:30,0],X_3[0:30,1],'og')
	plt.plot(X_4[0:30,0],X_4[0:30,1],'oy')
	plt.show()
	

if __name__ == '__main__':
	plot_2D_feature()
