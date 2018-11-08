# -*- coding: utf-8 -*- 
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
# 每次产生一个新的坐标点 

th1 = [1,2,3,4,5,6,7,8]
th2 = [1,2,3,4,5,6,7,8]

def data_gen(): 
  t = data_gen.t 
  cnt = 0
  while cnt < len(th1): 
    
    t += 0.05
    yield th1[cnt], th2[cnt] 
    cnt+=1
data_gen.t = 0
# 绘图 
fig, ax = plt.subplots() 
line, = ax.plot([], [], lw=2) 
ax.set_ylim(0,10) 
ax.set_xlim(0, 10) 
ax.grid() 
xdata, ydata = [], [] 
# 因为run的参数是调用函数data_gen,
# 所以第一个参数可以不是framenum:设置line的数据,返回line 

def run(data): 
  # update the data 
  t,y = data 
  xdata.append(t) 
  ydata.append(y) 
  xmin, xmax = ax.get_xlim() 
  if t >= xmax: 
    ax.set_xlim(xmin, 2*xmax) 
    ax.figure.canvas.draw() 
  line.set_data(xdata, ydata) 
  return line, 
# 每隔10秒调用函数run,run的参数为函数data_gen, 
# 表示图形只更新需要绘制的元素 
ani = animation.FuncAnimation(fig, run, data_gen, blit=True, interval=200, 
  repeat=False) 
plt.show() 