import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd


def draw_ani(x, y):

    def data_gen(x, y):
        t = data_gen.t
        cnt = 0
        while cnt < len(x):
            t += 0.05
            yield x[cnt], y[cnt]
            cnt += 1

    data_gen.t = 0

    fig, ax = plt.subplots() 
    line, = ax.plot([], [], lw=4)
    ax.set_ylim(150, 210)
    ax.set_xlim(-80, -20)
    # ax.grid()
    xdata, ydata = [], []

    def run(data):
        # update the data
        t, y = data
        xdata.append(t)
        ydata.append(y)
        xmin, xmax = ax.get_xlim()
        if t >= xmax:
            ax.set_xlim(xmin, 2 * xmax)
            ax.figure.canvas.draw()
        line.set_data(xdata, ydata)
        return line,
    data = pd.read_csv('demonstrated_traj.csv')
    df=np.transpose(data.values)
    plt.plot(df[0], df[1], lw=4)
    ani = animation.FuncAnimation(fig, run, data_gen(x, y), blit=True, interval=200,
                                  repeat=False)

    plt.show()

