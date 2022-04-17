import threading
import time
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from AStarPlanner import AStarPlanner
# from Astar import AStarPlanner

## TODO
# from planner import AStarPlanner,RRTPlanner

plt.rcParams["figure.figsize"] = [8.0,8.0]
plt.rcParams["figure.autolayout"] = True
plt.rcParams['keymap.save'].remove('s')

class Playground:
    def __init__(self,planner=None):
        self.dt = 0.2

        self.fig, self.ax = plt.subplots()

        self.fig.canvas.mpl_connect('button_press_event', self.on_mousepress)
        self.fig.canvas.mpl_connect('key_press_event', self.on_press)
        self.fig.canvas.mpl_connect('motion_notify_event',self.on_mousemove)
        self.NEED_EXIT = False

        ############################################

        self.planning_minx = -10
        self.planning_miny = -10
        self.planning_maxx =  10
        self.planning_maxy =  10

        self.planning_obs = np.empty(shape=(0,2))
        self.planning_obs_radius = 1.5
        self.planning_path = np.empty(shape=(0,2))

        self.planning_start = np.array([0.0,0.0])
        self.planning_target = None

        self.planner = planner

        #####################################
        self.temp_obs = [0,0]

    def run(self):
        while True:
            if self.NEED_EXIT:
                plt.close("all")
                break
            plt.cla()
            self.__draw()
    
    def add_obs(self,x,y):
        self.planning_obs = np.append(self.planning_obs,[[x,y]],axis=0)
    def add_obss(self,xs,ys):
        self.planning_obs = np.append(self.planning_obs,np.vstack([xs,ys]).T,axis=0)
    def __draw(self):
        assert(self.planning_path.shape[1]==2,"the shape of planning path should be '[x,2]', please check your algorithm.")
        assert(self.planning_obs.shape[1]==2,"the shape of self.planning_obs(obstacles) should be '[x,2]', please check your algorithm.")

        self.ax.plot(self.planning_start[0],self.planning_start[1],"k>",markersize=12)
        if self.planning_target is not None:
            self.ax.plot(self.planning_target[0],self.planning_target[1],"r*",markersize=20)

        self.ax.plot(self.planning_path[:,0], self.planning_path[:,1], 'b--')
        for obs in self.planning_obs:
            self.ax.add_artist(plt.Circle((obs[0],obs[1]), self.planning_obs_radius,fill=False))

        self.ax.set_xlim(self.planning_minx, self.planning_maxx)
        self.ax.set_ylim(self.planning_miny, self.planning_maxy)

        plt.pause(self.dt)

    def on_mousepress(self,event):
        if not event.dblclick:
            if event.button == 1:
                self.planning_start = np.array([event.xdata,event.ydata])
            if event.button == 3:
                self.planning_target = np.array([event.xdata,event.ydata])
            if event.button == 2:
                self.add_obs(event.xdata,event.ydata)
                self.temp_obs = [event.xdata,event.ydata]
    def on_mousemove(self,event):
        if hasattr(event,"button") and event.button == 2:
            dx = event.xdata-self.temp_obs[0]
            dy = event.ydata-self.temp_obs[1]
            if np.hypot(dx,dy) > self.planning_obs_radius*0.8:
                self.temp_obs = [event.xdata,event.ydata]
                self.add_obs(*self.temp_obs)
    def on_press(self,event):
        if(event.key == 'escape'):
            self.set_exit()
        if(event.key == ' '):
            print("---------------------------------\ndo planning...")
            print("obstacles : ",self.planning_obs)
            print("plan start : ",self.planning_start)
            print("plan target : ",self.planning_target)
            if self.planning_target is not None and self.planner is not None:
                ##### TODO
                # px,py = planner.planning(self.planning_obs[:,0],self.planning_obs[:,1], self.planning_start[0],self.planning_start[1],self.planning_target[0],self.planning_target[1],self.planning_minx,self.planning_miny,self.planning_maxx,self.planning_maxy)
                px,py = planner.planning(self.planning_obs[:,0],self.planning_obs[:,1], 1.5, self.planning_start[0],self.planning_start[1],self.planning_target[0],self.planning_target[1],self.planning_minx,self.planning_miny,self.planning_maxx,self.planning_maxy)

                self.planning_path = np.vstack([px,py]).T
                print("plan path : ",self.planning_path)
            else:
                print("planner or target is None,please check again.")

    def set_exit(self):
        self.NEED_EXIT = True

if __name__ == "__main__":
    planner = None
    # planner = AStarPlanner(0.1, 1.5)
    planner = AStarPlanner(0.05)

    # planner = RRTPlanner(0.2,1.5)
    pg = Playground(planner)
    pg.run()