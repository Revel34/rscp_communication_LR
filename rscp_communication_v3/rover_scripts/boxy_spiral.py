# In order to obtain bearing, add current heading
#   to the waypoint azimuth and modulo divide (%) it by 360

# BoxyShelly class:
#  Methods:
#    plot_it()
#       plots waypoints using matplotlib
#    r_buffer()
#        returns r_buffer with waypoints radiuses relative to the current position
#    r_azi_buffer()
#        returns azimuths of waypoints relative to the current heading, 
#        clockwise 0-360 degrees
#    print_r()
#       prints contents of the main two buffers with radiuses and azimuths
#       relative to the current heading
#     


# import matplotlib.pyplot as plt
# import matplotlib.patches as Circle
import math as mt
# import itertools
# import numpy as np
class  BoxyShelly:
    def __init__(self,radius,err_space):
        # Initialize starting point
        self.x, self.y = [0], [0]
        self.start_r = 1
        self.off = 1
        self.radius = radius
        self.error_space = err_space
        # Directions: right, up, left, down
        self.directions = [(0, 1), (-1, 0), (0, -1), (1, 0)]
        self.current_direction = 0  # Start moving right
        self.counter = 1
        # Build the spiral with two steps per r
        # self.x.append(self.x[-1] + (self.start_r+self.off)/2)
        # self.y.append(self.y[-1] - (self.start_r+self.off)/2)
        self.r_azi_buff = []
        self.r_buff = []
        self.r = self.error_space*2
        self.r_step = self.r/2
        while self.r<(self.error_space*20):
            for _ in range(2):  # Take two turns for each r
                if self.counter < (self.radius*20)*2: 
                    dx, dy = self.directions[self.current_direction % 4]
                    tmpx = [self.x[-1] + dx * self.r / 4,
                           self.x[-1] + dx * self.r / 2.0,
                           self.x[-1] + dx * self.r * 3/4,
                           self.x[-1] + dx * self.r
                            ]
                    tmpy = [self.y[-1] + dy * self.r / 4,
                           self.y[-1] + dy * self.r / 2.0,
                           self.y[-1] + dy * self.r * 3/4,
                           self.y[-1] + dy * self.r
                            ]
                    
                    for x,y in zip(tmpx,tmpy):
                        if mt.sqrt((x)**2+(y)**2)<=(self.radius+self.error_space*2):
                            self.x.append(x)
                            self.y.append(y)
                    self.current_direction += 1
                    self.counter += 1
                    self.r +=self.r_step

        for i in range(1,len(self.x)):
            bearing = mt.atan2(self.y[i],self.x[i]) 
            bearing = ( bearing if bearing >= 0 else bearing + (2*mt.pi) ) * 180 / mt.pi
            bearing = ((5*180/2)-bearing) % (360)
            self.r_azi_buff.append(bearing)
            r = mt.sqrt( ((self.x[i])**2) + ((self.y[i])**2) )
            self.r_buff.append(r)

    def r_buffer(self):
        return self.r_buff
    
    def r_azi_buffer(self):
        return self.r_azi_buff
        # Plot the spiral
        
    # def plot_it(self):
    #     fig,ax = plt.subplots()
    #     area = plt.Circle((0,0),self.radius,fill=False)
    #     ax.add_artist(area)
    #     for i in range(0,len(self.r_buff)+1):
    #         errcircle = plt.Circle((self.x[i],self.y[i]),self.error_space,fill=True)
    #         ax.add_artist(errcircle)
    #     plt.scatter(self.x,self.y,marker='o',color='m')
    #     plt.grid(True)
    #     plt.axis('equal')     
    #     plt.show()

    def print_r(self):
        print("r is {}: \n{}\nr_azi: \n{}".format(len(self.r_buff),self.r_buff,self.r_azi_buff))

# new = BoxyShelly(2,0.2)
# new.plot_it()
