import matplotlib.pyplot as mtplt
import math as mth
# Generator of archimedean spiral
class ArchimedeanSpiral:
    def __init__(self,radius,err_dist,a,b,theta,theta_step):
        self.x = []
        self.y = []
        self.azi_theta_buff = []
        self.geo_r_buff = []
        
        #initial
        self.err_dist = err_dist
        self.radius = radius
        self.x_max = self.radius+err_dist+err_dist*3/2
        self.y_max = self.radius+err_dist+err_dist*3/2
        # a is distance of the first point from the center
        self.a = a
        # distance between two points at theta: d = b * theta
        self.b = b
        self.theta = theta
        self.theta_step = theta_step
        self.r = ( self.a + ( self.b * self.theta ) )
        self.x_tmp = self.r * mth.cos(-self.theta)
        self.x.append(self.x_tmp)
        self.y_tmp = self.r * mth.sin(-self.theta)
        self.y.append(self.y_tmp)
        self.geo_r = mth.sqrt(((self.x_tmp)**2)+((self.y_tmp)**2))
        self.azi_theta = (( (self.theta + (mth.pi/2)) + (2*mth.pi) ) % (2*mth.pi) ) * 180 / mth.pi
        self.geo_r_buff.append(self.geo_r)
        self.azi_theta_buff.append(self.azi_theta)
        while (self.theta < (2*mth.pi)):
            self.theta=self.theta+(self.theta_step*2)
            self.r = ( self.a + ( self.b * self.theta ) )
            self.x_tmp = self.r * mth.cos(-self.theta)
            self.x.append(self.x_tmp)
            self.y_tmp = self.r * mth.sin(-self.theta)
            self.y.append(self.y_tmp)
            self.geo_r = mth.sqrt(((self.x_tmp)**2)+((self.y_tmp)**2))
            self.azi_theta = (( (self.theta + (mth.pi/2)) + (2*mth.pi) ) % (2*mth.pi) ) * 180 / mth.pi
            #print("azi theta,r:\n   {0}\n   {1}\n".format(self.azi_theta,self.geo_r))
            self.geo_r_buff.append(self.geo_r)
            self.azi_theta_buff.append(self.azi_theta)
        while (abs(self.x[-1]) < self.x_max) and (abs(self.y[-1]) < self.y_max):
            self.theta=self.theta+self.theta_step
            self.r = ( self.a + ( self.b * self.theta ) )
            self.x_tmp = self.r * mth.cos(-self.theta)
            self.x.append(self.x_tmp)
            self.y_tmp = self.r * mth.sin(-self.theta)
            self.y.append(self.y_tmp)
            self.geo_r = mth.sqrt(((self.x_tmp)**2)+((self.y_tmp)**2))
            self.azi_theta = (( (self.theta + (mth.pi/2)) + (2*mth.pi) ) % (2*mth.pi) ) * 180 / mth.pi
            #print("azi theta,r:\n   {0}\n   {1}\n".format(self.azi_theta,self.geo_r))
            self.geo_r_buff.append(self.geo_r)
            self.azi_theta_buff.append(self.azi_theta)
    
    def azi_theta_buffer(self):
        return self.azi_theta_buff
    
    def geo_r_buffer(self):
        return self.geo_r_buff
    
    def print_azi_theta_buffer(self):
        print("Azimuths buffer:\n {0}".format(self.azi_theta_buff))
    
    def print_geo_r_buffer(self):
        print("Radiuses buffer:\n {0}".format(self.geo_r_buff))
    def PlotShelly(self):
        figure, axes = mtplt.subplots()
        shell = mtplt.scatter(self.x,self.y)
        axes.add_artist(shell)
        area = mtplt.Circle((0.0,0.0),self.radius,fill=False)
        axes.add_artist(area)
        for i in range(len(self.x)):
            circle = mtplt.Circle((self.x[i],self.y[i]),0.5,fill = False)
            axes.add_artist(circle)        
        mtplt.title("Map of waypoints")
        mtplt.xlabel("[m]")
        mtplt.ylabel("[m]")
        mtplt.xlim(-7,7)
        mtplt.ylim(-7,7)
        mtplt.grid(visible=True)
        mtplt.minorticks_on()
        mtplt.show()

#trzeba manipulowac troche zeby bylo dobre pokrycie dla danego promienia
if __name__ == "__main__":
    shelly = ArchimedeanSpiral( 5, 0.3, 1, (3/2)*1/(2*mth.pi), -(mth.pi/2), mth.pi/8)
    shelly.print_azi_theta_buffer()
    shelly.print_geo_r_buffer()
    shelly.PlotShelly()


