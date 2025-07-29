import matplotlib.pyplot as mtplt
import math as mth
#plt.plot(x, y)
class CircleAnon:
    def __init__(self,radius,err_dist,theta_step):
        self.x = []
        self.y = []
        self.azi_theta_buff = []
        self.geo_r_buff = []
        self.r = 2
        #initial
        self.y_tmp = 0
        self.x_tmp = 0
        self.azi_theta = 0
        self.err_dist = err_dist
        self.radius = radius
        self.x_max = self.radius+err_dist
        self.y_max = self.radius+err_dist
        self.theta_step = theta_step
        self.theta = 0
        self.n_circles = 3
        for i in range(1,int(self.radius)+1):
            r = self.r*i

            if r<=self.radius+self.err_dist:
                while (self.theta <= (2*mth.pi)):
                    self.x_tmp = r * mth.cos(self.theta)
                    self.x.append(self.x_tmp)
                    self.y_tmp = r * mth.sin(self.theta)
                    self.y.append(self.y_tmp)
                    self.azi_theta = (( (-self.theta + (mth.pi/2)) + (2*mth.pi) ) % (2*mth.pi) ) * 180 / mth.pi
                    #print("azi theta,r:\n   {0}\n   {1}\n".format(self.azi_theta,self.geo_r))
                    self.geo_r_buff.append(r)
                    self.azi_theta_buff.append(self.azi_theta)
                    self.theta=self.theta+self.theta_step
            self.theta = 0
        self.theta = 0
        r = self.radius+self.err_dist
        while (self.theta <= (2*mth.pi)):
            self.x_tmp = r * mth.cos(self.theta)
            self.x.append(self.x_tmp)
            self.y_tmp = r * mth.sin(self.theta)
            self.y.append(self.y_tmp)
            self.azi_theta = (( (self.theta + (mth.pi/2)) + (2*mth.pi) ) % (2*mth.pi) ) * 180 / mth.pi
            #print("azi theta,r:\n   {0}\n   {1}\n".format(self.azi_theta,self.geo_r))
            self.geo_r_buff.append(r)
            self.azi_theta_buff.append(self.azi_theta)
            self.theta=self.theta+self.theta_step

    
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
        mtplt.plot(self.x,self.y)
        area = mtplt.Circle((0.0,0.0),self.radius,fill=False)
        axes.add_artist(area)

        for i in range(len(self.x)):
            circle = mtplt.Circle((self.x[i],self.y[i]),0.5,fill = False)
            axes.add_artist(circle)

        mtplt.title("Map of waypoints")
        mtplt.xlabel("[m]")
        mtplt.ylabel("[m]")
        mtplt.xlim(-(self.r*self.radius+2),self.r*self.radius+2)
        mtplt.ylim(-(self.r*self.radius+2),self.r*self.radius+2)
        mtplt.grid(visible=True)
        mtplt.minorticks_on()
        mtplt.show()

#trzeba manipulowac troche zeby bylo dobre pokrycie dla danego promienia
if __name__ == "__main__":
    shelly = CircleAnon(2,1,mth.pi*(1/8))
    shelly.print_azi_theta_buffer()
    shelly.print_geo_r_buffer()
    shelly.PlotShelly()
