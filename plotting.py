import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt 


class plotter:
    def __init__(self,simu=False,bias_stdy=False,path=""):

        self.simu = simu
        print("loading results ...")
        df_dvl = pd.read_csv("/home/maxwmr/Documents/work/simu_analysis_cpp/results/DVL.csv",sep=',')
        df_imu = pd.read_csv("/home/maxwmr/Documents/work/simu_analysis_cpp/results/IMU.csv",sep=',')
    
        self.imuaccel =  np.stack([df_imu['acc1'].to_numpy(),df_imu['acc2'].to_numpy(),df_imu['acc3'].to_numpy()]).T

        self.dvlspeed =  np.stack([df_dvl['speedX'].to_numpy(),df_dvl['speedY'].to_numpy(),df_dvl['speedZ'].to_numpy()]).T
        self.imuspeed =  np.stack([df_imu['speedX'].to_numpy(),df_imu['speedY'].to_numpy(),df_imu['speedZ'].to_numpy()]).T
        if simu:             
            self.refspeed_imutime = np.stack([df_imu['speedX_ref'].to_numpy(),df_imu['speedY_ref'].to_numpy(),df_imu['speedZ_ref'].to_numpy()]).T
            self.refspeed_dvltime = np.stack([df_dvl['speedX_ref'].to_numpy(),df_dvl['speedY_ref'].to_numpy(),df_dvl['speedZ_ref'].to_numpy()]).T

        self.dvlpos =  np.stack([df_dvl['posX'].to_numpy(),df_dvl['posY'].to_numpy(),df_dvl['posZ'].to_numpy()]).T
        self.imupos =  np.stack([df_imu['posX'].to_numpy(),df_imu['posY'].to_numpy(),df_imu['posZ'].to_numpy()]).T
        if simu:   
            self.refpos_imutime = np.stack([df_imu['posX_ref'].to_numpy(),df_imu['posY_ref'].to_numpy(),df_imu['posZ_ref'].to_numpy()]).T
        self.refpos_dvltime = np.stack([df_dvl['posX_ref'].to_numpy(),df_dvl['posY_ref'].to_numpy(),df_dvl['posZ_ref'].to_numpy()]).T

        self.dvltime = df_dvl['time'].to_numpy()
        self.imutime = df_imu['time'].to_numpy()

        if bias_stdy:
            self.estimated_bias = np.stack([df_imu['biasx'].to_numpy(),df_imu['biasy'].to_numpy(),df_imu['biasz'].to_numpy()]).T
            df_imu2 = pd.read_csv("/home/maxwmr/Documents/work/data2/"+path+"/IMU.csv",sep=',') 
            self.true_bias = np.stack([df_imu2['biasx'].to_numpy(),df_imu2['biasy'].to_numpy(),df_imu2['biasz'].to_numpy()]).T
            self.biastime = df_imu2['time'].to_numpy()



        if simu:
            self.reflab = 'ref'
        else:
            self.reflab = 'phins'


        print("done! ")

    

    def plotaccel(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.imutime,self.imuaccel[:,0],label='imu')
        plt.plot(self.dvltime,self.dvlspeed[:,0],label='dvl')

        plt.subplot(312)
        plt.plot(self.imutime,self.imuaccel[:,1],label='imu')
        plt.plot(self.dvltime,self.dvlspeed[:,1],label='dvl')

        plt.subplot(313)
        plt.plot(self.imutime,self.imuaccel[:,2],label='imu')
        plt.plot(self.dvltime,self.dvlspeed[:,2],label='dvl')

        plt.legend()
        plt.show()
        plt.close()


    def plotspeed(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.dvltime,self.dvlspeed[:,0],label='dvl')
        plt.plot(self.imutime,self.imuspeed[:,0],label='imu')
        if self.simu:
            plt.plot(self.imutime,self.refspeed_imutime[:,0],label='ref')
         
        plt.subplot(312)
        plt.plot(self.dvltime,self.dvlspeed[:,1],label='dvl')
        plt.plot(self.imutime,self.imuspeed[:,1],label='imu')
        if self.simu:
            plt.plot(self.imutime,self.refspeed_imutime[:,1],label='ref')

        plt.subplot(313)
        plt.plot(self.dvltime,self.dvlspeed[:,2],label='dvl')
        plt.plot(self.imutime,self.imuspeed[:,2],label='imu')
        if self.simu:
            plt.plot(self.imutime,self.refspeed_imutime[:,2],label='ref')

        plt.legend()
        plt.show()
        plt.close()

    
    def plotpos(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.dvltime,self.dvlpos[:,0],label='dvl')
        plt.plot(self.imutime,self.imupos[:,0],label='imu')
        # plt.plot(self.dvltime,self.refpos_dvltime[:,0],label=self.reflab)

            

        plt.subplot(312)
        plt.plot(self.dvltime,self.dvlpos[:,1],label='dvl')
        plt.plot(self.imutime,self.imupos[:,1],label='imu')
        # plt.plot(self.dvltime,self.refpos_dvltime[:,1],label=self.reflab)

        plt.subplot(313)
        plt.plot(self.dvltime,self.dvlpos[:,2],label='dvl')
        plt.plot(self.imutime,self.imupos[:,2],label='imu')
        # plt.plot(self.dvltime,self.refpos_dvltime[:,2],label=self.reflab)


        plt.legend()
        plt.show()
        plt.close()
    
    def plot3d(self):



        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True

        fig = plt.figure(4)
        ax = fig.add_subplot(projection='3d')

        ax.plot(self.imupos[:,0],self.imupos[:,1],self.imupos[:,2],label='imu')
        ax.plot(self.dvlpos[:,0],self.dvlpos[:,1],self.dvlpos[:,2],label='dvl')
        ax.plot(self.refpos_dvltime[:,0],self.refpos_dvltime[:,1],self.refpos_dvltime[:,2],label=self.reflab)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.legend()
        plt.show()
        plt.close()
    
    def plot_diff(self,vec1,vec2):

        
        l_diff = []
        
        for i in range(0,len(self.imutime)):
                    l_diff.append(vec1[i]-vec2[i])
                   

        l_diff = np.array(l_diff)
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.imutime,l_diff[:,0])
        plt.subplot(312)
        plt.plot(self.imutime,l_diff[:,1])
        plt.subplot(313)
        plt.plot(self.imutime,l_diff[:,2])
        plt.show()    
        


    def plot_bias(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.biastime,self.true_bias[:,0],label='true')
        plt.plot(self.imutime,self.estimated_bias[:,0],label='estimated')
        plt.xlabel('t (s)')            
        plt.ylabel('biais x (m/s-2)')

        plt.subplot(312)
        plt.plot(self.biastime,self.true_bias[:,1],label='true')
        plt.plot(self.imutime,self.estimated_bias[:,1],label='estimated')
        plt.xlabel('t (s)')
        plt.ylabel('biais y (m/s-2)')

        plt.subplot(313)
        plt.plot(self.biastime,self.true_bias[:,2],label='true')
        plt.plot(self.imutime,self.estimated_bias[:,2],label='estimated')
        plt.xlabel('t (s)')        
        plt.ylabel('biais z (m/s-2)')


        plt.legend()
        plt.show()
        plt.close()        

pl = plotter(simu=False,bias_stdy=False,path="SINS_CPP_TEST/SIM_0")
# pl.plot_bias()
# pl.plotaccel()
pl.plotspeed()
pl.plotpos()
# pl.plot3d()
# pl.plot_diff(pl.refspeed_imutime,pl.imuspeed)
# pl.plot_diff(pl.refpos_imutime,pl.imupos)
