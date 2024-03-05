import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt 


class plotter:
    def __init__(self,simu_name):

        print("loading results ...")
        # df_dvl = pd.read_csv("/home/maxwmr/Documents/work//data2/SINS_CPP_TEST/SIM_0/DVL.csv",sep=',')
        df_dvl2 = pd.read_csv("/home/maxwmr/Documents/work/simu_analysis_cpp/results/DVL.csv",sep=',')

        # df_imu = pd.read_csv("/home/maxwmr/Documents/work/data2/SINS_CPP_TEST/SIM_0/IMU.csv",sep=',')
        df_imu2 = pd.read_csv("/home/maxwmr/Documents/work/simu_analysis_cpp/results/IMU.csv",sep=',')
    
        # self.imuaccel =  np.stack([df_imu['acc1'].to_numpy(),df_imu['acc2'].to_numpy(),df_imu['acc3'].to_numpy()]).T
        self.imuaccel2 =  np.stack([df_imu2['acc1'].to_numpy(),df_imu2['acc2'].to_numpy(),df_imu2['acc3'].to_numpy()]).T

        # self.dvlspeed =  np.stack([df_dvl['speedX'].to_numpy(),df_dvl['speedY'].to_numpy(),df_dvl['speedZ'].to_numpy()]).T
        self.dvlspeed2 =  np.stack([df_dvl2['speedX'].to_numpy(),df_dvl2['speedY'].to_numpy(),df_dvl2['speedZ'].to_numpy()]).T

        # self.refpos = np.stack([df_ref['posX_ref'].to_numpy(),df_ref['posY_ref'].to_numpy(),df_ref['posZ_ref'].to_numpy()]).T

        self.dvltime = df_dvl2['time'].to_numpy()
        self.imutime = df_imu2['time'].to_numpy()


        print("done! ")

    

    def plotaccel(self):
        plt.figure(1)
        plt.subplot(311)
        # plt.plot(self.imutime,self.imuaccel[:,0],label='imu')
        plt.plot(self.imutime,self.imuaccel2[:,0],label='imu2')

        plt.subplot(312)
        # plt.plot(self.imutime,self.imuaccel[:,0],label='imu')
        plt.plot(self.imutime,self.imuaccel2[:,1],label='imu2')

        plt.subplot(313)
        # plt.plot(self.imutime,self.imuaccel[:,2],label='imu')
        plt.plot(self.imutime,self.imuaccel2[:,2],label='imu2')
        plt.legend()
        plt.show()
        plt.close()


    def plotspeed(self):
        plt.figure(1)
        plt.subplot(311)
        # plt.plot(self.dvltime,self.dvlspeed[:,0],label='dvl')
        plt.plot(self.dvltime,self.dvlspeed2[:,0],label='dvl2')
        # plt.plot(self.imutime,self.imuspeed[:,0],label='imu')

        plt.subplot(312)
        # plt.plot(self.dvltime,self.dvlspeed[:,1],label='dvl')
        plt.plot(self.dvltime,self.dvlspeed2[:,1],label='dvl2')
        # plt.plot(self.imutime,self.imuspeed[:,1],label='imu')


        plt.subplot(313)
        # plt.plot(self.dvltime,self.dvlspeed[:,2],label='dvl')
        plt.plot(self.dvltime,self.dvlspeed2[:,2],label='dvl2')
        # plt.plot(self.imutime,self.imuspeed[:,2],label='imu')


        plt.legend()
        plt.show()
        plt.close()

    
    def plotpos(self):
        plt.figure(1)
        plt.subplot(311)
        # plt.plot(self.dvltime,self.dvlpos[:,0],label='dvl')
        # plt.plot(self.imutime,self.imupos[:,0],label='imu')
        plt.plot(self.imutime,self.refpos_imutime[:,0],label='ref')

        plt.subplot(312)
        # plt.plot(self.dvltime,self.dvlpos[:,1],label='dvl')
        # plt.plot(self.imutime,self.imupos[:,1],label='imu')
        plt.plot(self.imutime,self.refpos_imutime[:,1],label='ref')

        plt.subplot(313)
        # plt.plot(self.dvltime,self.dvlpos[:,2],label='dvl')
        # plt.plot(self.imutime,self.imupos[:,2],label='imu')
        plt.plot(self.imutime,self.refpos_imutime[:,2],label='ref')


        plt.legend()
        plt.show()
        plt.close()
    



pl = plotter('SINS_TEST/SIMU_0')

pl.plotaccel()
pl.plotspeed()
# pl.plotpos()
# pl.plot3d()
