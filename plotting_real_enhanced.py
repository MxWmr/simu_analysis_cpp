import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt 
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

class plotter:
    def __init__(self,simu_name):

        print("loading results ...")
        df_dvl = pd.read_csv("/home/maxwmr/Documents/work/simu_analysis_cpp/results/DVL.csv",sep=',')
        df_imu = pd.read_csv("/home/maxwmr/Documents/work/simu_analysis_cpp/results/IMU.csv",sep=',')
        df_or = pd.read_csv("/home/maxwmr/Documents/work/data2/ESSCORAL19_02/Orientation.csv",sep=',')
        df_fog = pd.read_csv("/home/maxwmr/Documents/work/data2/ESSCORAL19_02/IMU.csv",sep=',')
        df_or['yaw'] = df_or['heading'].apply(lambda x: 90-x)

        df_dvl = pd.merge_asof(df_dvl,df_or,on=["time"],direction='nearest')
        df_imu = pd.merge_asof(df_imu,df_fog,on=["time"],suffixes=("","_ref"),direction='nearest')
        df_imu = pd.merge_asof(df_imu,df_or,on=["time"],direction='nearest')

        self.imuaccel =  np.stack([df_imu['acc1_ref'].to_numpy(),df_imu['acc2_ref'].to_numpy(),df_imu['acc3_ref'].to_numpy()]).T*50*1e-7

        self.fog =  np.stack([df_imu['fog1'].to_numpy(),df_imu['fog2'].to_numpy(),df_imu['fog3'].to_numpy()]).T*50*1e-7

        self.dvlspeed =  np.stack([df_dvl['speedX'].to_numpy(),df_dvl['speedY'].to_numpy(),df_dvl['speedZ'].to_numpy()]).T
        self.imuspeed =  np.stack([df_imu['speedX'].to_numpy(),df_imu['speedY'].to_numpy(),df_imu['speedZ'].to_numpy()]).T
        # self.refspeed_imutime = np.stack([df_imu['speedX_ref'].to_numpy(),df_imu['speedY_ref'].to_numpy(),df_imu['speedZ_ref'].to_numpy()]).T
        # self.refspeed_dvltime = np.stack([df_dvl['speedX_ref'].to_numpy(),df_dvl['speedY_ref'].to_numpy(),df_dvl['speedZ_ref'].to_numpy()]).T

        self.dvlpos =  np.stack([df_dvl['posX'].to_numpy(),df_dvl['posY'].to_numpy(),df_dvl['posZ'].to_numpy()]).T
        self.imupos =  np.stack([df_imu['posX'].to_numpy(),df_imu['posY'].to_numpy(),df_imu['posZ'].to_numpy()]).T
        # self.refpos_imutime = np.stack([df_imu['posX_ref'].to_numpy(),df_imu['posY_ref'].to_numpy(),df_imu['posZ_ref'].to_numpy()]).T
        self.refpos_dvltime = np.stack([df_dvl['posX_ref'].to_numpy(),df_dvl['posY_ref'].to_numpy(),df_dvl['posZ_ref'].to_numpy()]).T

        self.orientation =  np.stack([df_dvl['roll'].to_numpy(),df_dvl['pitch'].to_numpy(),df_dvl['yaw'].to_numpy()]).T
        self.orientation_imutime =  np.stack([df_imu['roll'].to_numpy(),df_imu['pitch'].to_numpy(),df_imu['yaw'].to_numpy()]).T


        self.dvltime = df_dvl['time'].to_numpy()
        self.imutime = df_imu['time'].to_numpy()

        for i in tqdm(range(len(self.imuaccel))):
            r = R.from_euler('xyz',self.orientation_imutime[i],degrees=True)
            R_n_b = r.as_matrix()
            self.imuaccel[i]=R_n_b@self.imuaccel[i]-np.array([0,0,9.81])
        print("done! ")

    

    def plotaccel(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.imutime[:100000],self.imuaccel[:100000,0],label='imu accel (m.s-2)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,0],label='dvl speed (m.s-1)')


        plt.subplot(312)
        plt.plot(self.imutime[:100000],self.imuaccel[:100000,1],label='imu accel (m.s-2)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,1],label='dvl speed (m.s-1)')


        plt.subplot(313)
        plt.plot(self.imutime[:100000],self.imuaccel[:100000,2],label='imu accel (m.s-2)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,2],label='dvl speed (m.s-1)')
        plt.xlabel('time (s)')
        plt.legend()
        plt.show()
        plt.close()


    def plot_or2(self,i):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.imutime[:100000],self.imuaccel[:100000,i],label='imu accel (m.s-2)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,i],label='dvl speed (m.s-1)')
        plt.xlabel('time (s)')
        plt.legend()

        plt.subplot(312)
        plt.plot(self.imutime[:100000],self.fog[:100000,i],'g',label='fog (°/s)')
        plt.xlabel('time (s)')
        plt.legend()

        plt.subplot(313)
        plt.plot(self.dvltime[:2000],self.orientation[:2000,i],'r',label='orientation (°)')

        plt.xlabel('time (s)')
        plt.legend()
        plt.show()
        plt.close()

    def plot_or(self):
        plt.figure(2)
        fig,host = plt.subplots(3,1)
        host0 = host[0]
        ax2 = host0.twinx()
        host0.set_ylabel("accel/speed")
        ax2.set_ylabel("orient (°)")
        host0.plot(self.imutime[:100000],self.imuaccel[:100000,0],label='imu accel (m.s-2)')
        ax2.plot(self.imutime[:100000],self.fog[:100000,0],'g',label='fog (°/s)')
        host0.plot(self.dvltime[:2000],self.dvlspeed[:2000,0],label='dvl speed (m.s-1)')
        ax2.plot(self.dvltime[:2000],self.orientation[:2000,0],'r',label='orientation (°)')

        host1 = host[1]
        ax3 = host1.twinx()
        host1.set_ylabel("accel/speed")
        ax3.set_ylabel("orient (°)")
        host1.plot(self.imutime[:100000],self.imuaccel[:100000,1],label='imu accel (m.s-2)')
        ax3.plot(self.imutime[:100000],self.fog[:100000,1],'g',label='fog (°/s)')
        host1.plot(self.dvltime[:2000],self.dvlspeed[:2000,1],label='dvl speed (m.s-1)')
        ax3.plot(self.dvltime[:2000],self.orientation[:2000,1],'r',label='orientation (°)')


        host2 = host[2]
        ax4 = host2.twinx()
        host2.set_ylabel("accel/speed")
        ax4.set_ylabel("orient/fog")
        host2.plot(self.imutime[:100000],self.imuaccel[:100000,2],label='imu accel (m.s-2)')
        ax4.plot(self.imutime[:100000],self.fog[:100000,2],'g',label='fog (°/s)')
        host2.plot(self.dvltime[:2000],self.dvlspeed[:2000,2],label='dvl speed (m.s-1)')
        ax4.plot(self.dvltime[:2000],self.orientation[:2000,2],'r',label='orientation (°)')


        plt.xlabel('time (s)')
        plt.legend()
        plt.show()
        plt.close()


    def plotspeed(self):
        plt.figure(2)
        plt.subplot(311)
        plt.plot(self.imutime[:100000],self.imuspeed[:100000,0],label='imu speed (m.s-1)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,0],label='dvl speed (m.s-1)')

        plt.subplot(312)
        plt.plot(self.imutime[:100000],self.imuspeed[:100000,1],label='imu speed (m.s-1)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,1],label='dvl speed (m.s-1)')


        plt.subplot(313)
        plt.plot(self.imutime[:100000],self.imuspeed[:100000,2],label='imu speed (m.s-1)')
        plt.plot(self.dvltime[:2000],self.dvlspeed[:2000,2],label='dvl speed (m.s-1)')
        plt.xlabel('time (s)')

        plt.legend()
        plt.show()
        plt.close()

    
    def plotpos(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.dvltime,self.dvlpos[:,0],label='dvl')
        plt.plot(self.imutime,self.imupos[:,0],label='imu')
        # plt.plot(self.imutime,self.refpos_imutime[:,0],label='ref')

        plt.subplot(312)
        plt.plot(self.dvltime,self.dvlpos[:,1],label='dvl')
        plt.plot(self.imutime,self.imupos[:,1],label='imu')
        # plt.plot(self.imutime,self.refpos_imutime[:,1],label='ref')

        plt.subplot(313)
        plt.plot(self.dvltime,self.dvlpos[:,2],label='dvl')
        plt.plot(self.imutime,self.imupos[:,2],label='imu')
        # plt.plot(self.imutime,self.refpos_imutime[:,2],label='ref')


        plt.legend()
        plt.show()
        plt.close()
    
    def plot3d(self):
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True

        fig = plt.figure(4)
        ax = fig.add_subplot(projection='3d')

        # ax.plot(self.imupos[:,0],self.imupos[:,1],self.imupos[:,2],label='imu')
        ax.plot(self.dvlpos[:,0],self.dvlpos[:,1],self.dvlpos[:,2],label='dvl')
        # ax.plot(self.refpos_dvltime[:,0],self.refpos_dvltime[:,1],self.refpos_dvltime[:,2],label='ref')

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
        


pl = plotter('SINS_TEST/SIMU_0')

# pl.plotaccel()
# pl.plotspeed()
# pl.plotpos()
pl.plot_or2(1)
# pl.plot3d()
# pl.plot_diff(pl.refspeed_imutime,pl.imuspeed)
# pl.plot_diff(pl.refpos_imutime,pl.imupos)
