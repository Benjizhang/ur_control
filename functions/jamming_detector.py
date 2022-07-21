# jamming detector
# Metrics:
#   M1: by difference between KF and Mean values
#   M2: by rising rate of M1
# 
# Z Zhang
# 2022/07

import numpy as np
import pylab
import time

class JDLib():
    def __init__(self,df_ls,ds_ls,fdhat,fdmean,diff_bar):
        self.df_ls = df_ls
        self.ds_ls = ds_ls
        self.fdhat = fdhat
        self.fdmean = fdmean
        # self.jdid = JDid
        self.diff_bar = diff_bar

    def JD(self,JDid):
        if (JDid % 1 != 0) or (JDid <= 0):
            raise Exception('Error: Invalid JD Id')
        if JDid == 1:
            return self.JD1(JDid)
        else:
            raise Exception('Error: No Such JD')
    
    ## Jamming Detector 1
    def JD1(self,JDid):
        self.JDid = JDid
        ## Metric: diff > diff_bar
        cur_kf = self.fdhat[-1]
        cur_mean = self.fdmean[-1]
        diff = abs(cur_kf - cur_mean)
        if round(diff,6)>self.diff_bar:
            return True
        else:
            return False
    
    ## plot the jamming detector results
    def plotJDRes(self,ds_obj,title_str,fig_path,expid):
        ds_adv = round(ds_obj-self.ds_ls[-1], 3) # >0 in theory
        ## plot results
        pylab.figure(figsize=(10,5))  
        pylab.title(title_str)
        pylab.plot(self.ds_ls,self.df_ls,'k+')     #观测值  
        pylab.plot(self.ds_ls,self.df_ls,'r-',label='noisy measurements')  #观测值 
        pylab.plot(self.ds_ls[:len(self.fdhat)],self.fdhat,'b-',label='a posteri estimate')  #滤波估计值  
        pylab.plot(self.ds_ls[:len(self.fdhat)],self.fdmean, color='y',label='mean')         #平均值 
        if len(self.ds_ls) == len(self.fdhat):
            pylab.axvline(self.ds_ls[-1],color='r',linestyle='--', label='jamming')
        pylab.axvline(ds_obj,color='g',linestyle='--', label='object')
        pylab.legend()  
        pylab.xlabel('Distance (m)')  
        pylab.ylabel('Force (N)')
        pylab.setp(pylab.gca(),'ylim',[0,10.]) 
        pylab.setp(pylab.gca(),'xlim',[0,.3])
        pylab.grid()
        # pylab.show()
        now_date = time.strftime("%m%d%H%M%S", time.localtime())
        pylab.savefig('{}/{}_Exp{}Ite{}JD{}.png'.format(fig_path,now_date,expid,len(self.df_ls),self.JDid))

#region
# def JD1(self,kf_ls, mean_ls, diff_bar, delta_ite):

#         ## M1: diff > diff_bar
#         cur_kf = kf_ls[-1]
#         cur_mean = mean_ls[-1]
#         diff = abs(cur_kf - cur_mean)
#         if round(diff,6)>diff_bar:
#             return 'M1', True

#         ## M2: privious kf and mean values
#         # sample_num = 3
#         # rising_bar = 0.005
#         # if len(kf_ls) >= delta_ite*sample_num:
#         #     sample_kf_ls   = kf_ls[-1:-delta_ite*sample_num:-delta_ite]
#         #     sample_mean_ls = mean_ls[-1:-delta_ite*sample_num:-delta_ite]
#         #     # Note: sample_mean_ls[0] is current value, sample_mean_ls[2] is previous value
#         #     diff_ls = [abs(sample_kf_ls[k] - sample_mean_ls[k]) for k in range(sample_num)]
#         #     rising_ls = [diff_ls[:-1][j] - diff_ls[1:][j] for j in range(sample_num-1)]
#         #     diff_max_rising = max(rising_ls)/delta_ite
#         #     diff_sign_ls = [np.sign(diff_ls[:-1][j] - diff_ls[1:][j]) for j in range(sample_num-1)]
#         #     ## all slope is positive (+) && rising rate > rate_bar
#         #     # if all(i > 0 for i in diff_sign_ls) > 0:
#         #     if all(i > 0 for i in diff_sign_ls) > 0 and diff_max_rising > rising_bar:
#         #         return 'M2', True        

#         return 'M-1', False

# endregion

# def plotJDRes(title_str,df_ls,ds_ls,ds_obj,fdhat,fdmean,fig_path,expid,JDid):
#     ds_adv = round(ds_obj-ds_ls[-1], 3) # >0 in theory
#     ## plot results
#     pylab.figure(figsize=(10,5))  
#     pylab.title(title_str)
#     pylab.plot(ds_ls,df_ls,'k+')     #观测值  
#     pylab.plot(ds_ls,df_ls,'r-',label='noisy measurements')  #观测值 
#     pylab.plot(ds_ls,fdhat,'b-',label='a posteri estimate')  #滤波估计值  
#     pylab.plot(ds_ls,fdmean, color='y',label='mean')         #平均值 
#     pylab.axvline(ds_ls[-1],color='r',linestyle='--', label='jamming')
#     pylab.axvline(ds_obj,color='g',linestyle='--', label='object')
#     pylab.legend()  
#     pylab.xlabel('Distance (m)')  
#     pylab.ylabel('Force (N)')
#     pylab.setp(pylab.gca(),'ylim',[0,10.]) 
#     pylab.setp(pylab.gca(),'xlim',[0,.3])
#     pylab.grid()
#     # pylab.show()
#     pylab.savefig('{}/exp{}_ite{}_JD{}.png'.format(fig_path,expid,len(df_ls),JDid))

