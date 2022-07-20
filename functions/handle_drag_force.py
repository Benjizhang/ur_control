import numpy

def smooth_fd_kf(fd_read_ls):
    ## parameters for KF
    n_iter = len(fd_read_ls)
    sz = (n_iter,) 
    z = fd_read_ls
    Q = 1e-5   # process variance (state func)
    R = 0.1**2 # estimate of measurement variance (measure func)
    # R = 0.1**1
    
    ## assign memory 
    xhat=numpy.zeros(sz)      # x 滤波估计值  
    P=numpy.zeros(sz)         # 滤波估计协方差矩阵  
    xhatminus=numpy.zeros(sz) #  x 估计值  
    Pminus=numpy.zeros(sz)    # 估计协方差矩阵  
    K=numpy.zeros(sz)         # 卡尔曼增益

    ## intial guesses  
    xhat[0] = 0.0  
    P[0] = 1.0 

    ## KF iterations 
    for k in range(1,n_iter): 
        # predict  
        xhatminus[k] = xhat[k-1]  #X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0  
        Pminus[k] = P[k-1]+Q      #P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1  
        
        # update  
        K[k] = Pminus[k]/( Pminus[k]+R ) #Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1  
        xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k]) #X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1  
        P[k] = (1-K[k])*Pminus[k] #P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1  

    return xhat, Pminus


def get_mean(fd_read_ls):
    n_iter = len(fd_read_ls)
    sz = (n_iter,) 
    xmean=numpy.zeros(sz) 
    for k in range(n_iter):
        xmean[k] = numpy.mean(fd_read_ls[:k+1])

    return xmean