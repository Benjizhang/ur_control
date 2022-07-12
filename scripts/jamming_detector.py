# jamming detector
# Metrics:
#   M1: by difference between KF and Mean values
#   M2: by rising rate of M1
# 
# Z Zhang
# 2022/07

import numpy as np

def jamming_detector1(kf_ls, mean_ls, diff_bar, delta_ite):

    ## M1: diff > diff_bar
    cur_kf = kf_ls[-1]
    cur_mean = mean_ls[-1]
    diff = abs(cur_kf - cur_mean)
    if round(diff,6)>diff_bar:
        return 'M1', True

    ## M2: privious kf and mean values
    # sample_num = 3
    # rising_bar = 0.005
    # if len(kf_ls) >= delta_ite*sample_num:
    #     sample_kf_ls   = kf_ls[-1:-delta_ite*sample_num:-delta_ite]
    #     sample_mean_ls = mean_ls[-1:-delta_ite*sample_num:-delta_ite]
    #     # Note: sample_mean_ls[0] is current value, sample_mean_ls[2] is previous value
    #     diff_ls = [abs(sample_kf_ls[k] - sample_mean_ls[k]) for k in range(sample_num)]
    #     rising_ls = [diff_ls[:-1][j] - diff_ls[1:][j] for j in range(sample_num-1)]
    #     diff_max_rising = max(rising_ls)/delta_ite
    #     diff_sign_ls = [np.sign(diff_ls[:-1][j] - diff_ls[1:][j]) for j in range(sample_num-1)]

    #     ## all slope is positive (+) && rising rate > rate_bar
    #     # if all(i > 0 for i in diff_sign_ls) > 0:
    #     if all(i > 0 for i in diff_sign_ls) > 0 and diff_max_rising > rising_bar:
    #         return 'M2', True

    
    return 'M-1', False

