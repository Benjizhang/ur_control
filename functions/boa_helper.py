# some supporting func for BOA

import numpy as np
import matplotlib.pyplot as plt

from matplotlib import cm

### plot_2d (v2)
def plot_2d2(slide_id, bo, util, kernel,x,y,XY, f_max, fig_path, name=None):

    mu, s, ut = posterior(bo, util, XY)
    fig, ax = plt.subplots(2, 2, figsize=(9, 8))
    gridsize=88
    str_kernel = str(kernel)
    
    fig.suptitle('{}-th slide, {} {}'.format(slide_id,str_kernel,kernel.length_scale_bounds), fontdict={'size':30})
    
    # GP regression output
    ax[0][0].set_title('GP Predicted Mean', fontdict={'size':15})
    im00 = ax[0][0].hexbin(x, y, C=mu, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=0, vmax=f_max)
    ax[0][0].axis('scaled')
    ax[0][0].axis([x.min(), x.max(), y.min(), y.max()])

    ax[0][1].set_title('Target Function', fontdict={'size':15})
    im01 = ax[0][1].hexbin(x, y, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=0, vmax=1)
    ax[0][1].axis('scaled')
    ax[0][1].axis([x.min(), x.max(), y.min(), y.max()])

    ax[1][0].set_title('GP Variance', fontdict={'size':15})
    im10 = ax[1][0].hexbin(x, y, C=s, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=0, vmax=1)
    ax[1][0].axis('scaled')
    ax[1][0].axis([x.min(), x.max(), y.min(), y.max()])    
    # plt.pause(0.1)

    ax[1][1].set_title('Acquisition Function', fontdict={'size':15})    
    im11 = ax[1][1].hexbin(x, y, C=ut, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=min(ut), vmax=max(ut))
    ax[1][1].axis('scaled')
    ax[1][1].axis([x.min(), x.max(), y.min(), y.max()])
    # plt.pause(0.1)

    # region
    # print(ut)
    # maxVal_x = np.where(ut.reshape((175, 125)) == ut.max())[0]
    # maxVal_y = np.where(ut.reshape((175, 125)) == ut.max())[1]
    # print(np.where(ut.reshape((175, 125)) == ut.max()))
    # print(maxVal_x)
    # print(maxVal_y)

    # ax[1][1].plot([np.where(ut.reshape((175, 125)) == ut.max())[1]*0.25/125.,
    #                np.where(ut.reshape((175, 125)) == ut.max())[1]*0.25/125.],
    #               [0, 0.35],
    #               '-', lw=2, color='k')
    # plt.show()

    # ax[1][1].plot([0, 0.25],
    #               [np.where(ut.reshape((175, 125)) == ut.max())[0]*0.35/175.,
    #                np.where(ut.reshape((175, 125)) == ut.max())[0]*0.35/175.],
    #               '-', lw=2, color='k')
    # plt.show()
    # endregion
    

    for im, axis in zip([im00, im01, im10, im11], ax.flatten()):
        cb = fig.colorbar(im, ax=axis)
        # cb.set_label('Value')

    if name is None:
        name = '_'

    # plt.tight_layout()

    ## Save or show figure?
    fig.savefig(fig_path + name + '.png')
    # plt.ioff()
    # plt.show()
    # plt.pause(2)
    # plt.close(fig)

### posterior
def posterior(bo, util, X):
    ur = unique_rows(bo._space.params)
    bo._gp.fit(bo._space.params[ur], bo._space.target[ur])
    mu, sigma2 = bo._gp.predict(X, return_std=True)
    ac = util.utility(X, bo._gp, bo._space.target.max())

    return mu, np.sqrt(sigma2), ac

### unique_rows
def unique_rows(a):
    """
    A functions to trim repeated rows that may appear when optimizing.
    This is necessary to avoid the sklearn GP object from breaking

    :param a: array to trim repeated rows from

    :return: mask of unique rows
    """

    # Sort array and kep track of where things should go back to
    order = np.lexsort(a.T)
    reorder = np.argsort(order)

    a = a[order]
    diff = np.diff(a, axis=0)
    ui = np.ones(len(a), 'bool')
    ui[1:] = (diff != 0).any(axis=1)

    return ui[reorder]