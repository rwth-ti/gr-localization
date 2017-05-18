import numpy as np
from scipy import interpolate


def corr_no_interpolation(y1,y2, window_size):
    corr = np.absolute(np.correlate(y1[0,:], y2[0,:], "full"))
    #print len(y1[0,:])
    delay = np.argmax(corr)
    return corr, delay

    
def corr_spline_interpolation(y1,y2, window_size):
    y1 = np.array([y1])
    y2 = np.array([y2])
    corr, delay = corr_no_interpolation(y1,y2, window_size)
    if window_size%2!=0:
        window_interp = np.arange(delay-(window_size-1)/2,delay+(window_size-1)/2)
    else:
        window_interp = np.arange(delay-(window_size)/2,delay+(window_size)/2-1)
    tck = interpolate.splrep(window_interp, corr[window_interp], s=0, k=4)
    dspl = interpolate.splder(tck)
    delay_spl = interpolate.sproot(dspl, mest = 20)
    if len(delay_spl) > 1: 
        delay = delay_spl[np.argmax(interpolate.splev(delay_spl, tck, der=0))]
    else:
        delay = delay_spl[0]
    return corr, delay-len(y1[0,:])+1
