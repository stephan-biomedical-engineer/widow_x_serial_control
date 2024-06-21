# -*- coding: utf-8 -*-
'''
#-------------------------------------------------------------------------------
# NATIONAL UNIVERSITY OF SINGAPORE - NUS
# SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
# Singapore
# URL: http://www.sinapseinstitute.org
#-------------------------------------------------------------------------------
# Neuromorphic Engineering Group
# Author: Andrei Nakagawa-Silva, MSc
# Contact: nakagawa.andrei@gmail.com
#-------------------------------------------------------------------------------
'''
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#LIBRARIES
import numpy as np
import scipy.signal as sig
from collections import deque
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
def convscale(x,xmin,xmax,ymin,ymax):
    '''
    converts values from a specific range to another
    '''
    vals = ((x-xmin)*(ymax-ymin) / (xmax-xmin)) + ymin;
    vals = np.array(vals)
    vals[np.where(vals < ymin)] = ymin
    vals[np.where(vals > ymax)] = ymax
    return vals
#-------------------------------------------------------------------------------
def funcsat(x,xmin,xmax):
    '''
    linear piece-wise saturation function
    '''
    x[np.where(x < xmin)] = xmin
    x[np.where(x > xmax)] = xmax
    return x
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#detect peaks
def detect_peaks(x, mph=None, mpd=1, threshold=0, edge='rising',
         kpsh=False, valley=False, show=False, ax=None):

    x = np.atleast_1d(x).astype('float64')
    if x.size < 3:
        return np.array([], dtype=int)
    if valley:
        x = -x
    # find indices of all peaks
    dx = x[1:] - x[:-1]
    # handle NaN's
    indnan = np.where(np.isnan(x))[0]
    if indnan.size:
        x[indnan] = np.inf
        dx[np.where(np.isnan(dx))[0]] = np.inf
    ine, ire, ife = np.array([[], [], []], dtype=int)
    if not edge:
        ine = np.where((np.hstack((dx, 0)) < 0) & (np.hstack((0, dx)) > 0))[0]
    else:
        if edge.lower() in ['rising', 'both']:
            ire = np.where((np.hstack((dx, 0)) <= 0) & (np.hstack((0, dx)) > 0))[0]
        if edge.lower() in ['falling', 'both']:
            ife = np.where((np.hstack((dx, 0)) < 0) & (np.hstack((0, dx)) >= 0))[0]
    ind = np.unique(np.hstack((ine, ire, ife)))
    # handle NaN's
    if ind.size and indnan.size:
        # NaN's and values close to NaN's cannot be peaks
        ind = ind[np.in1d(ind, np.unique(np.hstack((indnan, indnan-1, indnan+1))), invert=True)]
    # first and last values of x cannot be peaks
    if ind.size and ind[0] == 0:
        ind = ind[1:]
    if ind.size and ind[-1] == x.size-1:
        ind = ind[:-1]
    # remove peaks < minimum peak height
    if ind.size and mph is not None:
        ind = ind[x[ind] >= mph]
    # remove peaks - neighbors < threshold
    if ind.size and threshold > 0:
        dx = np.min(np.vstack([x[ind]-x[ind-1], x[ind]-x[ind+1]]), axis=0)
        ind = np.delete(ind, np.where(dx < threshold)[0])
    # detect small peaks closer than minimum peak distance
    if ind.size and mpd > 1:
        ind = ind[np.argsort(x[ind])][::-1]  # sort ind by peak height
        idel = np.zeros(ind.size, dtype=bool)
        for i in range(ind.size):
            if not idel[i]:
                # keep peaks with the same height if kpsh is True
                idel = idel | (ind >= ind[i] - mpd) & (ind <= ind[i] + mpd) \
                    & (x[ind[i]] > x[ind] if kpsh else True)
                idel[i] = 0  # Keep current peak
        # remove the small peaks and sort back the indices by their occurrence
        ind = np.sort(ind[~idel])

    if show:
        if indnan.size:
            x[indnan] = np.nan
        if valley:
            x = -x
        _plot(x, mph, mpd, threshold, edge, valley, ax, ind)

    return ind
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#this class defines a moving average algorithm with 99% overlap
class MovingAverage():
    def __init__(self,_windowSize=10,_sampfreq=100):
        self.window = deque() #processing window
        self.windowsum = 0 #keeps track of the sum of all elements in the window
        #counts the number of samples to decide how to calculate the moving
        #average
        self.samplecounter = 0
        #sampling rate
        self.sampfreq = _sampfreq
        #sampling period
        self.dt = 1.0 / self.sampfreq
        #size of the window
        self.windowSize = _windowSize
    def getSample(self,_newsample):
        #if the sample counter is greater than the size of the window, then
        #the window should move in order to output the new data
        if self.samplecounter >= self.windowSize:
            #subtracts from the first element of the window
            self.windowsum -= self.window.popleft()
            #adds the new sample to the end of the window
            self.windowsum += _newsample
            #appends the new sample to the window
            self.window.append(_newsample)
            #returns the moving average response
            return self.windowsum / self.windowSize
        else: #otherwise, just populate the window
            self.windowsum += _newsample
            self.window.append(_newsample) #appends the new sample
            self.samplecounter += 1 #increase the sample counter
            return self.windowsum / self.samplecounter
#-------------------------------------------------------------------------------
class LowPass():
    def __init__(self,order,cutoff,sampfreq):
        self.order = order
        self.cutoff = cutoff
        self.sampfreq = sampfreq
        self.wn = cutoff / (sampfreq / 2)
        if self.wn < 0 or self.wn > 1:
            return False
        self.b,self.a = sig.butter(self.order,self.wn,'low')
#-------------------------------------------------------------------------------
#testing the library
if __name__ == '__main__':
    #plotting with matplotlib
    import matplotlib.pyplot as plt
    #signal parameters
    sampfreq = 100 #sampling rate
    dt = 1.0 / sampfreq # sampling period
    f = 2 #sine wave frequency
    tb = 0 #time begins
    te = 5 #time ends
    #time
    x = np.arange(tb,te,dt)
    #pure sine wave
    y = np.sin(2.0*np.pi*f*x)
    #gaussian noise
    w = np.random.uniform(0,0.8,y.shape)
    #add noise to the original signal
    y += w
    #testing the moving average
    #moving average class
    windowsize = 10 #size of the window
    mva = MovingAverage(windowsize,sampfreq)
    filts = [] #new list containing the filtered signal
    #for every sample of the sine wave, generate the moving average value
    [filts.append(mva.getSample(x)) for x in y]

    #plots
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(x,y) #sine wave with noise
    plt.ylabel('Amplitude')
    plt.subplot(2,1,2)
    plt.plot(x,filts) #filtered with moving average
    plt.ylabel('Amplitude')
    plt.xlabel('Time (s)')
    plt.show()
