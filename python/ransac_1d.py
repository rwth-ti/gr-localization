import numpy as np
import scipy # use np if scipy unavailable
import scipy.linalg # use np if scipy unavailable


## Copyright (c) 2004-2007, Andrew D. Straw. All rights reserved.

## Copyright (c) 2017, Johannes Schmitz, Felix Bartsch. All rights reserved.

## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:

##     * Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.

##     * Redistributions in binary form must reproduce the above
##       copyright notice, this list of conditions and the following
##       disclaimer in the documentation and/or other materials provided
##       with the distribution.

##     * Neither the name of the of the authors nor the names of its
##       contributors may be used to endorse or promote products derived
##       from this software without specific prior written permission.

## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

class ransac_1d:
    def __init__(self, model, n_rel, d_rel, k, t):
        self.model = model
        self.k = k
        self.t = t
        self.n_rel = n_rel
        self.d_rel = d_rel

    def ransac_fit(self,data):
        """fit model parameters to data using the RANSAC algorithm

    This implementation written from pseudocode found at
    http://en.wikipedia.org/w/index.php?title=RANSAC&oldid=116358182

    {{{
    Given:
        data - a set of observed data points
        model - a model that can be fitted to data points
        n - the minimum number of data values required to fit the model
        k - the maximum number of iterations allowed in the algorithm
        t - a threshold value for determining when a data point fits a model
        d - the number of close data values required to assert that a model fits well to data
    Return:
        bestfit - model parameters which best fit the data (or nil if no good model is found)
    iterations = 0
    bestfit = nil
    besterr = something really large
    while iterations < k {
        maybeinliers = n randomly selected values from data
        maybemodel = model parameters fitted to maybeinliers
        alsoinliers = empty set
        for every point in data not in maybeinliers {
            if point fits maybemodel with an error smaller than t
                 add point to alsoinliers
        }
        if the number of elements in alsoinliers is >= d {
            % this implies that we may have found a good model
            % now test how good it is
            bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
            thiserr = a measure of how well model fits these points
            if thiserr < besterr {
                bestfit = bettermodel
                besterr = thiserr
            }
        }
        increment iterations
    }
    return bestfit
    }}}
    """
        n = int(self.n_rel * len(data))
        d = int(self.d_rel * len(data))
        iterations = 0
        bestfit = None
        besterr = np.inf
        best_inlier_idxs = None
        while iterations < self.k:
            maybe_idxs, test_idxs = random_partition(n,data.shape[0])
            maybeinliers = data[maybe_idxs]
            maybemodel = self.model.fit(maybeinliers)
            test_points = data[test_idxs]
            test_err = self.model.get_error(test_points, maybemodel)
            also_idxs = test_idxs[test_err < self.t] # select indices of rows with accepted points
            alsoinliers = data[also_idxs]
            bettermodel = maybemodel
            if len(alsoinliers) >= d:
                betterdata = np.concatenate( (maybeinliers, alsoinliers) )
                bettermodel = self.model.fit(betterdata)
                better_errs = self.model.get_error( betterdata, bettermodel)
                thiserr = np.mean( better_errs )
                if thiserr < besterr:
                    bestfit = bettermodel
                    besterr = thiserr
                    best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )

            iterations+=1
        if bestfit is None:
            print("Did not meet fit acceptance criteria. Return mean")
            return data.mean()
        else:
            return bestfit

def random_partition(n,n_data):
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = np.arange( n_data )
    np.random.shuffle(all_idxs)
    idxs1 = all_idxs[:n]
    idxs2 = all_idxs[n:]
    return idxs1, idxs2

class ConstantLeastSquaresModel:
    """linear system solved using linear least squares

    This class serves as an example that fulfills the model interface
    needed by the ransac() function.
    
    """
    def fit(self, data):
        c = np.mean(data)
        return c
    def get_error( self, data, model):
        err = np.subtract(data,model)**2
        return err


