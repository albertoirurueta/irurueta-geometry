/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.estimators.FundamentalMatrixRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 20, 2015
 */
package com.irurueta.geometry.epipolar.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes
 */
public interface FundamentalMatrixRobustEstimatorListener {
    
    /**
     * Called when estimation starts
     * @param estimator referente to robust estimator
     */
    public void onEstimateStart(FundamentalMatrixRobustEstimator estimator);
    
    /**
     * Called when estimation ends
     * @param estimator reference to robust estimator
     */
    public void onEstimateEnd(FundamentalMatrixRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution
     * @param estimator reference to robust estimator
     * @param iteration current iteration
     */
    public void onEstimateNextIteration(
            FundamentalMatrixRobustEstimator estimator, int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0
     */
    public void onEstimateProgressChange(
            FundamentalMatrixRobustEstimator estimator, float progress);
}
