/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PinholeCameraRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 8, 2015
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes
 */
public interface PinholeCameraRobustEstimatorListener {
    /**
     * Called when estimation starts
     * @param estimator reference to robust estimator
     */
    public void onEstimateStart(
            PinholeCameraRobustEstimator estimator);
    
    /**
     * Called when estimation ends
     * @param estimator reference to robust estimator
     */
    public void onEstimateEnd(PinholeCameraRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution
     * @param estimator reference to robust estimator
     * @param iteration current iteration
     */
    public void onEstimateNextIteration(
            PinholeCameraRobustEstimator estimator, int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0
     */
    public void onEstimateProgressChange(
            PinholeCameraRobustEstimator estimator, float progress);        
}
