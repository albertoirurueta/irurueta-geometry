/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 3, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes
 */
public interface ImageOfAbsoluteConicRobustEstimatorListener {
    
    /**
     * Called when estimation starts
     * @param estimator reference to robust estimator
     */
    void onEstimateStart(ImageOfAbsoluteConicRobustEstimator estimator);
    
    /**
     * Called when estimation ends
     * @param estimator reference to robust estimator
     */
    void onEstimateEnd(ImageOfAbsoluteConicRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution
     * @param estimator reference to robust estimator
     * @param iteration current iteration
     */
    void onEstimateNextIteration(
            ImageOfAbsoluteConicRobustEstimator estimator, int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0
     */
    void onEstimateProgressChange(
            ImageOfAbsoluteConicRobustEstimator estimator, float progress);
}
