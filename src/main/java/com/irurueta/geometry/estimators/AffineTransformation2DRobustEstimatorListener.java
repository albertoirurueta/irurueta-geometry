/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.AffineTransformation2DRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 8, 2015
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or 
 * when progress changes.
 */
public interface AffineTransformation2DRobustEstimatorListener {
    /**
     * Called when estimation starts.
     * @param estimator reference to robust estimator.
     */
    void onEstimateStart(AffineTransformation2DRobustEstimator estimator);
    
    /**
     * Called when estimation ends.
     * @param estimator reference to robust estimator.
     */
    void onEstimateEnd(AffineTransformation2DRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution.
     * @param estimator reference to robust estimator.
     * @param iteration current iteration.
     */
    void onEstimateNextIteration(
            AffineTransformation2DRobustEstimator estimator, 
            int iteration);
    
    /**
     * Called when estimation progress changes significantly..
     * @param estimator reference to robust estimator.
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0.
     */
    void onEstimateProgressChange(
            AffineTransformation2DRobustEstimator estimator, float progress);    
}
