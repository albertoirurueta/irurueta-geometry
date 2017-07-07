/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.Point2DRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes
 */
public interface Point2DRobustEstimatorListener {
    /**
     * Called when estimation starts
     * @param estimator reference to robust estimator
     */
    public void onEstimateStart(Point2DRobustEstimator estimator);
    
    /**
     * Called when estimation ends
     * @param estimator reference to robust estimator
     */
    public void onEstimateEnd(Point2DRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution
     * @param estimator reference to robust estimator
     * @param iteration current iteration
     */
    public void onEstimateNextIteration(Point2DRobustEstimator estimator,
            int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0
     */
    public void onEstimateProgressChange(Point2DRobustEstimator estimator,
            float progress);    
}
