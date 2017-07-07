/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.Line2DRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 1, 2015
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes
 */
public interface Line2DRobustEstimatorListener {
   /**
     * Called when estimation starts
     * @param estimator reference to robust estimator
     */
    public void onEstimateStart(Line2DRobustEstimator estimator);
    
    /**
     * Called when estimation ends
     * @param estimator reference to robust estimator
     */
    public void onEstimateEnd(Line2DRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution
     * @param estimator reference to robust estimator
     * @param iteration current iteration
     */
    public void onEstimateNextIteration(Line2DRobustEstimator estimator,
            int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0
     */
    public void onEstimateProgressChange(
            Line2DRobustEstimator estimator, float progress);        
}
