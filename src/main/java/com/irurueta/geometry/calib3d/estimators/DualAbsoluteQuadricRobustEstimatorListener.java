/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3D.estimators.DualAbsoluteQuadricRobustEstimatorListener
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes.
 */
public interface DualAbsoluteQuadricRobustEstimatorListener {
    
    /**
     * Called when estimation starts.
     * @param estimator reference to robust estimator.
     */
    void onEstimateStart(DualAbsoluteQuadricRobustEstimator estimator);
    
    /**
     * Called when estimation ends.
     * @param estimator reference to robust estimator.
     */
    void onEstimateEnd(DualAbsoluteQuadricRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution.
     * @param estimator reference to robust estimator.
     * @param iteration current iteration.
     */
    void onEstimateNextIteration(DualAbsoluteQuadricRobustEstimator estimator, 
            int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator.
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0.
     */
    void onEstimateProgressChange(DualAbsoluteQuadricRobustEstimator estimator, 
            float progress);
}
