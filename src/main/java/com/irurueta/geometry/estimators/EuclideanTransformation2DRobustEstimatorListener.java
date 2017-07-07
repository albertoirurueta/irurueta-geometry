/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.EuclideanTransformation2DRobustEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 24, 2017.
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts, ends or
 * when progress changes.
 */
public interface EuclideanTransformation2DRobustEstimatorListener {
    
    /**
     * Called when estimation starts.
     * @param estimator reference to robust estimator.
     */
    void onEstimateStart(EuclideanTransformation2DRobustEstimator estimator);
    
    /**
     * Called when estimation ends.
     * @param estimator reference to robust estimator.
     */
    void onEstimateEnd(EuclideanTransformation2DRobustEstimator estimator);
    
    /**
     * Called when estimator iterates to refine a possible solution.
     * @param estimator reference to robust estimator.
     * @param iteration current iteration.
     */
    void onEstimateNextIteration(
            EuclideanTransformation2DRobustEstimator estimator, int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param estimator reference to robust estimator.
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0.
     */
    void onEstimateProgressChange(
            EuclideanTransformation2DRobustEstimator estimator, float progress);
}
