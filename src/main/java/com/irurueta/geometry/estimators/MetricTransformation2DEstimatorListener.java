/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MetricTransformation2DEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 23, 2017.
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts or ends.
 */
public interface MetricTransformation2DEstimatorListener {
    
    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(MetricTransformation2DEstimator estimator);
    
    /**
     * Called when estimation ends.
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(MetricTransformation2DEstimator estimator);
}
