/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.SingleHomographyPinholeCameraEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 23, 2017.
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified when estimation starts or finishes.
 */
public interface SingleHomographyPinholeCameraEstimatorListener {
    /**
     * Called when an estimator starts.
     * @param estimator reference to an estimator.
     */
    void onEstimateStart(SingleHomographyPinholeCameraEstimator estimator);
    
    /**
     * Called when an estimator ends.
     * @param estimator reference to an estimator.
     */
    void onEstimateEnd(SingleHomographyPinholeCameraEstimator estimator);
}
