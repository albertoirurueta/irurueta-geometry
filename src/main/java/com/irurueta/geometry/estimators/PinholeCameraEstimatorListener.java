/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PinholeCameraEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2013
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified when estimation starts, finishes or any progress
 * changes
 */
public interface PinholeCameraEstimatorListener {
    
    /**
     * Called when an estimator starts the camera estimation process.
     * @param estimator Reference to pinhole camera estimator
     */
    public void onEstimateStart(PinholeCameraEstimator estimator);
    
    /**
     * Called when an estimator ends the camera estimation process.
     * @param estimator Reference to pinhole camera estimator
     */
    public void onEstimateEnd(PinholeCameraEstimator estimator);
    
    /**
     * Called to notify changes in camera estimation progress.
     * @param estimator Reference to pinhole camera estimator
     * @param progress Current percentage of progress expressed as a value 
     * between 0.0f and 1.0f
     */
    public void onEstimationProgressChange(PinholeCameraEstimator estimator, 
            float progress);
}
