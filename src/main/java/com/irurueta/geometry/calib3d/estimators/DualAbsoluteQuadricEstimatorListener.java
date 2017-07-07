/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.DualAbsoluteQuadricEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 22, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified when estimation starts, finishes or any progress 
 * changes.
 */
public interface DualAbsoluteQuadricEstimatorListener {
    
    /**
     * Called when an estimator starts the Dual Absolute Quadric estimation 
     * process.
     * @param estimator reference to a DAQ estimator.
     */
    void onEstimateStart(DualAbsoluteQuadricEstimator estimator);
    
    /**
     * Called when an estimator ends the Dual Absolute Quadric estimation 
     * process.
     * @param estimator reference to a DAQ estimator.
     */
    void onEstimateEnd(DualAbsoluteQuadricEstimator estimator);
    
    /**
     * Called to notify changes in DAQ estimation progress.
     * @param estimator reference to a DAQ estimator.
     * @param progress current percentage of progress expressed as a value
     * between 0.0f and 1.0f.
     */
    void onEstimationProgressChange(DualAbsoluteQuadricEstimator estimator,
            float progress);
}
