/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 1, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified when estimation starts, finishes or any progress
 * changes
 */
public interface ImageOfAbsoluteConicEstimatorListener {
    /**
     * Called when an estimator starts the IAC estimation process.
     * @param estimator reference to an IAC estimator
     */
    void onEstimateStart(ImageOfAbsoluteConicEstimator estimator);
    
    /**
     * Called when an estimator ends the IAC estimation process
     * @param estimator reference to an IAC estimator
     */
    void onEstimateEnd(ImageOfAbsoluteConicEstimator estimator);
    
    /**
     * Called to notify changes in IAC estimation progress.
     * @param estimator reference to an IAC estimator
     * @param progress current percentage of progress expressed as a value
     * between 0.0f and 1.0f
     */
    void onEstimationProgressChange(ImageOfAbsoluteConicEstimator estimator, 
            float progress);
}
