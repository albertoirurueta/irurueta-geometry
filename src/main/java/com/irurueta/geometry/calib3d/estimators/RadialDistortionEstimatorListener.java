/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.RadialDistortionEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 16, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified when estimation starts, finishes or any progress 
 * changes
 */
public interface RadialDistortionEstimatorListener {
    
    /**
     * Called when an estimator starts the radial distortion estimation process.
     * @param estimator reference to a radial distortion estimator
     */
    public void onEstimateStart(RadialDistortionEstimator estimator);
    
    /**
     * Called when an estimator ends the radial distortion estimation process
     * @param estimator reference to a radial distortion estimator
     */
    public void onEstimateEnd(RadialDistortionEstimator estimator);
    
    /**
     * Called to notify changes in radial distortion estimation progress.
     * @param estimator reference to a radial distortion estimator
     * @param progress current percentage of progress expressed as a value
     * between 0.0f and 1.0f
     */
    public void onEstimationProgressChange(RadialDistortionEstimator estimator,
            float progress);
}
