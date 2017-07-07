/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.KruppaDualImageOfAbsoluteConicEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 18, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Listener to be notified when estimation starts, finishes or any progress
 * changes.
 */
public interface KruppaDualImageOfAbsoluteConicEstimatorListener {
    
    /**
     * Called when an estimator starts the DIAC estimation process.
     * @param estimator reference to a DIAC estimator.
     */
    void onEstimateStart(KruppaDualImageOfAbsoluteConicEstimator estimator);
    
    /**
     * Called when an estimator ends the DIAC estimation process.
     * @param estimator reference to a DIAC estimator.
     */
    void onEstimateEnd(KruppaDualImageOfAbsoluteConicEstimator estimator);    
}
