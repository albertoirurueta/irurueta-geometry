/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.InitialCamerasEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 5, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.PinholeCamera;

/**
 * Listener in charge of attending events for an InitialCamerasEstimator,
 * such as when estimation starts or finishes.
 */
public interface InitialCamerasEstimatorListener {
    
    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onStart(InitialCamerasEstimator estimator);
    
    /**
     * Called when estimation successfully finishes.
     * @param estimator estimator raising the event.
     * @param estimatedLeftCamera estimated camera on left view.
     * @param estimatedRightCamera estimated camera on right view.
     */
    void onFinish(InitialCamerasEstimator estimator, 
            PinholeCamera estimatedLeftCamera, 
            PinholeCamera estimatedRightCamera);
    
    /**
     * Called when estimation fails.
     * @param estimator estimator raising the event.
     * @param e reason of estimation failure.
     */
    void onFail(InitialCamerasEstimator estimator, 
            InitialCamerasEstimationFailedException e);
}
