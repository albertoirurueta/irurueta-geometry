/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.EuclideanTransformation3DEstimatorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 22, 2017.
 */
package com.irurueta.geometry.estimators;

/**
 * Listener to be notified of events such as when estimation starts or ends.
 */
public interface EuclideanTransformation3DEstimatorListener {
    
    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(EuclideanTransformation3DEstimator estimator);
    
    /**
     * Called when estimation ends.
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(EuclideanTransformation3DEstimator estimator);
}
