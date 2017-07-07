/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 18, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.AbsoluteOrientationConstantVelocityModelSlamEstimator;

/**
 * Estimates cameras and 3D reconstructed points from sparse image point
 * correspondences in two views and using SLAM (with accelerometer and gyroscope
 * data) with absolute orientation for overall scale and orientation estimation.
 * NOTE: This implementation does not seem to be very reliable because the SLAM
 * estimator is not accurate at all on position or orientation estimator.
 * Use AbsoluteOrientationSlamTwoViewsSparseReconstructor if absolute 
 * orientation is needed, or SlamTwoViewsSparseReconstructor otherwise to get
 * better accuracy.
 */
public class AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor 
        extends BaseAbsoluteOrientationSlamTwoViewsSparseReconstructor<
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration,
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor,
        AbsoluteOrientationConstantVelocityModelSlamEstimator> {
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor(
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration,
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor(
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        this(new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration(), 
                listener);
    }
    
    /**
     * Starts reconstruction.
     * If reconstruction has already started and is running, calling this 
     * method has no effect.
     * @throws FailedReconstructionException if reconstruction fails for some 
     * reason.
     * @throws CancelledReconstructionException if reconstruction is cancelled.
     */
    @Override
    public void start() throws FailedReconstructionException,
            CancelledReconstructionException {
        mFirstOrientation = null;
        mSlamEstimator = 
                new AbsoluteOrientationConstantVelocityModelSlamEstimator();
        setUpCalibrationData();        
        super.start();
        updateScaleAndOrientation();
    }       
}
