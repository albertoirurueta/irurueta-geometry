/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.ConstantVelocityModelSlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 18, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.ConstantVelocityModelSlamEstimator;

/**
 * Estimates cameras and 3D reconstructed points from sparse image point
 * correspondences in two views and using SLAM (with accelerometer and gyroscope
 * data) with constant velocity model for overall scale estimation.
 */
public class ConstantVelocityModelSlamTwoViewsSparseReconstructor extends 
        BaseSlamTwoViewsSparseReconstructor<
        ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration,
        ConstantVelocityModelSlamTwoViewsSparseReconstructor, 
        ConstantVelocityModelSlamEstimator> {
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public ConstantVelocityModelSlamTwoViewsSparseReconstructor(
            ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration,
            ConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public ConstantVelocityModelSlamTwoViewsSparseReconstructor(
            ConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener)
            throws NullPointerException {
        this(new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration(), 
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
        mSlamEstimator = new ConstantVelocityModelSlamEstimator();
        setUpCalibrationData();
        super.start();
        updateScale();
    }        
}
