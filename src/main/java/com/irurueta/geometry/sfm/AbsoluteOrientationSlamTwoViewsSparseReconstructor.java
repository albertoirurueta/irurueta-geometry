/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.AbsoluteOrientationSlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 18, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.AbsoluteOrientationSlamEstimator;

/**
 * Estimates cameras and 3D reconstructed points from sparse image point 
 * correspondences in two views and using SLAM (with accelerometer and gyroscope
 * data) with absolute orientation for overall scale and orientation estimation.
 */
public class AbsoluteOrientationSlamTwoViewsSparseReconstructor extends 
        BaseAbsoluteOrientationSlamTwoViewsSparseReconstructor<
        AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration,
        AbsoluteOrientationSlamTwoViewsSparseReconstructor,
        AbsoluteOrientationSlamEstimator> {
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public AbsoluteOrientationSlamTwoViewsSparseReconstructor(
            AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration configuration,
            AbsoluteOrientationSlamTwoViewsSparseReconstructorListener listener)
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public AbsoluteOrientationSlamTwoViewsSparseReconstructor(
            AbsoluteOrientationSlamTwoViewsSparseReconstructorListener listener)
            throws NullPointerException {
        this(new AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration(), 
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
        mSlamEstimator = new AbsoluteOrientationSlamEstimator();
        setUpCalibrationData();
        super.start();
        updateScaleAndOrientation();
    }    
}
