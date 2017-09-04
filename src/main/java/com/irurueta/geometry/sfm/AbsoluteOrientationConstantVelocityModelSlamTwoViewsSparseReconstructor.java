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
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction step by step,
     * one view at a time.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    @Override
    public boolean processOneView() {
        if (!mRunning) {
            mSlamEstimator = new AbsoluteOrientationConstantVelocityModelSlamEstimator();
            setUpCalibrationData();
        }

        return super.processOneView();
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on those
     * implementations where scale can be measured or is already known.
     * @return true if post processing succeeded, false otherwise.
     */
    @Override
    protected boolean postProcessOne() {
        return updateScaleAndOrientation();
    }
}
