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
        AbsoluteOrientationSlamTwoViewsSparseReconstructorListener,
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
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction step by step,
     * one view at a time.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    @Override
    public boolean processOneView() {
        if (!mRunning) {
            mSlamEstimator = new AbsoluteOrientationSlamEstimator();
            setUpSlamEstimatorListener();
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
