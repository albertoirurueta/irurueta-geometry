/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.SlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 17, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.SlamEstimator;

/**
 * Estimates cameras and 3D reconstructed points from sparse image point 
 * correspondences in two views and using SLAM (with accelerometer and gyroscope
 * data) for overall scale estimation.
 */
public class SlamTwoViewsSparseReconstructor extends 
        BaseSlamTwoViewsSparseReconstructor<
        SlamTwoViewsSparseReconstructorConfiguration, 
        SlamTwoViewsSparseReconstructor, SlamEstimator>{
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public SlamTwoViewsSparseReconstructor(
            SlamTwoViewsSparseReconstructorConfiguration configuration,
            SlamTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public SlamTwoViewsSparseReconstructor(
            SlamTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        this(new SlamTwoViewsSparseReconstructorConfiguration(), listener);
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction step by step,
     * one view at a time.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    @Override
    public synchronized boolean processOneView() {
        if (!mRunning) {
            mSlamEstimator = new SlamEstimator();
            setUpCalibrationData();
        }

        boolean result = super.processOneView();

        if (!result) {
            updateScale();
        }

        return result;
    }
}
