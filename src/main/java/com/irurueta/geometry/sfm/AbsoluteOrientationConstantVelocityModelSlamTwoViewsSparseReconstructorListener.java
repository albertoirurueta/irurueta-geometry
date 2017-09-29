/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 18, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction
 * from sparse image point correspondences in two views and using SLAM with
 * absolute orientation and constant velocity model for scale and orientation
 * estimation.
 */
public interface AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener 
        extends BaseSlamTwoViewsSparseReconstructorListener<
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor> { }
