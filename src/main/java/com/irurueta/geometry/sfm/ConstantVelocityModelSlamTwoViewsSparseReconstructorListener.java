/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.ConstantVelocityModelSlamTwoViewsSparseReconstructorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 18, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction
 * from sparse image point correspondence in two views and using SLAM with
 * constant velocity model for scale estimation.
 */
public interface ConstantVelocityModelSlamTwoViewsSparseReconstructorListener 
        extends BaseSlamTwoViewsSparseReconstructorListener<
        ConstantVelocityModelSlamTwoViewsSparseReconstructor> { }
