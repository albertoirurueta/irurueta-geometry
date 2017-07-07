/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.AbsoluteOrientationSlamTwoViewsSparseReconstructorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 18, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction 
 * from sparse image point correspondences in two views and using SLAM with
 * absolute orientation for scale and orientation estimation.
 */
public interface AbsoluteOrientationSlamTwoViewsSparseReconstructorListener 
        extends BaseTwoViewsSparseReconstructorListener<
        AbsoluteOrientationSlamTwoViewsSparseReconstructor> { }
