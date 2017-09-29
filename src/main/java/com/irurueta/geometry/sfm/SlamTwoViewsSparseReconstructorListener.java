/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.SlamTwoViewsSparseReconstructorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 17, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction
 * from sparse image point correspondences in two views and using SLAM for scale
 * estimation.
 */
public interface SlamTwoViewsSparseReconstructorListener extends
        BaseSlamTwoViewsSparseReconstructorListener<
        SlamTwoViewsSparseReconstructor> { }
