/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.KnownBaselineTwoViewsSparseReconstructorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 17, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction
 * from sparse image point correspondences in two views when baseline is known.
 */
public interface KnownBaselineTwoViewsSparseReconstructorListener extends 
        BaseTwoViewsSparseReconstructorListener<
        KnownBaselineTwoViewsSparseReconstructor> { }
