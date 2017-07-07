/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.TwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 14, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in two views.
 */
public class TwoViewsSparseReconstructor extends 
        BaseTwoViewsSparseReconstructor<
        TwoViewsSparseReconstructorConfiguration,
        TwoViewsSparseReconstructor> {
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public TwoViewsSparseReconstructor(
            TwoViewsSparseReconstructorConfiguration configuration,
            TwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor with default configuration.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public TwoViewsSparseReconstructor(
            TwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        this(new TwoViewsSparseReconstructorConfiguration(), listener);
    }                        
}
