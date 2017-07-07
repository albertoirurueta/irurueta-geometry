/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.TwoViewsReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 14, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Contains configuration for a two view sparse reconstructor.
 */
public class TwoViewsSparseReconstructorConfiguration extends 
        BaseTwoViewsSparseReconstructorConfiguration<
        TwoViewsSparseReconstructorConfiguration> {
    

    /**
     * Constructor.
     */
    public TwoViewsSparseReconstructorConfiguration() { }
    
    /**
     * Creates an instance of a two views sparse reconstructor configuration.
     * @return configuration instance.
     */
    public static TwoViewsSparseReconstructorConfiguration make() {
        return new TwoViewsSparseReconstructorConfiguration();
    }   
}
