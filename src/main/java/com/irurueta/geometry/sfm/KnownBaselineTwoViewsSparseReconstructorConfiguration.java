/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.KnownBaselineTwoViewsSparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 21, 2017.
 */
package com.irurueta.geometry.sfm;

/**
 * Contains configuration for a two view sparse reconstructor assuming that the
 * baseline (separation between cameras) is known.
 */
public class KnownBaselineTwoViewsSparseReconstructorConfiguration extends 
        BaseTwoViewsSparseReconstructorConfiguration<
        KnownBaselineTwoViewsSparseReconstructorConfiguration> {
    
    /**
     * Default camera baseline (expressed in a unit of distance such as meters).
     * Methods such as DIAC or Essential assume that the camera baseline is 1.0,
     * which yields a reconstruction up to scale, unless the real baseline is
     * provided.
     */
    public static final double DEFAULT_BASELINE = 1.0;
    
    /**
     * Camera baseline (expressed in a unit of distance such as meters).
     * Contains the real separation between camera centers so that the real
     * scale of cameras and reconstructed points can be retrieved.
     */
    private double mBaseline = DEFAULT_BASELINE;
    
    /**
     * Constructor.
     */
    public KnownBaselineTwoViewsSparseReconstructorConfiguration() { }
    
    /**
     * Creates an instance of a two views sparse reconstructor configuration 
     * with known camera baseline.
     * @return configuration instance.
     */
    public static KnownBaselineTwoViewsSparseReconstructorConfiguration make() {
        return new KnownBaselineTwoViewsSparseReconstructorConfiguration();
    }
    
    /**
     * Gets camera baseline (expressed in a unit of distance such as meters).
     * Contains the real separation between camera centers so that the real
     * scale of cameras and reconstructed points can be retrieved.
     * @return camera baseline.
     */
    public double getBaseline() {
        return mBaseline;
    }
    
    /**
     * Sets camera baseline (expressed in a unit of distance such as meters).
     * Contains the real separation between camera centers so that the real
     * scale of cameras and reconstructed points can be retrieved.
     * @param baseline camera baseline.
     * @return this instance so that method can be easily chained.
     */
    public KnownBaselineTwoViewsSparseReconstructorConfiguration setBaseline(
            double baseline) {
        mBaseline = baseline;
        return this;
    }
}
