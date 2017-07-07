/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.SamplesRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 7, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.estimators.LockedException;
import java.util.List;

/**
 * Refines an instance of type T by taking into account an initial estimation 
 * and a collection of samples.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * used in some other situations.
 * @param <T> type of instance to be refined.
 * @param <S> type of samples.
 */
public abstract class SamplesRefiner<T, S> extends Refiner<T> {
    
    /**
     * Collection of samples.
     */
    protected List<S> mSamples;
    
    /**
     * Constructor.
     */
    public SamplesRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param samples collection of samples.
     */
    public SamplesRefiner(T initialEstimation, boolean keepCovariance,
            List<S> samples) {
        super(initialEstimation, keepCovariance);
        mSamples = samples;
    }
    
    /**
     * Gets collection of samples.
     * @return collection of samples.
     */
    public List<S> getSamples() {
        return mSamples;
    }
    
    /**
     * Sets collection of samples.
     * @param samples collection of samples.
     * @throws LockedException if estimator is locked.
     */
    public void setSamples(List<S> samples) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSamples = samples;
    }
}
