/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.SamplesAndInliersDataRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 7, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Refines an instance of type T by taking into account an initial estimation,
 * inlier samples, their residuals and a collection of samples.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 * @param <T> type of instance to be refined.
 * @param <S> type of samples.
 */
public abstract class SamplesAndInliersDataRefiner<T, S> extends 
        InliersDataRefiner<T> {
    
    /**
     * Collection of samples.
     */
    protected List<S> mSamples;
    
    /**
     * Constructor.
     */
    public SamplesAndInliersDataRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples collection of samples.
     */
    public SamplesAndInliersDataRefiner(T initialEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals,
            int numInliers, List<S> samples) {
        super(initialEstimation, keepCovariance, inliers, residuals, 
                numInliers);
        mSamples = samples;
    }

    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples collection of samples.
     */
    public SamplesAndInliersDataRefiner(T initialEstimation, 
            boolean keepCovariance, InliersData inliersData, List<S> samples) {
        super(initialEstimation, keepCovariance, inliersData);
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
    
    /**
     * Indicates whether this refiner is ready to start refinement computation.
     * @return true if refiner is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mInitialEstimation != null && mInliers != null &&
                mResiduals != null && mSamples != null &&
                mResiduals.length == mSamples.size() && mNumInliers > 0;
    }
}
