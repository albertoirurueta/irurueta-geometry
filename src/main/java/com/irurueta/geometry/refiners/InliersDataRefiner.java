/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;

/**
 * Refines an instance of type T by taking into account an initial estimation, 
 * inlier matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be 
 * useful in some other situations.
 * @param <T> type of instance to be refined.
 */
public abstract class InliersDataRefiner<T> extends Refiner<T> {
    /**
     * Array indicating which of the provided matches are inliers.
     */
    protected BitSet mInliers;

    /**
     * Residuals for matched data corresponding to the initial estimation.
     * Residuals are used to determine the amount of precision of each match
     * in order to find a refined solution that minimizes the LMSE error of all
     * inlier matches.
     */
    protected double[] mResiduals;
    
    /**
     * Number of inliers on initial estimation.
     */
    protected int mNumInliers;   
    
    /**
     * Constructor.
     */
    public InliersDataRefiner() { }
        
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     */
    public InliersDataRefiner(T initialEstimation, boolean keepCovariance,
            BitSet inliers, double[] residuals, int numInliers) {
        super(initialEstimation, keepCovariance);
        mInliers = inliers;
        mResiduals = residuals;
        mNumInliers = numInliers;        
    }    
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     */
    public InliersDataRefiner(T initialEstimation, boolean keepCovariance,
            InliersData inliersData) {
        super(initialEstimation, keepCovariance);
        mInliers = inliersData.getInliers();
        mResiduals = inliersData.getResiduals();
        mNumInliers = inliersData.getNumInliers();        
    }
    
    /**
     * Gets set indicating which of the provided matches are inliers.
     * @return set indicating which of the provided matches are inliers.
     */
    public BitSet getInliers() {
        return mInliers;
    }
    
    /**
     * Specifies set indicating which of the provided matches are inliers.
     * @param inliers set indicating which of the provided matches are inliers.
     * @throws LockedException if estimator is locked.
     */
    public void setInliers(BitSet inliers) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInliers = inliers;
    }
    
    /**
     * Gets residuals for matched samples corresponding to the initial 
     * estimation.
     * Residuals are used to determine the amount of precision of each matched
     * sample in order to find a refined solution that minimizes the LMSE error
     * of all inlier matches.
     * @return residuals for matched samples.
     */
    public double[] getResiduals() {
        return mResiduals;
    }
    
    /**
     * Sets residuals for matched samples corresponding to the initial 
     * estimation.
     * Residuals are used to determine the amount of precision of each matched
     * sample in order to find a refined solution that minimizes the LMSE error
     * of all inlier matches.
     * @param residuals residuals for matched samples.
     * @throws LockedException if estimator is locked.
     */
    public void setResiduals(double[] residuals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mResiduals = residuals;
    }
    
    /**
     * Gets number of inliers on initial estimation.
     * @return number of inliers on initial estimation.
     */
    public int getNumInliers() {
        return mNumInliers;
    }
    
    /**
     * Sets number of inliers on initial estimation.
     * @param numInliers number of inliers on initial estimation.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setNumInliers(int numInliers) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (numInliers <= 0) {
            throw new IllegalArgumentException();
        }
        mNumInliers = numInliers;
    }
    
    /**
     * Gets total number of provided matched samples.
     * @return total number of provided matched samples.
     */
    public int getTotalSamples() {
        return mResiduals != null ? mResiduals.length : 0;
    }
    
    /**
     * Sets inlier data.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @throws LockedException if estimator is locked.
     */
    public void setInliersData(InliersData inliersData) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInliers = inliersData.getInliers();
        mResiduals = inliersData.getResiduals();
        mNumInliers = inliersData.getNumInliers();
    }    
}
