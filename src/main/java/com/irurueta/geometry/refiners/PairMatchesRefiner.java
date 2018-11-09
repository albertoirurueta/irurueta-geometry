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

import java.util.List;

/**
 * Refines an instance of type T by taking into account an initial estimation
 * and pairs of matched samples.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 * @param <T> type of instance to be refined.
 * @param <S1> type of matched samples in 1st set.
 * @param <S2> type of matched samples in 2nd set.
 */
public abstract class PairMatchesRefiner<T, S1, S2> extends Refiner<T> {
    
    /**
     * 1st set of paired samples.
     */
    protected List<S1> mSamples1;
    
    /**
     * 2nd set of paired samples.
     */
    protected List<S2> mSamples2;
    
    /**
     * Constructor.
     */
    public PairMatchesRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     */
    public PairMatchesRefiner(T initialEstimation, boolean keepCovariance,
            List<S1> samples1, List<S2> samples2) {
        super(initialEstimation, keepCovariance);
        mSamples1 = samples1;
        mSamples2 = samples2;
    }
    
    /**
     * Gets 1st set of paired samples.
     * @return 1st set of paired samples.
     */
    public List<S1> getSamples1() {
        return mSamples1;
    }
    
    /**
     * Sets 1st set of paired samples.
     * @param samples1 1st set of paired samples.
     * @throws LockedException if estimator is locked.
     */
    public void setSamples1(List<S1> samples1) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSamples1 = samples1;
    }
    
    /**
     * Gets 2nd set of paired samples.
     * @return 2nd set of paired samples.
     */
    public List<S2> getSamples2() {
        return mSamples2;
    }
    
    /**
     * Sets 2nd set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @throws LockedException if estimator is locked.
     */
    public void setSamples2(List<S2> samples2) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSamples2 = samples2;
    }
}
