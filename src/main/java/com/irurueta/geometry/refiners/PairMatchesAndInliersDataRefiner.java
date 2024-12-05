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
import java.util.List;

/**
 * Refines an instance of type T by taking into account an initial estimation,
 * inlier matches, their residuals and pairs of matches samples.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 *
 * @param <T>  type of instance to be refined.
 * @param <S1> type of matched samples in 1st set.
 * @param <S2> type of matched samples in 2nd set.
 */
public abstract class PairMatchesAndInliersDataRefiner<T, S1, S2> extends InliersDataRefiner<T> {

    /**
     * 1st set of paired samples.
     */
    protected List<S1> samples1;

    /**
     * 2nd set of paired samples.
     */
    protected List<S2> samples2;

    /**
     * Constructor.
     */
    protected PairMatchesAndInliersDataRefiner() {
    }

    /**
     * Constructor.
     *
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance    true if covariance of estimation must be kept after
     *                          refinement, false otherwise.
     * @param inliers           set indicating which of the provided matches are inliers.
     * @param residuals         residuals for matched samples.
     * @param numInliers        number of inliers on initial estimation.
     * @param samples1          1st set of paired samples.
     * @param samples2          2nd set of paired samples.
     */
    protected PairMatchesAndInliersDataRefiner(
            final T initialEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<S1> samples1, final List<S2> samples2) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers);
        this.samples1 = samples1;
        this.samples2 = samples2;
    }

    /**
     * Constructor.
     *
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance    true if covariance of estimation must be kept after
     *                          refinement, false otherwise.
     * @param inliersData       inlier data, typically obtained from a robust
     *                          estimator.
     * @param samples1          1st set of paired samples.
     * @param samples2          2nd set of paired samples.
     */
    protected PairMatchesAndInliersDataRefiner(
            final T initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<S1> samples1, final List<S2> samples2) {
        super(initialEstimation, keepCovariance, inliersData);
        this.samples1 = samples1;
        this.samples2 = samples2;
    }

    /**
     * Gets 1st set of paired samples.
     *
     * @return 1st set of paired samples.
     */
    public List<S1> getSamples1() {
        return samples1;
    }

    /**
     * Sets 1st set of paired samples.
     *
     * @param samples1 1st set of paired samples.
     * @throws LockedException if estimator is locked.
     */
    public void setSamples1(final List<S1> samples1) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.samples1 = samples1;
    }

    /**
     * Gets 2nd set of paired samples.
     *
     * @return 2nd set of paired samples.
     */
    public List<S2> getSamples2() {
        return samples2;
    }

    /**
     * Sets 2nd set of paired samples.
     *
     * @param samples2 2nd set of paired samples.
     * @throws LockedException if estimator is locked.
     */
    public void setSamples2(final List<S2> samples2) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.samples2 = samples2;
    }

    /**
     * Indicates whether this refiner is ready to start refinement computation.
     *
     * @return true if refiner is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return initialEstimation != null && inliers != null && residuals != null && samples1 != null
                && samples2 != null && residuals.length == samples1.size() && samples1.size() == samples2.size()
                && numInliers > 0;
    }
}
