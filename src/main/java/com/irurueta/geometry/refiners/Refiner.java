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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

/**
 * Refines an instance of type T by taking into account an initial estimation.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 * This is a base abstract class to be used by any refiner implementation.
 *
 * @param <T> type of instance to be refined.
 */
public abstract class Refiner<T> {

    /**
     * Indicates whether by default covariance of estimation must be kept.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = false;

    /**
     * Initial estimation.
     */
    protected T initialEstimation;

    /**
     * Indicates whether covariance of estimation must be kept after refinement.
     */
    protected boolean keepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Estimated covariance after refinement.
     */
    protected Matrix covariance;

    /**
     * Indicates if this estimator is locked because a refinement is being
     * computed.
     */
    protected boolean locked;

    /**
     * Listener in charge of attending events generated by this instance.
     */
    protected RefinerListener<T> listener;

    /**
     * Constructor.
     */
    protected Refiner() {
    }

    /**
     * Constructor.
     *
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance    true if covariance of estimation must be kept after
     *                          refinement, false otherwise.
     */
    protected Refiner(final T initialEstimation, final boolean keepCovariance) {
        this.initialEstimation = initialEstimation;
        this.keepCovariance = keepCovariance;
    }

    /**
     * Gets listener in charge of attending events generated by this instance.
     *
     * @return listener in charge of attending events generated by this
     * instance.
     */
    public RefinerListener<T> getListener() {
        return listener;
    }

    /**
     * Sets listener in charge of attending events generated by this instance.
     *
     * @param listener listener in charge of attending events generated by this
     *                 instance.
     */
    public void setListener(final RefinerListener<T> listener) {
        this.listener = listener;
    }

    /**
     * Gets initial estimation.
     *
     * @return initial estimation.
     */
    public T getInitialEstimation() {
        return initialEstimation;
    }

    /**
     * Sets initial estimation.
     *
     * @param initialEstimation initial estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialEstimation(final T initialEstimation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialEstimation = initialEstimation;
    }

    /**
     * Indicates whether covariance of estimation must be kept after refinement
     * or not.
     *
     * @return true if covariance of estimation must be kept after refinement,
     * false otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance of estimation must be kept after refinement
     * or not.
     *
     * @param keepCovariance true if covariance of estimation must be kept after
     *                       refinement, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }


    /**
     * Indicates if this estimator is locked because a refinement is being
     * computed.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Gets estimated covariance after refinement.
     *
     * @return estimated covariance after refinement.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Indicates whether this refiner is ready to start refinement computation.
     *
     * @return true if refiner is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Refines provided initial estimation.
     * Notice that implementations of this method might set a value into result
     * even if error is not improved in LMSE terms.
     *
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (decreases) in LMSE terms respect to
     * initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    public abstract boolean refine(final T result) throws NotReadyException, LockedException, RefinerException;

    /**
     * Refines provided initial estimation.
     *
     * @return refined estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    public abstract T refine() throws NotReadyException, LockedException, RefinerException;

}
