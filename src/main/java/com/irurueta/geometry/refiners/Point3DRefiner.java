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

import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Refines a 3D point by taking into account an initial estimation, inlier
 * samples and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 *
 * @param <T> an implementation of a 3D point.
 */
public abstract class Point3DRefiner<T extends Point3D> extends SamplesAndInliersDataRefiner<T, Plane> {

    /**
     * Standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     */
    private double refinementStandardDeviation;

    /**
     * Constructor.
     */
    protected Point3DRefiner() {
    }

    /**
     * Constructor.
     *
     * @param initialEstimation           initial estimation to be set.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliers                     set indicating which of the provided matches are inliers.
     * @param residuals                   residuals for matched samples.
     * @param numInliers                  number of inliers on initial estimation.
     * @param samples                     collection of samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    protected Point3DRefiner(
            final T initialEstimation, final boolean keepCovariance, final BitSet inliers, double[] residuals,
            final int numInliers, final List<Plane> samples, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers, samples);
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param initialEstimation           initial estimation to be set.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliersData                 inlier data, typically obtained from a robust
     *                                    estimator.
     * @param samples                     collection of samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    protected Point3DRefiner(
            final T initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<Plane> samples, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples);
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @return standard deviation used for refinement.
     */
    public double getRefinementStandardDeviation() {
        return refinementStandardDeviation;
    }

    /**
     * Sets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @param refinementStandardDeviation standard deviation used for
     *                                    refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementStandardDeviation(final double refinementStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Computes the residual between a point and a plane as their distance.
     *
     * @param point a point.
     * @param plane a plane.
     * @return residual (distance between provided point and plane).
     */
    protected double residual(final Point3D point, final Plane plane) {
        point.normalize();
        plane.normalize();
        return Math.abs(plane.signedDistance(point));
    }

    /**
     * Computes total residual among all provided inlier samples.
     *
     * @param point a point.
     * @return total residual.
     */
    protected double totalResidual(final Point3D point) {
        var result = 0.0;

        final var nSamples = inliers.length();
        for (var i = 0; i < nSamples; i++) {
            if (inliers.get(i)) {
                // sample is inlier
                final var plane = samples.get(i);
                result += residual(point, plane);
            }
        }

        return result;
    }
}
