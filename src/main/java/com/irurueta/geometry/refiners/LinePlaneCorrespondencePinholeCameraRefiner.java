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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Base class for a pinhole camera refiner using line/plane correspondences.
 * Implementations of this class refine a pinhole camera by taking into account
 * an initial estimation, inlier line/plane matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
public abstract class LinePlaneCorrespondencePinholeCameraRefiner extends
        PinholeCameraRefiner<Plane, Line2D> {

    /**
     * Plane to be reused when computing residuals.
     */
    private final Plane mResidualTestPlane = new Plane();

    /**
     * Constructor.
     */
    protected LinePlaneCorrespondencePinholeCameraRefiner() {
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
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    protected LinePlaneCorrespondencePinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance,
            final BitSet inliers, final double[] residuals, final int numInliers,
            final List<Plane> samples1, final List<Line2D> samples2,
            final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers,
                samples1, samples2, refinementStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param initialEstimation           initial estimation to be set.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliersData                 inlier data, typically obtained from a robust
     *                                    estimator.
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    protected LinePlaneCorrespondencePinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance,
            final InliersData inliersData, final List<Plane> samples1,
            final List<Line2D> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1,
                samples2, refinementStandardDeviation);
    }

    /**
     * Total residual to be used during Powell refinement.
     * Powell refinement uses Powell algorithm to minimize a cost function
     * consisting on the sum of squared projection residuals plus the
     * suggestion residual for any suggested terms.
     *
     * @param pinholeCamera camera to be checked.
     * @param params        camera parameters. In the following order:
     *                      skewness, horizontal focal length, vertical focal length,
     *                      horizontal principal point, vertical principal point, quaternion A,
     *                      quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight        weight for suggestion residual.
     * @return total residual during Powell refinement.
     */
    protected double residualPowell(final PinholeCamera pinholeCamera,
                                    final double[] params, final double weight) {
        return backprojectionResidual(pinholeCamera) +
                suggestionResidual(params, weight);
    }

    /**
     * Computes total line back-projection residual for provided camera.
     * This method computes the sum of the squared residuals for all inlier
     * back-projected lines.
     *
     * @param pinholeCamera camera to compute residual for.
     * @return total back-projection residual.
     */
    private double backprojectionResidual(final PinholeCamera pinholeCamera) {
        pinholeCamera.normalize();

        // back-projection inlier lines into test plane
        final int nSamples = mInliers.length();
        Line2D line;
        Plane plane;
        double residual = 0.0;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                line = mSamples2.get(i);
                plane = mSamples1.get(i);

                line.normalize();
                plane.normalize();

                residual += Math.pow(singleBackprojectionResidual(
                        pinholeCamera, line, plane), 2.0);
            }
        }

        return residual;
    }

    /**
     * Computes total residual to be used during Levenberg/Marquard covariance
     * estimation.
     *
     * @param pinholeCamera camera to estimate covariance for.
     * @param line          2D line to be back-projected with provided pinhole camera.
     * @param plane         plane to be compared with back-projected line.
     * @param params        camera parameters. In the following order:
     *                      skewness, horizontal focal length, vertical focal length,
     *                      horizontal principal point, vertical principal point, quaternion A,
     *                      quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight        weight for suggestion residual.
     * @return total residual.
     */
    protected double residualLevenbergMarquardt(
            final PinholeCamera pinholeCamera, final Line2D line, final Plane plane,
            final double[] params, final double weight) {
        double residual = singleBackprojectionResidual(pinholeCamera, line,
                plane);
        if (hasSuggestions()) {
            residual += suggestionResidual(params, weight);
        }
        return residual;
    }

    /**
     * Back-projection residual/error for a single line using provided camera.
     *
     * @param pinholeCamera camera ot be checked.
     * @param line          line to be back-projected.
     * @param plane         plane to check against.
     * @return dot product distance between back-projected line and plane.
     */
    @SuppressWarnings("DuplicatedCode")
    private double singleBackprojectionResidual(final PinholeCamera pinholeCamera,
                                                final Line2D line, final Plane plane) {
        // back-project line into test plane
        pinholeCamera.backProject(line, mResidualTestPlane);
        mResidualTestPlane.normalize();

        final double dotProduct = Math.abs(plane.getA() * mResidualTestPlane.getA() +
                plane.getB() * mResidualTestPlane.getB() +
                plane.getC() * mResidualTestPlane.getC() +
                plane.getD() * mResidualTestPlane.getD());
        return 1.0 - dotProduct;
    }
}
