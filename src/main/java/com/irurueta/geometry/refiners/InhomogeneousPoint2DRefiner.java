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
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Refines an inhomogeneous 2D point by taking into account an initial
 * estimation, inlier samples and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class InhomogeneousPoint2DRefiner extends Point2DRefiner<InhomogeneousPoint2D> {

    /**
     * Constructor.
     */
    public InhomogeneousPoint2DRefiner() {
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
    public InhomogeneousPoint2DRefiner(
            final InhomogeneousPoint2D initialEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<Line2D> samples,
            final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers, samples, refinementStandardDeviation);
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
    public InhomogeneousPoint2DRefiner(
            final InhomogeneousPoint2D initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<Line2D> samples, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples, refinementStandardDeviation);
    }

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
    @Override
    public InhomogeneousPoint2D refine() throws NotReadyException, LockedException, RefinerException {
        final var result = new InhomogeneousPoint2D();
        refine(result);
        return result;
    }

    /**
     * Refines provided initial estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
     *
     * @param result instance where refined estimation will be stored.
     * @return true if result improved (decreases) in LMSE terms respect to
     * initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    @Override
    public boolean refine(final InhomogeneousPoint2D result) throws NotReadyException, LockedException,
            RefinerException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        locked = true;

        if (listener != null) {
            listener.onRefineStart(this, initialEstimation);
        }

        final var initialTotalResidual = totalResidual(initialEstimation);

        try {
            final var initParams = initialEstimation.asArray();

            // output values to be fitted/optimized will contain residuals
            final var y = new double[numInliers];
            // input values will contain lines to compute residuals
            final var nDims = Line2D.LINE_NUMBER_PARAMS;
            final var x = new Matrix(numInliers, nDims);
            final var nSamples = inliers.length();
            var pos = 0;
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    final var line = samples.get(i);
                    line.normalize();
                    x.setElementAt(pos, 0, line.getA());
                    x.setElementAt(pos, 1, line.getB());
                    x.setElementAt(pos, 2, line.getC());

                    y[pos] = residuals[i];
                    pos++;
                }
            }

            final var evaluator = new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

                private final Line2D line = new Line2D();

                private final InhomogeneousPoint2D point = new InhomogeneousPoint2D();

                private final GradientEstimator gradientEstimator = new GradientEstimator(p -> {
                    this.point.setCoordinates(p);
                    return residual(this.point, line);
                });

                @Override
                public int getNumberOfDimensions() {
                    return nDims;
                }

                @Override
                public double[] createInitialParametersArray() {
                    return initParams;
                }

                @Override
                public double evaluate(final int i, final double[] point, final double[] params,
                                       final double[] derivatives) throws EvaluationException {
                    // point contains, a,b,c values for line
                    line.setParameters(point);

                    // param contains coordinates of point
                    this.point.setCoordinates(params);

                    final var y = residual(this.point, line);
                    gradientEstimator.gradient(params, derivatives);

                    return y;
                }
            };

            final var fitter = new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y,
                    getRefinementStandardDeviation());

            fitter.fit();

            // obtain estimated params
            final var params = fitter.getA();

            // update point
            result.setCoordinates(params);

            if (keepCovariance) {
                // keep covariance
                covariance = fitter.getCovar();
            }

            final var finalTotalResidual = totalResidual(result);
            final var errorDecreased = finalTotalResidual < initialTotalResidual;

            if (listener != null) {
                listener.onRefineEnd(this, initialEstimation, result, errorDecreased);
            }

            return errorDecreased;
        } catch (final Exception e) {
            throw new RefinerException(e);
        } finally {
            locked = false;
        }
    }
}
