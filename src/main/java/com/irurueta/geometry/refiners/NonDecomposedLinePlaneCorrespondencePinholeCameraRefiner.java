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
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
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
 * A pinhole camera refiner using line/plane correspondences and the
 * Levenberg-Marquardt algorithm to try to decrease overall error in LMSE terms
 * among inlier samples by taking the pinhole camera matrix as a whole without
 * decomposition.
 * Typically, this refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner extends
        LinePlaneCorrespondencePinholeCameraRefiner {

    /**
     * Default value for the weight applied to errors related to suggested
     * camera parameters during computation of projection residuals.
     */
    public static final double DEFAULT_SUGGESTION_ERROR_WEIGHT = 2.0;

    /**
     * Dimensions for refinement.
     */
    private static final int REFINE_DIMS = 12;

    /**
     * Suggestion error weight. This weight is applied to errors related to
     * suggested camera parameters during computation of projection residuals.
     */
    private double suggestionErrorWeight = DEFAULT_SUGGESTION_ERROR_WEIGHT;

    /**
     * Constructor.
     */
    public NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner() {
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
    public NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance,
            final BitSet inliers, final double[] residuals, final int numInliers,
            final List<Plane> samples1, final List<Line2D> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers, samples1, samples2,
                refinementStandardDeviation);
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
    public NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance,
            final InliersData inliersData, final List<Plane> samples1, final List<Line2D> samples2,
            final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, samples2, refinementStandardDeviation);
    }

    /**
     * Gets suggestion error weight. This weight is applied to errors related to
     * suggested camera parameters during computation of projection residuals.
     *
     * @return suggestion error weight.
     */
    public double getSuggestionErrorWeight() {
        return suggestionErrorWeight;
    }

    /**
     * Sets suggestion error weight. This weight is applied to errors related to
     * suggested camera parameters during computation of projection residuals.
     *
     * @param suggestionErrorWeight suggestion error weight.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestionErrorWeight(final double suggestionErrorWeight) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestionErrorWeight = suggestionErrorWeight;
    }

    /**
     * Refines provided initial estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
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
    @Override
    public boolean refine(final PinholeCamera result) throws NotReadyException, LockedException, RefinerException {
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

        try {
            initialEstimation.normalize();

            // output values to be fitted/optimized will contain residuals
            final var y = new double[numInliers];
            // input values will contain line and plane to compute residuals
            final var nDims = Line2D.LINE_NUMBER_PARAMS + Plane.PLANE_NUMBER_PARAMS;
            final var x = new Matrix(numInliers, nDims);
            final var nSamples = inliers.length();
            var pos = 0;
            final var initParams = new double[REFINE_DIMS];
            cameraToParameters(initialEstimation, initParams);

            final var initResidual = residualPowell(initialEstimation, initParams, suggestionErrorWeight);

            final var suggestionResidual = hasSuggestions() ? suggestionResidual(initParams, suggestionErrorWeight) :
                    0.0;
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    final var line = samples2.get(i);
                    final var plane = samples1.get(i);
                    line.normalize();
                    plane.normalize();
                    x.setElementAt(pos, 0, line.getA());
                    x.setElementAt(pos, 1, line.getB());
                    x.setElementAt(pos, 2, line.getC());
                    x.setElementAt(pos, 3, plane.getA());
                    x.setElementAt(pos, 4, plane.getB());
                    x.setElementAt(pos, 5, plane.getC());
                    x.setElementAt(pos, 6, plane.getD());

                    y[pos] = Math.pow(residuals[i], 2.0) + suggestionResidual;
                    pos++;
                }
            }

            final var evaluator = new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

                private final Line2D line = new Line2D();

                private final Plane plane = new Plane();

                private final PinholeCamera pinholeCamera = new PinholeCamera();

                private final GradientEstimator gradientEstimator = new GradientEstimator(params -> {
                    parametersToCamera(params, pinholeCamera);
                    return residualLevenbergMarquardt(pinholeCamera, line, plane, params, suggestionErrorWeight);
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
                    line.setParameters(point[0], point[1], point[2]);
                    plane.setParameters(point[3], point[4], point[5], point[6]);

                    line.normalize();
                    plane.normalize();

                    parametersToCamera(params, pinholeCamera);
                    final var y = residualLevenbergMarquardt(pinholeCamera, line, plane, params, suggestionErrorWeight);
                    gradientEstimator.gradient(params, derivatives);

                    return y;
                }
            };

            final var fitter = new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y, refinementStandardDeviation);

            fitter.fit();

            final var finalParams = fitter.getA();

            parametersToCamera(finalParams, result);

            if (keepCovariance) {
                covariance = fitter.getCovar();
            }

            final var finalResidual = residualPowell(result, finalParams, suggestionErrorWeight);
            final var errorDecreased = finalResidual < initResidual;

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
