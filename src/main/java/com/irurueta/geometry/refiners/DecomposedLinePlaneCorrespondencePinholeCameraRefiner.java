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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.optimization.PowellMultiOptimizer;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * A pinhole camera refiner using line/plane correspondences and the
 * Powell algorithm to try to decrease overall error in LMSE terms among
 * inlier samples by taking the decomposed parameters of a pinhole camera.
 * Typically, this refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class DecomposedLinePlaneCorrespondencePinholeCameraRefiner extends LinePlaneCorrespondencePinholeCameraRefiner {

    /**
     * Default value for minimum suggestion weight. This weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    public static final double DEFAULT_MIN_SUGGESTION_WEIGHT = 0.1;

    /**
     * Default value for maximum suggestion weight. This weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    public static final double DEFAULT_MAX_SUGGESTION_WEIGHT = 2.0;

    /**
     * Default value for the step to increase suggestion weight. This weight is
     * used to slowly draw original camera parameters into desired suggested
     * values. Suggestion weight slowly increases each time Levenberg-Marquardt
     * is used to find a solution so that the algorithm can converge into
     * desired value. The faster the weights are increased the less likely that
     * suggested values can be converged if they differ too much from the
     * original ones.
     */
    public static final double DEFAULT_SUGGESTION_WEIGHT_STEP = 0.475;

    /**
     * Dimensions for refinement.
     */
    private static final int REFINE_DIMS = 12;

    /**
     * Minimum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    private double minSuggestionWeight = DEFAULT_MIN_SUGGESTION_WEIGHT;

    /**
     * Maximum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    private double maxSuggestionWeight = DEFAULT_MAX_SUGGESTION_WEIGHT;

    /**
     * Step to increase suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     */
    private double suggestionWeightStep = DEFAULT_SUGGESTION_WEIGHT_STEP;

    /**
     * Instance of a pinhole camera to be reused during refinement.
     */
    private PinholeCamera refineCamera;

    /**
     * Current weight during refinement.
     */
    private double currentWeight;

    /**
     * Constructor.
     */
    public DecomposedLinePlaneCorrespondencePinholeCameraRefiner() {
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
    public DecomposedLinePlaneCorrespondencePinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<Plane> samples1, final List<Line2D> samples2,
            final double refinementStandardDeviation) {
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
    public DecomposedLinePlaneCorrespondencePinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<Plane> samples1, final List<Line2D> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, samples2, refinementStandardDeviation);
    }

    /**
     * Gets minimum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @return minimum suggestion weight.
     */
    public double getMinSuggestionWeight() {
        return minSuggestionWeight;
    }

    /**
     * Sets minimum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @param minSuggestionWeight minimum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMinSuggestionWeight(final double minSuggestionWeight) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.minSuggestionWeight = minSuggestionWeight;
    }

    /**
     * Gets maximum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @return maximum suggestion weight.
     */
    public double getMaxSuggestionWeight() {
        return maxSuggestionWeight;
    }

    /**
     * Sets maximum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMaxSuggestionWeight(final double maxSuggestionWeight) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.maxSuggestionWeight = maxSuggestionWeight;
    }

    /**
     * Sets minimum and maximum suggestion weights. Suggestion weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @param minSuggestionWeight minimum suggestion weight.
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if minimum suggestion weight is greater
     *                                  or equal than maximum value.
     */
    public void setMinMaxSuggestionWeight(final double minSuggestionWeight, final double maxSuggestionWeight)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minSuggestionWeight >= maxSuggestionWeight) {
            throw new IllegalArgumentException();
        }

        this.minSuggestionWeight = minSuggestionWeight;
        this.maxSuggestionWeight = maxSuggestionWeight;
    }

    /**
     * Gets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     *
     * @return step to increase suggestion weight.
     */
    public double getSuggestionWeightStep() {
        return suggestionWeightStep;
    }

    /**
     * Sets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     *
     * @param suggestionWeightStep step to increase suggestion weight.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided step is negative or zero.
     */
    public void setSuggestionWeightStep(final double suggestionWeightStep) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (suggestionWeightStep <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.suggestionWeightStep = suggestionWeightStep;
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
     */
    @Override
    public boolean refine(final PinholeCamera result) throws NotReadyException, LockedException {
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

        final var improved = refinePowell(result);

        if (keepCovariance) {
            covariance = estimateCovarianceLevenbergMarquardt(improved ? result : initialEstimation, currentWeight);
        }

        if (listener != null) {
            listener.onRefineEnd(this, initialEstimation, result, improved);
        }

        locked = false;

        return improved;
    }

    /**
     * Estimates covariance matrix for provided estimated and refined camera
     *
     * @param pinholeCamera pinhole camera to estimate covariance for.
     * @param weight        weight for suggestion residual.
     * @return estimated covariance or null if anything fails.
     */
    private Matrix estimateCovarianceLevenbergMarquardt(final PinholeCamera pinholeCamera, final double weight) {
        try {
            pinholeCamera.normalize();

            // output values to be fitted/optimized will contain residuals
            final var y = new double[numInliers];
            // input values will contain 2D line and 3D plane to compute
            // residuals
            final var nDims = Plane.PLANE_NUMBER_PARAMS + Line2D.LINE_NUMBER_PARAMS;
            final var x = new Matrix(numInliers, nDims);
            final var nSamples = inliers.length();
            var pos = 0;
            final var initParams = new double[REFINE_DIMS];
            cameraToParameters(pinholeCamera, initParams);

            final var suggestionResidual = hasSuggestions() ? suggestionResidual(initParams, weight) : 0.0;
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    final var plane = samples1.get(i);
                    final var line = samples2.get(i);
                    plane.normalize();
                    line.normalize();
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

                private final Plane plane = new Plane();

                private final Line2D line = new Line2D();

                private final PinholeCamera pinholeCamera = new PinholeCamera();

                private final GradientEstimator gradientEstimator = new GradientEstimator(params -> {
                    parametersToCamera(params, pinholeCamera);
                    return residualLevenbergMarquardt(pinholeCamera, line, plane, params, weight);
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

                    parametersToCamera(params, pinholeCamera);
                    final var y = residualLevenbergMarquardt(pinholeCamera, line, plane, params, weight);
                    gradientEstimator.gradient(params, derivatives);

                    return y;
                }
            };

            final var fitter = new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y,
                    getRefinementStandardDeviation());

            fitter.fit();

            // obtain covariance
            return fitter.getCovar();

        } catch (final Exception e) {
            // estimation failed, so we return null
            return null;
        }
    }

    /**
     * Refines camera using Powell optimization to minimize a cost function
     * consisting on the sum of squared projection residuals plus the
     * suggestion residual for any suggested terms.
     *
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (decreases) in LMSE terms respect to
     * initial estimation, false if no improvement has been achieved.
     */
    private boolean refinePowell(final PinholeCamera result) {
        var improvedAtLeastOnce = false;
        currentWeight = minSuggestionWeight;

        if (hasSuggestions()) {
            try {
                // copy camera into a new instance
                refineCamera = new PinholeCamera(new Matrix(initialEstimation.getInternalMatrix()));
                refineCamera.normalize();

                final var startPoint = new double[REFINE_DIMS];
                final var listener = new RefinementMultiDimensionFunctionEvaluatorListener();
                final var optimizer = new PowellMultiOptimizer(listener, PowellMultiOptimizer.DEFAULT_TOLERANCE);

                boolean improved;
                do {
                    improved = refinementStepPowell(optimizer, listener, startPoint, currentWeight);

                    if (improved) {
                        // update result
                        result.setInternalMatrix(new Matrix(refineCamera.getInternalMatrix()));
                        improvedAtLeastOnce = true;
                    }

                    currentWeight += suggestionWeightStep;
                } while (currentWeight < maxSuggestionWeight && improved);

                return improvedAtLeastOnce;
            } catch (final GeometryException | NumericalException | AlgebraException e) {
                // refinement failed, so we return input value
                return improvedAtLeastOnce;
            }
        }
        return false;
    }

    /**
     * Computes one refinement step using Powell optimizer for a given weight
     * on suggestion terms.
     *
     * @param optimizer  Powell optimizer to be reused.
     * @param listener   Powell optimizer listener to be reused.
     * @param startPoint starting point for powell optimization. This array is
     *                   passed only for reuse purposes.
     * @param weight     suggestion terms weight.
     * @return true if this refinement step decreased projection error in LMSE
     * terms, false otherwise.
     * @throws GeometryException  if something failed.
     * @throws NumericalException if something failed.
     */
    private boolean refinementStepPowell(
            final PowellMultiOptimizer optimizer, final RefinementMultiDimensionFunctionEvaluatorListener listener,
            final double[] startPoint, final double weight) throws GeometryException, NumericalException {

        listener.weight = weight;
        cameraToParameters(refineCamera, startPoint);
        final var initResidual = residualPowell(refineCamera, startPoint, weight);

        optimizer.setStartPoint(startPoint);
        optimizer.minimize();

        final var resultParams = optimizer.getResult();
        parametersToCamera(resultParams, refineCamera);

        final var finalResidual = residualPowell(refineCamera, resultParams, weight);

        return finalResidual < initResidual;
    }

    /**
     * Listener for powell optimizer to minimize cost function during
     * refinement.
     * A weight can be provided so that required parameters are slowly drawn
     * to suggested values.
     */
    private class RefinementMultiDimensionFunctionEvaluatorListener implements MultiDimensionFunctionEvaluatorListener {
        /**
         * Weight to slowly draw parameters to suggested values.
         */
        double weight;

        /**
         * Evaluates cost function
         *
         * @param point parameters to evaluate cost function.
         * @return cost value.
         */
        @Override
        public double evaluate(final double[] point) {
            parametersToCamera(point, refineCamera);
            return residualPowell(refineCamera, point, weight);
        }
    }
}
