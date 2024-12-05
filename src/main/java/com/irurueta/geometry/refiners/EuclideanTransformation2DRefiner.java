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
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.Point2D;
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
 * Refines a 2D Euclidean transformation by taking into account an initial
 * estimation, inlier point matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class EuclideanTransformation2DRefiner extends
        PairMatchesAndInliersDataRefiner<EuclideanTransformation2D, Point2D, Point2D> {

    /**
     * Point to be reused when computing residuals.
     */
    private final Point2D residualTestPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

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
    public EuclideanTransformation2DRefiner() {
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
    public EuclideanTransformation2DRefiner(
            final EuclideanTransformation2D initialEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<Point2D> samples1, final List<Point2D> samples2,
            final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers, samples1, samples2);
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
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    public EuclideanTransformation2DRefiner(
            final EuclideanTransformation2D initialEstimation, final boolean keepCovariance,
            final InliersData inliersData, final List<Point2D> samples1, final List<Point2D> samples2,
            final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, samples2);
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
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
     * Refines provided initial estimation.
     *
     * @return refines estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    @Override
    public EuclideanTransformation2D refine() throws NotReadyException, LockedException, RefinerException {
        final EuclideanTransformation2D result = new EuclideanTransformation2D();
        refine(result);
        return result;
    }

    /**
     * Refines provided initial estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
     *
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (error decreases) in LMSE terms respect
     * to initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    @Override
    public boolean refine(final EuclideanTransformation2D result) throws NotReadyException, LockedException,
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
            // parameters: rotation angle + translation
            final var initParams = new double[1 + EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
            // copy values
            initParams[0] = initialEstimation.getRotation().getTheta();
            System.arraycopy(initialEstimation.getTranslation(), 0, initParams,
                    1, EuclideanTransformation2D.NUM_TRANSLATION_COORDS);

            // output values to be fitted/optimized will contain residuals
            final var y = new double[numInliers];
            // input values will contain 2 sets of 2D points to compute residuals
            final var nDims = 2 * Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH;
            final var x = new Matrix(numInliers, nDims);
            final var nSamples = inliers.length();
            var pos = 0;
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    final var inputPoint = samples1.get(i);
                    final var outputPoint = samples2.get(i);
                    inputPoint.normalize();
                    outputPoint.normalize();
                    x.setElementAt(pos, 0, inputPoint.getHomX());
                    x.setElementAt(pos, 1, inputPoint.getHomY());
                    x.setElementAt(pos, 2, inputPoint.getHomW());
                    x.setElementAt(pos, 3, outputPoint.getHomX());
                    x.setElementAt(pos, 4, outputPoint.getHomY());
                    x.setElementAt(pos, 5, outputPoint.getHomW());

                    y[pos] = residuals[i];
                    pos++;
                }
            }

            final var evaluator = new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

                private final Point2D inputPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

                private final Point2D outputPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

                private final EuclideanTransformation2D transformation = new EuclideanTransformation2D();

                private final GradientEstimator mGradientEstimator = new GradientEstimator(params -> {
                    // copy values
                    transformation.getRotation().setTheta(params[0]);
                    System.arraycopy(params, 1, transformation.getTranslation(), 0,
                            EuclideanTransformation2D.NUM_TRANSLATION_COORDS);

                    return residual(transformation, inputPoint, outputPoint);
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
                    inputPoint.setHomogeneousCoordinates(point[0], point[1], point[2]);
                    outputPoint.setHomogeneousCoordinates(point[3], point[4], point[5]);

                    // copy values
                    transformation.getRotation().setTheta(params[0]);
                    System.arraycopy(params, 1, transformation.getTranslation(), 0,
                            EuclideanTransformation2D.NUM_TRANSLATION_COORDS);

                    final var y = residual(transformation, inputPoint, outputPoint);
                    mGradientEstimator.gradient(params, derivatives);

                    return y;
                }
            };

            final var fitter = new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y,
                    getRefinementStandardDeviation());

            fitter.fit();

            // obtain estimated params
            final var params = fitter.getA();

            // update transformation
            result.getRotation().setTheta(params[0]);
            System.arraycopy(params, 1, result.getTranslation(), 0,
                    EuclideanTransformation2D.NUM_TRANSLATION_COORDS);

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

    /**
     * Computes the residual between the Euclidean transformation and a pair or
     * matched points.
     *
     * @param transformation a transformation.
     * @param inputPoint     input 2D point.
     * @param outputPoint    output 2D point.
     * @return residual.
     */
    private double residual(final EuclideanTransformation2D transformation, final Point2D inputPoint,
                            final Point2D outputPoint) {
        inputPoint.normalize();
        outputPoint.normalize();

        transformation.transform(inputPoint, residualTestPoint);
        return residualTestPoint.distanceTo(outputPoint);
    }

    /**
     * Computes total residual among all provided inlier samples.
     *
     * @param transformation a transformation.
     * @return total residual.
     */
    private double totalResidual(final EuclideanTransformation2D transformation) {
        var result = 0.0;

        final var nSamples = inliers.length();
        for (var i = 0; i < nSamples; i++) {
            if (inliers.get(i)) {
                // sample is inlier
                final var inputPoint = samples1.get(i);
                final var outputPoint = samples2.get(i);
                inputPoint.normalize();
                outputPoint.normalize();
                result += residual(transformation, inputPoint, outputPoint);
            }
        }

        return result;
    }
}
