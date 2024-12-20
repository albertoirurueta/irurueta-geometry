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
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point3D;
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
 * A 3D affine transformation refiner using point correspondences.
 * This class takes into account an initial estimation, inlier point matches and
 * their residuals to find a solution that minimizes error of inliers in LMSE
 * terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class PointCorrespondenceAffineTransformation3DRefiner extends AffineTransformation3DRefiner<Point3D, Point3D> {

    /**
     * Point to be reused when computing residuals.
     */
    private final Point3D residualTestPoint = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

    /**
     * Constructor.
     */
    public PointCorrespondenceAffineTransformation3DRefiner() {
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
    public PointCorrespondenceAffineTransformation3DRefiner(
            final AffineTransformation3D initialEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<Point3D> samples1, final List<Point3D> samples2,
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
    public PointCorrespondenceAffineTransformation3DRefiner(
            final AffineTransformation3D initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<Point3D> samples1, final List<Point3D> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, samples2, refinementStandardDeviation);
    }

    /**
     * Refines provide initial estimation.
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
    public boolean refine(final AffineTransformation3D result) throws NotReadyException, LockedException,
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
            final var initParams = new double[AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS
                    + AffineTransformation3D.NUM_TRANSLATION_COORDS];
            // copy values for A matrix
            System.arraycopy(initialEstimation.getA().getBuffer(), 0, initParams, 0,
                    AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS);
            // copy values for translation
            System.arraycopy(initialEstimation.getTranslation(), 0, initParams,
                    AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.NUM_TRANSLATION_COORDS);

            // output values to be fitted/optimized will contain residuals
            final var y = new double[numInliers];
            // input values will contain 2 sets of 2D points to compute residuals
            final var nDims = 2 * Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH;
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
                    x.setElementAt(pos, 2, inputPoint.getHomZ());
                    x.setElementAt(pos, 3, inputPoint.getHomW());
                    x.setElementAt(pos, 4, outputPoint.getHomX());
                    x.setElementAt(pos, 5, outputPoint.getHomY());
                    x.setElementAt(pos, 6, outputPoint.getHomZ());
                    x.setElementAt(pos, 7, outputPoint.getHomW());

                    y[pos] = residuals[i];
                    pos++;
                }
            }

            final var evaluator = new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

                private final Point3D inputPoint = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

                private final Point3D outputPoint = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

                private final AffineTransformation3D transformation = new AffineTransformation3D();

                private final GradientEstimator gradientEstimator = new GradientEstimator(params -> {
                    // copy values for A matrix
                    System.arraycopy(params, 0, transformation.getA().getBuffer(), 0,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS);
                    // copy values for translation
                    System.arraycopy(params,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS,
                            transformation.getTranslation(), 0, AffineTransformation3D.NUM_TRANSLATION_COORDS);

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
                    inputPoint.setHomogeneousCoordinates(point[0], point[1], point[2], point[3]);
                    outputPoint.setHomogeneousCoordinates(point[4], point[5], point[6], point[7]);

                    // copy values for A matrix
                    System.arraycopy(params, 0, transformation.getA().getBuffer(), 0,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS);
                    // copy values for translation
                    System.arraycopy(params,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS,
                            transformation.getTranslation(), 0, AffineTransformation3D.NUM_TRANSLATION_COORDS);

                    final var y = residual(transformation, inputPoint, outputPoint);
                    gradientEstimator.gradient(params, derivatives);

                    return y;
                }
            };

            final var fitter = new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y,
                    getRefinementStandardDeviation());

            fitter.fit();

            // obtain estimated params
            final var params = fitter.getA();

            // update transformation

            // copy values for A matrix
            System.arraycopy(params, 0, result.getA().getBuffer(), 0,
                    AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS);
            // copy values for translation
            System.arraycopy(params, AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS,
                    result.getTranslation(), 0, AffineTransformation3D.NUM_TRANSLATION_COORDS);

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
     * Computes the residual between the affine transformation and a pair of
     * matched points.
     *
     * @param transformation a transformation.
     * @param inputPoint     input 3D point.
     * @param outputPoint    output 3D point.
     * @return residual.
     */
    private double residual(final AffineTransformation3D transformation, final Point3D inputPoint,
                            final Point3D outputPoint) {
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
    private double totalResidual(final AffineTransformation3D transformation) {
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
