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
import com.irurueta.geometry.AffineTransformation3D;
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
 * A 3D affine transformation refiner using plane correspondences.
 * This class takes into account an initial estimation, inlier line matches
 * and their residuals to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class PlaneCorrespondenceAffineTransformation3DRefiner extends AffineTransformation3DRefiner<Plane, Plane> {

    /**
     * Plane to be reused when computing residuals.
     */
    private final Plane residualTestPlane = new Plane();

    /**
     * Constructor.
     */
    public PlaneCorrespondenceAffineTransformation3DRefiner() {
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
    public PlaneCorrespondenceAffineTransformation3DRefiner(
            final AffineTransformation3D initialEstimation, final boolean keepCovariance,
            final BitSet inliers, final double[] residuals, final int numInliers, final List<Plane> samples1,
            final List<Plane> samples2, final double refinementStandardDeviation) {
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
    public PlaneCorrespondenceAffineTransformation3DRefiner(
            final AffineTransformation3D initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<Plane> samples1, final List<Plane> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, samples2, refinementStandardDeviation);
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
            final var nDims = 2 * Plane.PLANE_NUMBER_PARAMS;
            final var x = new Matrix(numInliers, nDims);
            final var nSamples = inliers.length();
            var pos = 0;
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    final var inputPlane = samples1.get(i);
                    final var outputPlane = samples2.get(i);
                    inputPlane.normalize();
                    outputPlane.normalize();
                    x.setElementAt(pos, 0, inputPlane.getA());
                    x.setElementAt(pos, 1, inputPlane.getB());
                    x.setElementAt(pos, 2, inputPlane.getC());
                    x.setElementAt(pos, 3, inputPlane.getD());
                    x.setElementAt(pos, 4, outputPlane.getA());
                    x.setElementAt(pos, 5, outputPlane.getB());
                    x.setElementAt(pos, 6, outputPlane.getC());
                    x.setElementAt(pos, 7, outputPlane.getD());

                    y[pos] = residuals[i];
                    pos++;
                }
            }

            final var evaluator = new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

                private final Plane inputPlane = new Plane();

                private final Plane outputPlane = new Plane();

                private final AffineTransformation3D transformation = new AffineTransformation3D();

                private final GradientEstimator gradientEstimator = new GradientEstimator(params -> {
                    // copy values for A matrix
                    System.arraycopy(params, 0, transformation.getA().getBuffer(), 0,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS);
                    // copy values for translation
                    System.arraycopy(params,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS,
                            transformation.getTranslation(), 0, AffineTransformation3D.NUM_TRANSLATION_COORDS);

                    return residual(transformation, inputPlane, outputPlane);
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
                    inputPlane.setParameters(point[0], point[1], point[2], point[3]);
                    outputPlane.setParameters(point[4], point[5], point[6], point[7]);

                    // copy values for A matrix
                    System.arraycopy(params, 0, transformation.getA().getBuffer(), 0,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS);
                    // copy values for translation
                    System.arraycopy(params,
                            AffineTransformation3D.INHOM_COORDS * AffineTransformation3D.INHOM_COORDS,
                            transformation.getTranslation(), 0, AffineTransformation3D.NUM_TRANSLATION_COORDS);

                    final var y = residual(transformation, inputPlane, outputPlane);
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
     * matched planes.
     *
     * @param transformation a transformation.
     * @param inputPlane     input 3D plane.
     * @param outputPlane    output 3D plane.
     * @return residual.
     */
    private double residual(final AffineTransformation3D transformation, final Plane inputPlane,
                            final Plane outputPlane) {
        try {
            inputPlane.normalize();
            outputPlane.normalize();

            transformation.transform(inputPlane, residualTestPlane);
            return 1.0 - Math.abs(outputPlane.dotProduct(residualTestPlane));
        } catch (final AlgebraException e) {
            return 1.0;
        }
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
                final var inputPlane = samples1.get(i);
                final var outputPlane = samples2.get(i);
                inputPlane.normalize();
                outputPlane.normalize();
                result += residual(transformation, inputPlane, outputPlane);
            }
        }

        return result;
    }
}
