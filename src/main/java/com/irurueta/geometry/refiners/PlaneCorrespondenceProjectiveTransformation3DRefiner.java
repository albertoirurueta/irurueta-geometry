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
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * A 3D projective transformation refiner using plane correspondences.
 * This class takes into account an initial estimation, inlier line matches
 * and their residuals to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
@SuppressWarnings("DuplicatedCode")
public class PlaneCorrespondenceProjectiveTransformation3DRefiner extends
        ProjectiveTransformation3DRefiner<Plane, Plane> {

    /**
     * Plane to be reused when computing residuals.
     */
    private final Plane mResidualTestPlane = new Plane();

    /**
     * Constructor.
     */
    public PlaneCorrespondenceProjectiveTransformation3DRefiner() {
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
    public PlaneCorrespondenceProjectiveTransformation3DRefiner(
            final ProjectiveTransformation3D initialEstimation, final boolean keepCovariance,
            final BitSet inliers, final double[] residuals, final int numInliers,
            final List<Plane> samples1, final List<Plane> samples2,
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
    public PlaneCorrespondenceProjectiveTransformation3DRefiner(
            final ProjectiveTransformation3D initialEstimation, final boolean keepCovariance,
            final InliersData inliersData, final List<Plane> samples1,
            final List<Plane> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1,
                samples2, refinementStandardDeviation);
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
    public boolean refine(final ProjectiveTransformation3D result)
            throws NotReadyException, LockedException, RefinerException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        mLocked = true;

        if (mListener != null) {
            mListener.onRefineStart(this, mInitialEstimation);
        }

        final double initialTotalResidual = totalResidual(mInitialEstimation);

        try {
            final double[] initParams = new double[
                    ProjectiveTransformation3D.HOM_COORDS *
                            ProjectiveTransformation3D.HOM_COORDS];
            // copy values
            System.arraycopy(mInitialEstimation.getT().getBuffer(), 0,
                    initParams, 0, initParams.length);

            // output values to be fitted/optimized will contain residuals
            final double[] y = new double[mNumInliers];
            // input values will contain 2 sets of 2D points to compute residuals
            final int nDims = 2 * Plane.PLANE_NUMBER_PARAMS;
            final Matrix x = new Matrix(mNumInliers, nDims);
            final int nSamples = mInliers.length();
            int pos = 0;
            Plane inputPlane;
            Plane outputPlane;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    // sample is inlier
                    inputPlane = mSamples1.get(i);
                    outputPlane = mSamples2.get(i);
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

                    y[pos] = mResiduals[i];
                    pos++;
                }
            }

            final LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {

                        private final Plane mInputPlane = new Plane();

                        private final Plane mOutputPlane = new Plane();

                        private final ProjectiveTransformation3D mTransformation =
                                new ProjectiveTransformation3D();

                        private final GradientEstimator mGradientEstimator =
                                new GradientEstimator(
                                        new MultiDimensionFunctionEvaluatorListener() {
                                            @Override
                                            public double evaluate(final double[] params) {
                                                // copy values
                                                System.arraycopy(params, 0,
                                                        mTransformation.getT().getBuffer(), 0,
                                                        params.length);

                                                return residual(mTransformation, mInputPlane,
                                                        mOutputPlane);
                                            }
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
                            mInputPlane.setParameters(point[0], point[1], point[2],
                                    point[3]);
                            mOutputPlane.setParameters(point[4], point[5], point[6],
                                    point[7]);

                            // copy values
                            System.arraycopy(params, 0,
                                    mTransformation.getT().getBuffer(), 0,
                                    params.length);

                            final double y = residual(mTransformation, mInputPlane,
                                    mOutputPlane);
                            mGradientEstimator.gradient(params, derivatives);

                            return y;
                        }
                    };

            final LevenbergMarquardtMultiDimensionFitter fitter =
                    new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y,
                            getRefinementStandardDeviation());

            fitter.fit();

            // obtain estimated params
            final double[] params = fitter.getA();

            // update transformation

            // copy values for A matrix
            System.arraycopy(params, 0, result.getT().getBuffer(), 0,
                    params.length);

            if (mKeepCovariance) {
                // keep covariance
                mCovariance = fitter.getCovar();
            }

            final double finalTotalResidual = totalResidual(result);
            final boolean errorDecreased = finalTotalResidual < initialTotalResidual;

            if (mListener != null) {
                mListener.onRefineEnd(this, mInitialEstimation, result,
                        errorDecreased);
            }

            return errorDecreased;

        } catch (final Exception e) {
            throw new RefinerException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Computes the residual between the projective transformation and a pair of
     * matched planes.
     *
     * @param transformation a transformation.
     * @param inputPlane     input 3D plane.
     * @param outputPlane    output 3D plane.
     * @return residual.
     */
    private double residual(final ProjectiveTransformation3D transformation,
                            final Plane inputPlane, final Plane outputPlane) {
        try {
            inputPlane.normalize();
            outputPlane.normalize();

            transformation.transform(inputPlane, mResidualTestPlane);
            return 1.0 - Math.abs(outputPlane.dotProduct(mResidualTestPlane));
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
    private double totalResidual(final ProjectiveTransformation3D transformation) {
        double result = 0.0;

        final int nSamples = mInliers.length();
        Plane inputPlane;
        Plane outputPlane;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                // sample is inlier
                inputPlane = mSamples1.get(i);
                outputPlane = mSamples2.get(i);
                inputPlane.normalize();
                outputPlane.normalize();
                result += residual(transformation, inputPlane, outputPlane);
            }
        }

        return result;
    }
}
