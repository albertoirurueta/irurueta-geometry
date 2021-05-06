/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best projective 3D transformation for provided collections of
 * matched 3D planes using LMedS algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator
        extends PlaneCorrespondenceProjectiveTransformation3DRobustEstimator {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used
     * to keep the algorithm iterating in case that best estimated threshold
     * using median of residuals is not small enough. Once a solution is found
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-6;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold;


    /**
     * Constructor.
     */
    public LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of planes to be used to estimate a projective 3D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        super(inputPlanes, outputPlanes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            final ProjectiveTransformation3DRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with listener and lists of lines to be used to estimate a
     * projective 3D transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input lines to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        super(listener, inputPlanes, outputPlanes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = stopThreshold;
    }

    /**
     * Estimates a projective 3D transformation using a robust estimator and
     * the best set of matched 3D planes correspondences found using the robust
     * estimator.
     *
     * @return a projective 3D transformation.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public ProjectiveTransformation3D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final LMedSRobustEstimator<ProjectiveTransformation3D> innerEstimator =
                new LMedSRobustEstimator<>(
                        new LMedSRobustEstimatorListener<ProjectiveTransformation3D>() {

                            // plane to be reused when computing residuals
                            private final Plane mTestPlane = new Plane();

                            @Override
                            public int getTotalSamples() {
                                return mInputPlanes.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<ProjectiveTransformation3D> solutions) {
                                final Plane inputPlane1 = mInputPlanes.get(samplesIndices[0]);
                                final Plane inputPlane2 = mInputPlanes.get(samplesIndices[1]);
                                final Plane inputPlane3 = mInputPlanes.get(samplesIndices[2]);
                                final Plane inputPlane4 = mInputPlanes.get(samplesIndices[3]);
                                final Plane inputPlane5 = mInputPlanes.get(samplesIndices[4]);

                                final Plane outputPlane1 = mOutputPlanes.get(samplesIndices[0]);
                                final Plane outputPlane2 = mOutputPlanes.get(samplesIndices[1]);
                                final Plane outputPlane3 = mOutputPlanes.get(samplesIndices[2]);
                                final Plane outputPlane4 = mOutputPlanes.get(samplesIndices[3]);
                                final Plane outputPlane5 = mOutputPlanes.get(samplesIndices[4]);

                                try {
                                    final ProjectiveTransformation3D transformation =
                                            new ProjectiveTransformation3D(inputPlane1,
                                                    inputPlane2, inputPlane3, inputPlane4, inputPlane5,
                                                    outputPlane1, outputPlane2, outputPlane3,
                                                    outputPlane4, outputPlane5);
                                    solutions.add(transformation);
                                } catch (final CoincidentPlanesException e) {
                                    // if lines are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(
                                    final ProjectiveTransformation3D currentEstimation, final int i) {
                                final Plane inputPlane = mInputPlanes.get(i);
                                final Plane outputPlane = mOutputPlanes.get(i);

                                // transform input line and store result in mTestLine
                                try {
                                    currentEstimation.transform(inputPlane, mTestPlane);

                                    return getResidual(outputPlane, mTestPlane);
                                } catch (final AlgebraException e) {
                                    // this happens when internal matrix of affine transformation
                                    // cannot be reverse (i.e. transformation is not well defined,
                                    // numerical instabilities, etc)
                                    return Double.MAX_VALUE;
                                }
                            }

                            @Override
                            public boolean isReady() {
                                return LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                                        this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<ProjectiveTransformation3D> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<ProjectiveTransformation3D> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<ProjectiveTransformation3D> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<ProjectiveTransformation3D> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.this,
                                            progress);
                                }
                            }
                        });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            innerEstimator.setStopThreshold(mStopThreshold);
            final ProjectiveTransformation3D transformation =
                    innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(transformation);
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.LMedS;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     *
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        final LMedSRobustEstimator.LMedSInliersData inliersData =
                (LMedSRobustEstimator.LMedSInliersData) getInliersData();

        // avoid setting a threshold too strict
        final double threshold = inliersData.getEstimatedThreshold();
        return Math.max(threshold, mStopThreshold);
    }
}
