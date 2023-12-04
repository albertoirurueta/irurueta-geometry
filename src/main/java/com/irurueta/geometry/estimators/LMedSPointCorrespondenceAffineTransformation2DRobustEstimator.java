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

import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best affine 2D transformation for provided collections of matched
 * 2D points using LMedS algorithm.
 */
public class LMedSPointCorrespondenceAffineTransformation2DRobustEstimator
        extends PointCorrespondenceAffineTransformation2DRobustEstimator {

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
    public static final double DEFAULT_STOP_THRESHOLD = 1.0;

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
    public LMedSPointCorrespondenceAffineTransformation2DRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of points to be used to estimate an affine 2D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     affine 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     affine 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        super(inputPoints, outputPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(
            final AffineTransformation2DRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with listener and lists of points to be used to estimate an
     * affine 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate an
     *                     affine 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     affine 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public LMedSPointCorrespondenceAffineTransformation2DRobustEstimator(
            final AffineTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        super(listener, inputPoints, outputPoints);
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
     * Estimates an affine 2D transformation using a robust estimator and
     * the best set of matched 2D point correspondences found using the robust
     * estimator.
     *
     * @return an affine 2D transformation.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public AffineTransformation2D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final LMedSRobustEstimator<AffineTransformation2D> innerEstimator =
                new LMedSRobustEstimator<>(
                        new LMedSRobustEstimatorListener<AffineTransformation2D>() {

                            // point to be reused when computing residuals
                            private final Point2D mTestPoint = Point2D.create(
                                    CoordinatesType.HOMOGENEOUS_COORDINATES);

                            @Override
                            public int getTotalSamples() {
                                return mInputPoints.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return AffineTransformation2DRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<AffineTransformation2D> solutions) {
                                final Point2D inputPoint1 = mInputPoints.get(samplesIndices[0]);
                                final Point2D inputPoint2 = mInputPoints.get(samplesIndices[1]);
                                final Point2D inputPoint3 = mInputPoints.get(samplesIndices[2]);

                                final Point2D outputPoint1 = mOutputPoints.get(samplesIndices[0]);
                                final Point2D outputPoint2 = mOutputPoints.get(samplesIndices[1]);
                                final Point2D outputPoint3 = mOutputPoints.get(samplesIndices[2]);

                                try {
                                    final AffineTransformation2D transformation =
                                            new AffineTransformation2D(inputPoint1, inputPoint2,
                                                    inputPoint3, outputPoint1, outputPoint2, outputPoint3);
                                    solutions.add(transformation);
                                } catch (final CoincidentPointsException e) {
                                    // if points are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(
                                    final AffineTransformation2D currentEstimation, final int i) {
                                final Point2D inputPoint = mInputPoints.get(i);
                                final Point2D outputPoint = mOutputPoints.get(i);

                                // transform input point and store result in mTestPoint
                                currentEstimation.transform(inputPoint, mTestPoint);

                                return outputPoint.distanceTo(mTestPoint);
                            }

                            @Override
                            public boolean isReady() {
                                return LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.
                                        this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<AffineTransformation2D> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<AffineTransformation2D> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<AffineTransformation2D> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<AffineTransformation2D> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            LMedSPointCorrespondenceAffineTransformation2DRobustEstimator.this,
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
            final AffineTransformation2D transformation = innerEstimator.estimate();
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
        return RobustEstimatorMethod.LMEDS;
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
    @Override
    protected double getRefinementStandardDeviation() {
        final LMedSRobustEstimator.LMedSInliersData inliersData =
                (LMedSRobustEstimator.LMedSInliersData) getInliersData();

        // avoid setting a threshold too strict
        final double threshold = inliersData.getEstimatedThreshold();
        return Math.max(threshold, mStopThreshold);
    }
}
