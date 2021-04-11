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
import com.irurueta.geometry.CoincidentLinesException;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best projective 2D transformation for provided collections of
 * matched 2D lines using PROMedS algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator
        extends LineCorrespondenceProjectiveTransformation2DRobustEstimator {

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
     * Quality scores corresponding to each pair of matched lines.
     * The larger the score value the betther the quality of the matching
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of lines to be used to estimate a projective 2D
     * transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greter or equal than MINIMUM_SIZE.
     *
     * @param inputLines  list of input lines to be used to estimate a projective
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate a
     *                    projective 2D transformation.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final List<Line2D> inputLines, final List<Line2D> outputLines) {
        super(inputLines, outputLines);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final ProjectiveTransformation2DRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with listener and lists of lines to be used to estimate a
     * projective 2D transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener    lsitener to be notified of events such as when estimation
     *                    starts, ends or its progress significantly changes.
     * @param inputLines  list of input lines to be used to estimate a projective
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate a
     *                    projective 2D transformation.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final ProjectiveTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines) {
        super(listener, inputLines, outputLines);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final double[] qualityScores) {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with lists of lines to be used to estimate a projective 2D
     * transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greter or equal than MINIMUM_SIZE.
     *
     * @param inputLines    list of input lines to be used to estimate a projective
     *                      2D transformation.
     * @param outputLines   list of output lines to be used to estimate a
     *                      projective 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @throws IllegalArgumentException if provided lists of lines and array
     *                                  of quality scores don't have the same size or their size is smaller than
     *                                  MINIMUM_SIZE.
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final double[] qualityScores) {
        super(inputLines, outputLines);

        if (qualityScores.length != inputLines.size()) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final ProjectiveTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with listener and lists of lines to be used to estimate a
     * projective 2D transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputLines    list of input lines to be used to estimate a projective
     *                      2D transformation.
     * @param outputLines   list of output lines to be used to estimate a
     *                      projective 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator(
            final ProjectiveTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final double[] qualityScores) {
        super(listener, inputLines, outputLines);

        if (qualityScores.length != inputLines.size()) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
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
     * Returns quality scores corresponding to each pair of matched lines.
     * The larger the score value the betther the quality of the matching.
     *
     * @return quality scores corresponding to each pair of matched lines.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each pair of matched lines.
     * The larger the score value the better the quality of the matching.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if estimator is ready to start the affine 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched lines and quality
     * scores) are provided and a minimum of MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mInputLines.size();
    }

    /**
     * Estimates a projective 2D transformation using a robust estimator and
     * the best set of matched 2D lines correspondences found using the robust
     * estimator.
     *
     * @return a projective 2D transformation.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public ProjectiveTransformation2D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROMedSRobustEstimator<ProjectiveTransformation2D> innerEstimator =
                new PROMedSRobustEstimator<>(
                        new PROMedSRobustEstimatorListener<ProjectiveTransformation2D>() {

                            // line to be reused when computing residuals
                            private final Line2D mTestLine = new Line2D();

                            @Override
                            public double getThreshold() {
                                return mStopThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mInputLines.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<ProjectiveTransformation2D> solutions) {
                                final Line2D inputLine1 = mInputLines.get(samplesIndices[0]);
                                final Line2D inputLine2 = mInputLines.get(samplesIndices[1]);
                                final Line2D inputLine3 = mInputLines.get(samplesIndices[2]);
                                final Line2D inputLine4 = mInputLines.get(samplesIndices[3]);

                                final Line2D outputLine1 = mOutputLines.get(samplesIndices[0]);
                                final Line2D outputLine2 = mOutputLines.get(samplesIndices[1]);
                                final Line2D outputLine3 = mOutputLines.get(samplesIndices[2]);
                                final Line2D outputLine4 = mOutputLines.get(samplesIndices[3]);

                                try {
                                    final ProjectiveTransformation2D transformation =
                                            new ProjectiveTransformation2D(inputLine1,
                                                    inputLine2, inputLine3, inputLine4, outputLine1,
                                                    outputLine2, outputLine3, outputLine4);
                                    solutions.add(transformation);
                                } catch (final CoincidentLinesException e) {
                                    // if lines are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(final ProjectiveTransformation2D currentEstimation,
                                                          final int i) {
                                final Line2D inputLine = mInputLines.get(i);
                                final Line2D outputLine = mOutputLines.get(i);

                                // transform input line and store result in mTestLine
                                try {
                                    currentEstimation.transform(inputLine, mTestLine);

                                    return getResidual(outputLine, mTestLine);
                                } catch (final AlgebraException e) {
                                    // this happens when internal matrix of affine transformation
                                    // cannot be reverse (i.e. transformation is not well defined,
                                    // numerical instabilities, etc)
                                    return Double.MAX_VALUE;
                                }
                            }

                            @Override
                            public boolean isReady() {
                                return PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.
                                        this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<ProjectiveTransformation2D> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<ProjectiveTransformation2D> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<ProjectiveTransformation2D> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<ProjectiveTransformation2D> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            PROMedSLineCorrespondenceProjectiveTransformation2DRobustEstimator.this,
                                            progress);
                                }
                            }

                            @Override
                            public double[] getQualityScores() {
                                return mQualityScores;
                            }
                        });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final ProjectiveTransformation2D transformation =
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
        return RobustEstimatorMethod.PROMedS;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        final PROMedSRobustEstimator.PROMedSInliersData inliersData =
                (PROMedSRobustEstimator.PROMedSInliersData) getInliersData();

        // avoid setting a threshold too strict
        final double threshold = inliersData.getEstimatedThreshold();
        return Math.max(threshold, mStopThreshold);
    }

    /**
     * Sets quality scores corresponding to each pair of matched lines.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
