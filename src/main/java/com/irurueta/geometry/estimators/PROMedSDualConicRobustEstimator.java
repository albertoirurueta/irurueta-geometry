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

import com.irurueta.geometry.CoincidentLinesException;
import com.irurueta.geometry.DualConic;
import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best conic for provided collection of 2D lines using PROMedS
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROMedSDualConicRobustEstimator extends DualConicRobustEstimator {
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
    public static final double DEFAULT_STOP_THRESHOLD = 1e-9;

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
     * Quality scores corresponding to each 2D line.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSDualConicRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param lines 2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROMedSDualConicRobustEstimator(final List<Line2D> lines) {
        super(lines);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROMedSDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener,
            final List<Line2D> lines) {
        super(listener, lines);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public PROMedSDualConicRobustEstimator(final double[] qualityScores) {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points.
     *
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROMedSDualConicRobustEstimator(final List<Line2D> lines,
                                           final double[] qualityScores) {
        super(lines);

        if (qualityScores.length != lines.size()) {
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
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public PROMedSDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener, final double[] qualityScores) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }


    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROMedSDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener,
            final List<Line2D> lines, final double[] qualityScores) {
        super(listener, lines);

        if (qualityScores.length != lines.size()) {
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
     * As in LMedS, the stop threshold can be used to prevent the PROMedS
     * algorithm iterating too many times in cases where samples have a very
     * similar accuracy.
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
     * As in LMedS, the stop threshold can be used to prevent the PROMedS
     * algorithm iterating too many times in cases where samples have a very
     * similar accuracy.
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
     * Returns quality scores corresponding to each provided line.
     * The larger the score value the better the quality of the sampled line.
     *
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided line.
     * The larger the score value the better the quality of the sampled line.
     *
     * @param qualityScores quality scores corresponding to each line.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 samples).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if estimator is ready to start the conic estimation.
     * This is true when input data (i.e. 2D points and quality scores) are
     * provided and a minimum of MINIMUM_SIZE lines are available
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mLines.size();
    }

    /**
     * Estimates a dual conic using a robust estimator and the best set of 2D
     * lines that fit into the locus of the estimated dual conic found using the
     * robust estimator.
     *
     * @return a dual conic.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public DualConic estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROMedSRobustEstimator<DualConic> innerEstimator =
                new PROMedSRobustEstimator<>(
                        new PROMedSRobustEstimatorListener<DualConic>() {

                            @Override
                            public double getThreshold() {
                                return mStopThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mLines.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return DualConicRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<DualConic> solutions) {
                                final Line2D line1 = mLines.get(samplesIndices[0]);
                                final Line2D line2 = mLines.get(samplesIndices[1]);
                                final Line2D line3 = mLines.get(samplesIndices[2]);
                                final Line2D line4 = mLines.get(samplesIndices[3]);
                                final Line2D line5 = mLines.get(samplesIndices[4]);

                                try {
                                    final DualConic dualConic = new DualConic(line1, line2, line3,
                                            line4, line5);
                                    solutions.add(dualConic);
                                } catch (final CoincidentLinesException e) {
                                    // if points are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(final DualConic currentEstimation, final int i) {
                                return residual(currentEstimation, mLines.get(i));
                            }

                            @Override
                            public boolean isReady() {
                                return PROMedSDualConicRobustEstimator.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(final RobustEstimator<DualConic> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            PROMedSDualConicRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(final RobustEstimator<DualConic> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            PROMedSDualConicRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<DualConic> estimator, final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            PROMedSDualConicRobustEstimator.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<DualConic> estimator, final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            PROMedSDualConicRobustEstimator.this, progress);
                                }
                            }

                            @Override
                            public double[] getQualityScores() {
                                return mQualityScores;
                            }
                        });

        try {
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            return innerEstimator.estimate();
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
        return RobustEstimatorMethod.PROMEDS;
    }

    /**
     * Sets quality scores corresponding to each provided line.
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
