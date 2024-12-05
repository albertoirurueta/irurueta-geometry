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
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best dual conic for provided collection of 2D lines using PROSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROSACDualConicRobustEstimator extends DualConicRobustEstimator {

    /**
     * Constant defining default threshold to determine whether lines are
     * inliers or not.
     * Threshold is defined by the equation abs(trans(l) * dc * l) &lt; t, where
     * trans is the transposition, l is a line, dc is a dual conic and t is a
     * threshold.
     * This equation determines the lines l belonging to the locus of a dual
     * conic dC up to a certain threshold.
     */
    public static final double DEFAULT_THRESHOLD = 1e-7;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether lines are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of algebraic error a possible
     * solution has on a given line.
     */
    private double threshold;

    /**
     * Quality scores corresponding to each line.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACDualConicRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param lines 2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualConicRobustEstimator(final List<Line2D> lines) {
        super(lines);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACDualConicRobustEstimator(final DualConicRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
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
    public PROSACDualConicRobustEstimator(final DualConicRobustEstimatorListener listener, final List<Line2D> lines) {
        super(listener, lines);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with quality scores.
     *
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public PROSACDualConicRobustEstimator(final double[] qualityScores) {
        super();
        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with lines and quality scores.
     *
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualConicRobustEstimator(final List<Line2D> lines, final double[] qualityScores) {
        super(lines);

        if (qualityScores.length != lines.size()) {
            throw new IllegalArgumentException();
        }

        threshold = DEFAULT_THRESHOLD;
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
    public PROSACDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener, final double[] qualityScores) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
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
    public PROSACDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener, final List<Line2D> lines, final double[] qualityScores) {
        super(listener, lines);

        if (qualityScores.length != lines.size()) {
            throw new IllegalArgumentException();
        }

        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to determine whether lines are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a
     * given line.
     *
     * @return threshold to determine whether lines are inliers or not when
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether lines are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of algebraic error a possible
     * solution has on a given line.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     *                                  zero.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        this.threshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each provided line.
     * The larger the score value the better the quality of the sampled line.
     *
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
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
     * This is true when input data (i.e. 2D lines and quality scores) are
     * provided and a minimum of MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == lines.size();
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
    public DualConic estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<DualConic>() {

            @Override
            public double getThreshold() {
                return threshold;
            }

            @Override
            public int getTotalSamples() {
                return lines.size();
            }

            @Override
            public int getSubsetSize() {
                return DualConicRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<DualConic> solutions) {
                final var line1 = lines.get(samplesIndices[0]);
                final var line2 = lines.get(samplesIndices[1]);
                final var line3 = lines.get(samplesIndices[2]);
                final var line4 = lines.get(samplesIndices[3]);
                final var line5 = lines.get(samplesIndices[4]);

                try {
                    final var dualConic = new DualConic(line1, line2, line3, line4, line5);
                    solutions.add(dualConic);
                } catch (final CoincidentLinesException e) {
                    // if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(final DualConic currentEstimation, int i) {
                return residual(currentEstimation, lines.get(i));
            }

            @Override
            public boolean isReady() {
                return PROSACDualConicRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<DualConic> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(PROSACDualConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<DualConic> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(PROSACDualConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<DualConic> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(PROSACDualConicRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<DualConic> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(PROSACDualConicRobustEstimator.this, progress);
                }
            }

            @Override
            public double[] getQualityScores() {
                return qualityScores;
            }
        });

        try {
            locked = true;
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            return innerEstimator.estimate();
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
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

        this.qualityScores = qualityScores;
    }
}
