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
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best dual conic for provided collection of 2D lines using MSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class MSACDualConicRobustEstimator extends DualConicRobustEstimator {

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
    private double mThreshold;

    /**
     * Constructor.
     */
    public MSACDualConicRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param lines 2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public MSACDualConicRobustEstimator(final List<Line2D> lines) {
        super(lines);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public MSACDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
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
    public MSACDualConicRobustEstimator(
            final DualConicRobustEstimatorListener listener,
            final List<Line2D> lines) {
        super(listener, lines);
        mThreshold = DEFAULT_THRESHOLD;
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
        return mThreshold;
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
        mThreshold = threshold;
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

        final MSACRobustEstimator<DualConic> innerEstimator =
                new MSACRobustEstimator<>(
                        new MSACRobustEstimatorListener<DualConic>() {

                            @Override
                            public double getThreshold() {
                                return mThreshold;
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
                                return MSACDualConicRobustEstimator.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(final RobustEstimator<DualConic> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            MSACDualConicRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(final RobustEstimator<DualConic> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            MSACDualConicRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<DualConic> estimator, final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            MSACDualConicRobustEstimator.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<DualConic> estimator, final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            MSACDualConicRobustEstimator.this, progress);
                                }
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
        return RobustEstimatorMethod.MSAC;
    }
}
