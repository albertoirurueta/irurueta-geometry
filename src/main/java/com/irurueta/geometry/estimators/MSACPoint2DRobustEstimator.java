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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.NoIntersectionException;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best 2D point for provided collection of 2D lines using MSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class MSACPoint2DRobustEstimator extends Point2DRobustEstimator {

    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * Because typical resolution for points is 1 pixel, then default threshold
     * is defined as 1.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether lines are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible
     * solution has on a sampled line.
     */
    private double threshold;

    /**
     * Constructor.
     */
    public MSACPoint2DRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with lines.
     *
     * @param lines 2D lines to estimate a 2D point.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public MSACPoint2DRobustEstimator(final List<Line2D> lines) {
        super(lines);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public MSACPoint2DRobustEstimator(final Point2DRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a 2D point.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public MSACPoint2DRobustEstimator(final Point2DRobustEstimatorListener listener, final List<Line2D> lines) {
        super(listener, lines);
        threshold = DEFAULT_THRESHOLD;
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
     * The threshold refers to the amount of error a possible solution has on
     * a given line.
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
     * Estimates a 2D point using a robust estimator and the best set of 2D
     * lines that intersect into the estimated 2D point.
     *
     * @return a 2D point.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Point2D estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new MSACRobustEstimator<>(new MSACRobustEstimatorListener<Point2D>() {

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
                return Point2DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Point2D> solutions) {
                final var line1 = lines.get(samplesIndices[0]);
                final var line2 = lines.get(samplesIndices[1]);

                try {
                    final var point = line1.getIntersection(line2);
                    solutions.add(point);
                } catch (final NoIntersectionException e) {
                    // if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(final Point2D currentEstimation, final int i) {
                return residual(currentEstimation, lines.get(i));
            }

            @Override
            public boolean isReady() {
                return MSACPoint2DRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Point2D> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(MSACPoint2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Point2D> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(MSACPoint2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Point2D> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(MSACPoint2DRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Point2D> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(MSACPoint2DRobustEstimator.this, progress);
                }
            }
        });

        try {
            locked = true;
            inliersData = null;
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            final var result = innerEstimator.estimate();
            inliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
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
        return RobustEstimatorMethod.MSAC;
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
        return threshold;
    }
}
