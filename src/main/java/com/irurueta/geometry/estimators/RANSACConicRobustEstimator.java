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

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.Conic;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best conic for provided collection of 2D points using RANSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class RANSACConicRobustEstimator extends ConicRobustEstimator {

    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * Threshold is defined by the equation abs(trans(x) * C * x) &lt; t, where
     * trans is the transposition, x is a point, C is a conic and t is a
     * threshold.
     * This equation determines the points x belonging to the locus of a conic
     * C up to a certain threshold.
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible
     * solution has on a matched pair of points.
     */
    private double threshold;

    /**
     * Constructor.
     */
    public RANSACConicRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param points 2D points to estimate a conic.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public RANSACConicRobustEstimator(final List<Point2D> points) {
        super(points);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RANSACConicRobustEstimator(final ConicRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   2D points to estimate a conic.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public RANSACConicRobustEstimator(final ConicRobustEstimatorListener listener, final List<Point2D> points) {
        super(listener, points);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Returns threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a
     * given point.
     *
     * @return threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on
     * a given point.
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
     * Estimates a conic using a robust estimator and the best set of 2D points
     * that fit into the locus of the estimated conic found using the robust
     * estimator.
     *
     * @return a conic.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Conic estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new RANSACRobustEstimator<>(new RANSACRobustEstimatorListener<Conic>() {

                    @Override
                    public double getThreshold() {
                        return threshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return points.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return ConicRobustEstimator.MINIMUM_SIZE;
                    }

                    @Override
                    public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Conic> solutions) {
                        final var point1 = points.get(samplesIndices[0]);
                        final var point2 = points.get(samplesIndices[1]);
                        final var point3 = points.get(samplesIndices[2]);
                        final var point4 = points.get(samplesIndices[3]);
                        final var point5 = points.get(samplesIndices[4]);

                        try {
                            final var conic = new Conic(point1, point2, point3, point4, point5);
                            solutions.add(conic);
                        } catch (final CoincidentPointsException e) {
                            // if points are coincident, no solution is added
                        }
                    }

                    @Override
                    public double computeResidual(final Conic currentEstimation, final int i) {
                        return residual(currentEstimation, points.get(i));
                    }

                    @Override
                    public boolean isReady() {
                        return RANSACConicRobustEstimator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<Conic> estimator) {
                        if (listener != null) {
                            listener.onEstimateStart(RANSACConicRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<Conic> estimator) {
                        if (listener != null) {
                            listener.onEstimateEnd(RANSACConicRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(final RobustEstimator<Conic> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onEstimateNextIteration(RANSACConicRobustEstimator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(final RobustEstimator<Conic> estimator, final float progress) {
                        if (listener != null) {
                            listener.onEstimateProgressChange(RANSACConicRobustEstimator.this, progress);
                        }
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
        return RobustEstimatorMethod.RANSAC;
    }
}
