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
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quadric;
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best quadric for provided collection of 3D points using RANSAC
 * algorithm.
 */
public class RANSACQuadricRobustEstimator extends QuadricRobustEstimator {
    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * Threshold is defined by the equation abs(trans(X)) * Q * X) &lt; t, where
     * trans is the transposition, X is a point, Q is a quadric and t is a
     * threshold.
     * This equation determines the points X belonging to the locus of a quadric
     * Q up to a certain threshold.
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
    private double mThreshold;

    /**
     * Constructor.
     */
    public RANSACQuadricRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param points 3D points to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public RANSACQuadricRobustEstimator(final List<Point3D> points) {
        super(points);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RANSACQuadricRobustEstimator(
            final QuadricRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   3D points to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public RANSACQuadricRobustEstimator(final QuadricRobustEstimatorListener listener,
                                        final List<Point3D> points) {
        super(listener, points);
        mThreshold = DEFAULT_THRESHOLD;
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
        return mThreshold;
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
        mThreshold = threshold;
    }


    /**
     * Estimates a quadric using a robust estimator and the best set of 3D
     * points that fit into the locus of the estimated quadric found using the
     * robust estimator.
     *
     * @return a quadric.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public Quadric estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final RANSACRobustEstimator<Quadric> innerEstimator =
                new RANSACRobustEstimator<>(
                        new RANSACRobustEstimatorListener<Quadric>() {

                            @Override
                            public double getThreshold() {
                                return mThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mPoints.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return QuadricRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<Quadric> solutions) {
                                final Point3D point1 = mPoints.get(samplesIndices[0]);
                                final Point3D point2 = mPoints.get(samplesIndices[1]);
                                final Point3D point3 = mPoints.get(samplesIndices[2]);
                                final Point3D point4 = mPoints.get(samplesIndices[3]);
                                final Point3D point5 = mPoints.get(samplesIndices[4]);
                                final Point3D point6 = mPoints.get(samplesIndices[5]);
                                final Point3D point7 = mPoints.get(samplesIndices[6]);
                                final Point3D point8 = mPoints.get(samplesIndices[7]);
                                final Point3D point9 = mPoints.get(samplesIndices[8]);

                                try {
                                    final Quadric quadric = new Quadric(point1, point2, point3,
                                            point4, point5, point6, point7, point8, point9);
                                    solutions.add(quadric);
                                } catch (final CoincidentPointsException e) {
                                    // if points are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(final Quadric currentEstimation, final int i) {
                                return residual(currentEstimation, mPoints.get(i));
                            }

                            @Override
                            public boolean isReady() {
                                return RANSACQuadricRobustEstimator.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(final RobustEstimator<Quadric> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(RANSACQuadricRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(final RobustEstimator<Quadric> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(RANSACQuadricRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<Quadric> estimator, final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            RANSACQuadricRobustEstimator.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<Quadric> estimator, final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            RANSACQuadricRobustEstimator.this, progress);
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
        return RobustEstimatorMethod.RANSAC;
    }
}
