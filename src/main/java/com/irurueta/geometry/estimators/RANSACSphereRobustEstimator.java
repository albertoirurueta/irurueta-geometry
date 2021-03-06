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

import com.irurueta.geometry.CoplanarPointsException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best sphere for provided collection of 3D points using RANSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class RANSACSphereRobustEstimator extends SphereRobustEstimator {
    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * Because typical resolution for points is 1 voxel, then default threshold
     * is defined as 1.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;

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
    public RANSACSphereRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param points 3D points to estimate a sphere.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public RANSACSphereRobustEstimator(final List<Point3D> points) {
        super(points);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RANSACSphereRobustEstimator(final SphereRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   3D points to estimate a sphere.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public RANSACSphereRobustEstimator(final SphereRobustEstimatorListener listener,
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
     * Estimates a sphere using a robust estimator and the best set of 3D points
     * that fit into the locus of the estimated sphere found using the robust
     * estimator.
     *
     * @return a sphere.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Sphere estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final RANSACRobustEstimator<Sphere> innerEstimator =
                new RANSACRobustEstimator<>(
                        new RANSACRobustEstimatorListener<Sphere>() {

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
                                return SphereRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<Sphere> solutions) {
                                final Point3D point1 = mPoints.get(samplesIndices[0]);
                                final Point3D point2 = mPoints.get(samplesIndices[1]);
                                final Point3D point3 = mPoints.get(samplesIndices[2]);
                                final Point3D point4 = mPoints.get(samplesIndices[3]);

                                try {
                                    final Sphere sphere = new Sphere(point1, point2, point3, point4);
                                    solutions.add(sphere);
                                } catch (final CoplanarPointsException e) {
                                    // if points are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(final Sphere currentEstimation, final int i) {
                                return residual(currentEstimation, mPoints.get(i));
                            }

                            @Override
                            public boolean isReady() {
                                return RANSACSphereRobustEstimator.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(final RobustEstimator<Sphere> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(RANSACSphereRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(final RobustEstimator<Sphere> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(RANSACSphereRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<Sphere> estimator, final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            RANSACSphereRobustEstimator.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<Sphere> estimator, final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            RANSACSphereRobustEstimator.this, progress);
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
