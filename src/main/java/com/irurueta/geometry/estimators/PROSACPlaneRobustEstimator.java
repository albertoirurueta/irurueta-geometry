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

import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best 2D plane for provided collection of 3D points using PROSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROSACPlaneRobustEstimator extends PlaneRobustEstimator {
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
     * solution has on a sampled line.
     */
    private double mThreshold;

    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROSACPlaneRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param points 3D points to estimate a 3D plane.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACPlaneRobustEstimator(final List<Point3D> points) {
        super(points);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACPlaneRobustEstimator(final PlaneRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   3D points to estimate a 3D plane.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACPlaneRobustEstimator(final PlaneRobustEstimatorListener listener,
                                      final List<Point3D> points) {
        super(listener, points);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 points).
     */
    public PROSACPlaneRobustEstimator(final double[] qualityScores) {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points.
     *
     * @param points        3D points to estimate a 3D plane.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACPlaneRobustEstimator(final List<Point3D> points,
                                      final double[] qualityScores) {
        super(points);

        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }

        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 points).
     */
    public PROSACPlaneRobustEstimator(final PlaneRobustEstimatorListener listener,
                                      final double[] qualityScores) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }


    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        3D points to estimate a 3D plane.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACPlaneRobustEstimator(final PlaneRobustEstimatorListener listener,
                                      final List<Point3D> points, final double[] qualityScores) {
        super(listener, points);

        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }

        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
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
     * Returns quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     *
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     *
     * @param qualityScores quality scores corresponding to each point.
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
     * Indicates if estimator is ready to start the 2D line estimation.
     * This is true when input data (i.e. 2D points and quality scores) are
     * provided and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mPoints.size();
    }

    /**
     * Estimates a 3D plane using a robust estimator and the best set of 3D
     * points that pass through the estimated 3D plane (i.e. belong to its
     * locus).
     *
     * @return a 3D plane.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Plane estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROSACRobustEstimator<Plane> innerEstimator =
                new PROSACRobustEstimator<>(
                        new PROSACRobustEstimatorListener<Plane>() {

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
                                return PlaneRobustEstimator.MINIMUM_SIZE;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<Plane> solutions) {
                                final Point3D point1 = mPoints.get(samplesIndices[0]);
                                final Point3D point2 = mPoints.get(samplesIndices[1]);
                                final Point3D point3 = mPoints.get(samplesIndices[2]);

                                try {
                                    final Plane plane = new Plane(point1, point2, point3);
                                    solutions.add(plane);
                                } catch (final ColinearPointsException e) {
                                    //if points are coincident, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(final Plane currentEstimation, int i) {
                                return residual(currentEstimation, mPoints.get(i));
                            }

                            @Override
                            public boolean isReady() {
                                return PROSACPlaneRobustEstimator.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(final RobustEstimator<Plane> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(PROSACPlaneRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(final RobustEstimator<Plane> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(PROSACPlaneRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<Plane> estimator, final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            PROSACPlaneRobustEstimator.this, iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<Plane> estimator, final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            PROSACPlaneRobustEstimator.this, progress);
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
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each provided point.
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
