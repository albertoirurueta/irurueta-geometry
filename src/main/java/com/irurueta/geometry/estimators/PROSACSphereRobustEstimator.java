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
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best sphere for provided collection of 3D points using PROSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROSACSphereRobustEstimator extends SphereRobustEstimator {
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
    private double threshold;

    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACSphereRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param points 3D points to estimate a sphere.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACSphereRobustEstimator(final List<Point3D> points) {
        super(points);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACSphereRobustEstimator(final SphereRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
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
    public PROSACSphereRobustEstimator(final SphereRobustEstimatorListener listener, final List<Point3D> points) {
        super(listener, points);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 4 points).
     */
    public PROSACSphereRobustEstimator(final double[] qualityScores) {
        super();
        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points.
     *
     * @param points        3D points to estimate a sphere.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACSphereRobustEstimator(final List<Point3D> points, final double[] qualityScores) {
        super(points);

        if (qualityScores.length != points.size()) {
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
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 points).
     */
    public PROSACSphereRobustEstimator(final SphereRobustEstimatorListener listener, final double[] qualityScores) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }


    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        3D points to estimate a sphere.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACSphereRobustEstimator(
            final SphereRobustEstimatorListener listener, final List<Point3D> points, final double[] qualityScores) {
        super(listener, points);

        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }

        threshold = DEFAULT_THRESHOLD;
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
     * Returns quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     *
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
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
     * Indicates if estimator is ready to start the conic estimation.
     * This is true when input data (i.e. 2D points and quality scores) are
     * provided and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == points.size();
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
    public Sphere estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<Sphere>() {

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
                return SphereRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Sphere> solutions) {
                final var point1 = points.get(samplesIndices[0]);
                final var point2 = points.get(samplesIndices[1]);
                final var point3 = points.get(samplesIndices[2]);
                final var point4 = points.get(samplesIndices[3]);

                try {
                    final var sphere = new Sphere(point1, point2, point3, point4);
                    solutions.add(sphere);
                } catch (final CoplanarPointsException e) {
                    // if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(final Sphere currentEstimation, final int i) {
                return residual(currentEstimation, points.get(i));
            }

            @Override
            public boolean isReady() {
                return PROSACSphereRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Sphere> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(PROSACSphereRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Sphere> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(PROSACSphereRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Sphere> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(PROSACSphereRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Sphere> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(PROSACSphereRobustEstimator.this, progress);
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

        this.qualityScores = qualityScores;
    }
}
