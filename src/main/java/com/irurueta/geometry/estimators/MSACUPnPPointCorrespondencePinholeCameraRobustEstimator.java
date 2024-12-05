/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best pinhole camera for provided collections of matched 2D/3D
 * points using MSAC + UPnP algorithms.
 */
@SuppressWarnings("DuplicatedCode")
public class MSACUPnPPointCorrespondencePinholeCameraRobustEstimator extends
        UPnPPointCorrespondencePinholeCameraRobustEstimator {

    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * By default, 1.0 is considered a good value for cases where measures are
     * done on pixels, since typically the minimum resolution is 1 pixel.
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
     * Constructor.
     */
    public MSACUPnPPointCorrespondencePinholeCameraRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size (6
     *                                  correspondences).
     */
    public MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(points3D, points2D);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with listener and lists of points to be used to estimate a
     * pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size (6
     *                                  correspondences).
     */
    public MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(listener, points3D, points2D);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Returns threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. Euclidean distance) a
     * possible solution has on projected 2D points.
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
     * The threshold refers to the amount of error (i.e. Euclidean distance) a
     * possible solution has on projected 2D points.
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
     * Estimates a pinhole camera using a robust estimator and
     * the best set of matched 2D/3D point correspondences or 2D line/3D plane
     * correspondences found using the robust estimator.
     *
     * @return a pinhole camera.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public PinholeCamera estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        // pinhole camera estimator using UPnP (Uncalibrated Perspective-n-Point) algorithm
        final var nonRobustEstimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        nonRobustEstimator.setPlanarConfigurationAllowed(planarConfigurationAllowed);
        nonRobustEstimator.setNullspaceDimension2Allowed(nullspaceDimension2Allowed);
        nonRobustEstimator.setPlanarThreshold(planarThreshold);
        nonRobustEstimator.setSkewness(skewness);
        nonRobustEstimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
        nonRobustEstimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

        // suggestions
        nonRobustEstimator.setSuggestSkewnessValueEnabled(isSuggestSkewnessValueEnabled());
        nonRobustEstimator.setSuggestedSkewnessValue(getSuggestedSkewnessValue());
        nonRobustEstimator.setSuggestHorizontalFocalLengthEnabled(isSuggestHorizontalFocalLengthEnabled());
        nonRobustEstimator.setSuggestedHorizontalFocalLengthValue(getSuggestedHorizontalFocalLengthValue());
        nonRobustEstimator.setSuggestVerticalFocalLengthEnabled(isSuggestVerticalFocalLengthEnabled());
        nonRobustEstimator.setSuggestedVerticalFocalLengthValue(getSuggestedVerticalFocalLengthValue());
        nonRobustEstimator.setSuggestAspectRatioEnabled(isSuggestAspectRatioEnabled());
        nonRobustEstimator.setSuggestedAspectRatioValue(getSuggestedAspectRatioValue());
        nonRobustEstimator.setSuggestPrincipalPointEnabled(isSuggestPrincipalPointEnabled());
        nonRobustEstimator.setSuggestedPrincipalPointValue(getSuggestedPrincipalPointValue());
        nonRobustEstimator.setSuggestRotationEnabled(isSuggestRotationEnabled());
        nonRobustEstimator.setSuggestedRotationValue(getSuggestedRotationValue());
        nonRobustEstimator.setSuggestCenterEnabled(isSuggestCenterEnabled());
        nonRobustEstimator.setSuggestedCenterValue(getSuggestedCenterValue());

        final var innerEstimator = new MSACRobustEstimator<>(new MSACRobustEstimatorListener<PinholeCamera>() {

            // point to be reused when computing residuals
            private final Point2D testPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

            // 3D points for a subset of samples
            private final List<Point3D> subset3D = new ArrayList<>();

            // 2D points for a subset of samples
            private final List<Point2D> subset2D = new ArrayList<>();

            @Override
            public double getThreshold() {
                return threshold;
            }

            @Override
            public int getTotalSamples() {
                return points3D.size();
            }

            @Override
            public int getSubsetSize() {
                return PointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<PinholeCamera> solutions) {
                subset3D.clear();
                subset3D.add(points3D.get(samplesIndices[0]));
                subset3D.add(points3D.get(samplesIndices[1]));
                subset3D.add(points3D.get(samplesIndices[2]));
                subset3D.add(points3D.get(samplesIndices[3]));
                subset3D.add(points3D.get(samplesIndices[4]));
                subset3D.add(points3D.get(samplesIndices[5]));

                subset2D.clear();
                subset2D.add(points2D.get(samplesIndices[0]));
                subset2D.add(points2D.get(samplesIndices[1]));
                subset2D.add(points2D.get(samplesIndices[2]));
                subset2D.add(points2D.get(samplesIndices[3]));
                subset2D.add(points2D.get(samplesIndices[4]));
                subset2D.add(points2D.get(samplesIndices[5]));

                try {
                    nonRobustEstimator.setLists(subset3D, subset2D);

                    final var cam = nonRobustEstimator.estimate();
                    solutions.add(cam);
                } catch (final Exception e) {
                    // if points configuration is degenerate, no solution is
                    // added
                }
            }

            @Override
            public double computeResidual(final PinholeCamera currentEstimation, final int i) {
                // pick i-th points
                final var point3D = points3D.get(i);
                final var point2D = points2D.get(i);

                // project point3D into test point
                currentEstimation.project(point3D, testPoint);

                // compare test point and 2D point
                return testPoint.distanceTo(point2D);
            }

            @Override
            public boolean isReady() {
                return MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<PinholeCamera> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<PinholeCamera> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<PinholeCamera> estimator, int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(
                            MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<PinholeCamera> estimator, float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(
                            MSACUPnPPointCorrespondencePinholeCameraRobustEstimator.this, progress);
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
            return attemptRefine(result, nonRobustEstimator.getMaxSuggestionWeight());
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
