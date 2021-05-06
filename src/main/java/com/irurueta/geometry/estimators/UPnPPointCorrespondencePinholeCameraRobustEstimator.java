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

import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Base abstract class for algorithms to robustly find the best pinhole camera
 * for collections of matched 3D/2D points using UPnP (Uncalibrated
 * Perspective-n-Point) algorithm.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class UPnPPointCorrespondencePinholeCameraRobustEstimator extends
        PointCorrespondencePinholeCameraRobustEstimator {

    /**
     * Indicates whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     */
    protected boolean mPlanarConfigurationAllowed =
            UPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_PLANAR_CONFIGURATION_ALLOWED;

    /**
     * Indicates whether the case where a dimension 2 null-space is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     */
    protected boolean mNullspaceDimension2Allowed =
            UPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_NULLSPACE_DIMENSION2_ALLOWED;

    /**
     * Threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     */
    protected double mPlanarThreshold =
            UPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_PLANAR_THRESHOLD;

    /**
     * Skewness value of intrinsic parameters to be used on estimated camera.
     */
    protected double mSkewness = UPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_SKEWNESS;

    /**
     * Horizontal coordinate of principal point on intrinsic parameters to be
     * used on estimated camera.
     */
    protected double mHorizontalPrincipalPoint =
            UPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_HORIZONTAL_PRINCIPAL_POINT;

    /**
     * Vertical coordinate of principal point on intrinsic parameters to be used
     * on estimated camera.
     */
    protected double mVerticalPrincipalPoint =
            UPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_VERTICAL_PRINCIPAL_POINT;

    /**
     * Constructor.
     */
    protected UPnPPointCorrespondencePinholeCameraRobustEstimator() {
        super();
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected UPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size (6
     *                                  correspondences).
     */
    protected UPnPPointCorrespondencePinholeCameraRobustEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(points3D, points2D);
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
    protected UPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(listener, points3D, points2D);
    }

    /**
     * Indicates whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     *
     * @return true to allow specific solutions for planar configurations,
     * false to always find a solution assuming the general case.
     */
    public boolean isPlanarConfigurationAllowed() {
        return mPlanarConfigurationAllowed;
    }

    /**
     * Specifies whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     *
     * @param planarConfigurationAllowed true to allow specific solutions for
     *                                   planar configurations, false to always find a solution assuming the
     *                                   general case.
     * @throws LockedException if estimator is locked.
     */
    public void setPlanarConfigurationAllowed(
            final boolean planarConfigurationAllowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPlanarConfigurationAllowed = planarConfigurationAllowed;
    }

    /**
     * Indicates whether the case where a dimension 2 null-space is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     *
     * @return true to allow 2 dimensional null-space, false otherwise.
     */
    public boolean isNullspaceDimension2Allowed() {
        return mNullspaceDimension2Allowed;
    }

    /**
     * Specifies whether the case where a dimension 2 null-space is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     *
     * @param nullspaceDimension2Allowed true to allow 2 dimensional nullspace,
     *                                   false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setNullspaceDimension2Allowed(
            final boolean nullspaceDimension2Allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mNullspaceDimension2Allowed = nullspaceDimension2Allowed;
    }

    /**
     * Gets threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     *
     * @return threshold to determine whether 3D matched points are in a planar
     * configuration.
     */
    public double getPlanarThreshold() {
        return mPlanarThreshold;
    }

    /**
     * Sets threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     *
     * @param planarThreshold threshold to determine whether 3D matched points
     *                        are in a planar configuration.
     * @throws IllegalArgumentException if provided threshold is negative.
     * @throws LockedException          if estimator is locked.
     */
    public void setPlanarThreshold(final double planarThreshold)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (planarThreshold < 0.0) {
            throw new IllegalArgumentException();
        }
        mPlanarThreshold = planarThreshold;
    }

    /**
     * Gets skewness value of intrinsic parameters to be used on estimated
     * camera.
     *
     * @return skewness value of intrinsic parameters to be used on estimated
     * camera.
     */
    public double getSkewness() {
        return mSkewness;
    }

    /**
     * Sets skewness value of intrinsic parameters to be used on estimated
     * camera.
     *
     * @param skewness skewness value of intrinsic parameters to be used on
     *                 estimated camera.
     * @throws LockedException if estimator is locked.
     */
    public void setSkewness(final double skewness) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSkewness = skewness;
    }

    /**
     * Gets horizontal coordinate of principal point on intrinsic parameters to
     * be used on estimated camera.
     *
     * @return horizontal coordinate of principal point.
     */
    public double getHorizontalPrincipalPoint() {
        return mHorizontalPrincipalPoint;
    }

    /**
     * Sets horizontal coordinate of principal point on intrinsic parameters to
     * be used on estimated camera.
     *
     * @param horizontalPrincipalPoint horizontal coordinate of principal point.
     * @throws LockedException if estimator is locked.
     */
    public void setHorizontalPrincipalPoint(final double horizontalPrincipalPoint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mHorizontalPrincipalPoint = horizontalPrincipalPoint;
    }

    /**
     * Gets vertical coordinate of principal point on intrinsic parameters to
     * be used on estimated camera.
     *
     * @return vertical coordinate of principal point.
     */
    public double getVerticalPrincipalPoint() {
        return mVerticalPrincipalPoint;
    }

    /**
     * Sets vertical coordinate of principal point on intrinsic parameters to
     * be used on estimated camera.
     *
     * @param verticalPrincipalPoint vertical coordinate of principal point.
     * @throws LockedException if estimator is locked.
     */
    public void setVerticalPrincipalPoint(final double verticalPrincipalPoint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mVerticalPrincipalPoint = verticalPrincipalPoint;
    }

    /**
     * Returns value indicating if each picked subset point correspondences are
     * normalized to increase the accuracy of the estimation.
     *
     * @return true if each picked subset point correspondences are normalized,
     * false otherwise.
     */
    @Override
    public boolean isNormalizeSubsetPointCorrespondences() {
        return false;
    }

    /**
     * Sets value indicating if each picked subset point correspondences are
     * normalized to increase the accuracy of the estimation.
     *
     * @param normalizeSubsetPointCorrespondences true if each picked subset
     *                                            point correspondences are normalized, false otherwise.
     * @throws LockedException if robust estimator is locked because an
     *                         estimation is already in progress.
     */
    @Override
    public void setNormalizeSubsetPointCorrespondences(
            final boolean normalizeSubsetPointCorrespondences)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate best
     *               pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator();
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator();
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size (6
     *                                  correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size (6
     *                                  correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and robust estimator
     * method.
     *
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener and quality scores.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and robust
     * estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case MSAC:
                return new MSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
            case PROSAC:
                return new PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D, qualityScores);
            case PROMedS:
                return new PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D, qualityScores);
            case RANSAC:
            default:
                return new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        listener, points3D, points2D);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method.
     *
     * @return an instance of a pinhole camera robust estimator.
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and default robust estimator method.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return create(points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and default robust estimator
     * method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and default robust estimator
     * method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return create(listener, points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and default robust
     * estimator method.
     *
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D,
            final double[] qualityScores) {
        return create(points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and default
     * robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static UPnPPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D,
            final double[] qualityScores) {
        return create(listener, points3D, points2D, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }
}
