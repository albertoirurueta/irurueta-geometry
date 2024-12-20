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

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.refiners.DecomposedPointCorrespondencePinholeCameraRefiner;
import com.irurueta.geometry.refiners.NonDecomposedPointCorrespondencePinholeCameraRefiner;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best pinhole
 * camera for collections of matched 3D/2D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class PointCorrespondencePinholeCameraRobustEstimator extends PinholeCameraRobustEstimator {

    /**
     * Minimum number of required point correspondences to estimate a pinhole
     * camera.
     */
    public static final int MIN_NUMBER_OF_POINT_CORRESPONDENCES = 6;

    /**
     * Indicates if by default point correspondences for each picked subset of
     * samples is normalized to increase the accuracy of the estimation.
     */
    public static final boolean DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES = true;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.PROMEDS;

    /**
     * List of matched 3D points.
     */
    protected List<Point3D> points3D;

    /**
     * List of matched 2D points.
     */
    protected List<Point2D> points2D;

    /**
     * Indicates if each picked subset point correspondences are normalized to
     * increase the accuracy of the estimation.
     */
    protected boolean normalizeSubsetPointCorrespondences;

    /**
     * Constructor.
     */
    protected PointCorrespondencePinholeCameraRobustEstimator() {
        super();
        normalizeSubsetPointCorrespondences = DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES;
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
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    protected PointCorrespondencePinholeCameraRobustEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super();
        normalizeSubsetPointCorrespondences = DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES;
        internalSetPoints(points3D, points2D);
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected PointCorrespondencePinholeCameraRobustEstimator(final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
        normalizeSubsetPointCorrespondences = DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES;
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
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    protected PointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(listener);
        normalizeSubsetPointCorrespondences = DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES;
        internalSetPoints(points3D, points2D);
    }

    /**
     * Returns list of 3D points to be used to estimate a pinhole camera.
     * Each point in the list of 3D points must be matched with the
     * corresponding 2D point in the list of projected 2D points located at the
     * same position. Hence, both 2D and 3D points lists must have the same
     * size, and their size must be greater or equal than
     * MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     *
     * @return list of 3D points to be used to estimate a pinhole camera.
     */
    public List<Point3D> getPoints3D() {
        return points3D;
    }

    /**
     * Returns list of 2D points ot be used to estimate a pinhole camera.
     * Each point in the list of 2D points must be matched with the
     * corresponding 3D point in the list of 3D points that can be projected
     * into a 2D point using a pinhole camera. Hence, both 2D and 3D points
     * lists must have the same size, and their size must be greater or equal
     * than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     *
     * @return list of 2D points to be used to estimate a pinhole camera.
     */
    public List<Point2D> getPoints2D() {
        return points2D;
    }

    /**
     * Sets lists of 3D/2D points to be used to estimate a pinhole camera.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public final void setPoints(final List<Point3D> points3D, final List<Point2D> points2D) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(points3D, points2D);
    }

    /**
     * Indicates if estimator is ready to start the pinhole camera estimation.
     * This is true when input data (i.e. lists of 2D/3D matched points) are
     * provided and a minimum of MIN_NUMBER_OF_POINT_CORRESPONDENCES are
     * available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return points3D != null && points2D != null && points3D.size() == points2D.size()
                && points3D.size() >= MIN_NUMBER_OF_POINT_CORRESPONDENCES;
    }

    /**
     * Returns quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each pair of matched points.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MIN_NUMBER_OF_POINT_CORRESPONDENCES (i.e. 6 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Returns value indicating if each picked subset point correspondences are
     * normalized to increase the accuracy of the estimation.
     *
     * @return true if each picked subset point correspondences are normalized,
     * false otherwise.
     */
    public boolean isNormalizeSubsetPointCorrespondences() {
        return normalizeSubsetPointCorrespondences;
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
    public void setNormalizeSubsetPointCorrespondences(final boolean normalizeSubsetPointCorrespondences)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.normalizeSubsetPointCorrespondences = normalizeSubsetPointCorrespondences;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method + DLT.
     *
     * @param method method of a robust estimator algorithm to estimate the best
     *               pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method + DLT.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and robust estimator method
     * + DLT.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(listener, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and robust estimator method +
     * DLT.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method + DLT.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and robust estimator
     * method + DLT.
     *
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and quality scores and robust
     * estimator method + DLT.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and robust
     * estimator method + DLT.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        return DLTPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D, qualityScores,
                method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method + EPnP.
     *
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param method    method of a robust estimator algorithm to estimate the best
     *                  pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method + EPnP.
     *
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @param method    method of a robust estimator algorithm to estimate the best
     *                  pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and robust estimator method
     * + EPnP.
     *
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param method    method of a robust estimator algorithm to estimate the best
     *                  pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and robust estimator method +
     * EPnP.
     *
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @param method    method of a robust estimator algorithm to estimate the best
     *                  pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, points3D, points2D,
                method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method + EPnP.
     *
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and robust estimator
     * method + EPnP.
     *
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, points3D, points2D, qualityScores,
                method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and quality scores and
     * a robust estimation method + EPnP.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and robust
     * estimator method + EPnP.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return EPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, points3D, points2D,
                qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method + UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method + UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and robust method + UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and robust estimator method +
     * UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D, final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method + UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(qualityScores, method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and robust estimator
     * method + UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                qualityScores, method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and quality scores and robust
     * method + UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, qualityScores,
                method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and robust
     * estimator method + UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        final var estimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D,
                qualityScores, method);
        try {
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
        } catch (final LockedException ignore) {
            // ignore
        }
        return estimator;
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method + DLT.
     *
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and default robust estimator method +
     * DLT.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return create(points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and default robust estimator
     * method + DLT.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and default robust estimator
     * method + DLT.
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
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D) {
        return create(listener, points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and default robust estimator method +
     * DLT.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and default robust
     * estimator method + DLT.
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
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores) {
        return create(points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method + DLT.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and default
     * robust estimator method + DLT.
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
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return create(listener, points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method + EPnP.
     *
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic) {
        return create(intrinsic, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and default robust estimator method +
     * EPnP.
     *
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D) {
        return create(intrinsic, points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and default robust estimator
     * method + EPnP.
     *
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic) {
        return create(listener, intrinsic, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and default robust estimator
     * method + EPnP.
     *
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return create(listener, intrinsic, points3D, points2D, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and default robust estimator method +
     * EPnP.
     *
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final double[] qualityScores) {
        return create(intrinsic, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and default robust
     * estimator method + EPnP.
     *
     * @param intrinsic     intrinsic parameters of camera to be estimated.
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
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return create(intrinsic, points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method + EPnP.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final double[] qualityScores) {
        return create(listener, intrinsic, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and default
     * robust estimator method + EPnP.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param intrinsic     intrinsic parameters of camera to be estimated.
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
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, List<Point2D> points2D, final double[] qualityScores) {
        return create(listener, intrinsic, points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method + UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint) {
        return create(skewness, horizontalPrincipalPoint, verticalPrincipalPoint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and default robust estimator method +
     * EPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return create(skewness, horizontalPrincipalPoint, verticalPrincipalPoint, points3D, points2D,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and default robust estimator
     * method + EPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint) {
        return create(listener, skewness, horizontalPrincipalPoint, verticalPrincipalPoint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points and default robust estimator
     * method + UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D) {
        return create(listener, skewness, horizontalPrincipalPoint, verticalPrincipalPoint, points3D, points2D,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and default robust estimator method +
     * UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final double[] qualityScores) {
        return create(skewness, horizontalPrincipalPoint, verticalPrincipalPoint, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points, quality scores and default robust
     * estimator method + UPnP.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores) {
        return create(skewness, horizontalPrincipalPoint, verticalPrincipalPoint, points3D, points2D, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method + UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final double[] qualityScores) {
        return create(listener, skewness, horizontalPrincipalPoint, verticalPrincipalPoint, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided listener, 2D/3D points, quality scores and default
     * robust estimator method + UPnP.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return create(listener, skewness, horizontalPrincipalPoint, verticalPrincipalPoint, points3D, points2D,
                qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Attempts to refine provided camera.
     *
     * @param pinholeCamera camera to be refined.
     * @param weight        weight for suggestion residual.
     * @return refined camera or provided camera if anything fails.
     */
    protected PinholeCamera attemptRefine(final PinholeCamera pinholeCamera, final double weight) {
        if (hasSuggestions() && useFastRefinement) {
            return attemptFastRefine(pinholeCamera, weight);
        } else {
            return attemptSlowRefine(pinholeCamera, weight);
        }
    }

    /**
     * Internal method to set lists of points to be used to estimate a pinhole
     * camera.
     * This method does not check whether estimator is locked or not.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size (6
     *                                  points).
     */
    private void internalSetPoints(final List<Point3D> points3D, final List<Point2D> points2D) {
        if (points3D.size() < MIN_NUMBER_OF_POINT_CORRESPONDENCES) {
            throw new IllegalArgumentException();
        }
        if (points3D.size() != points2D.size()) {
            throw new IllegalArgumentException();
        }
        this.points3D = points3D;
        this.points2D = points2D;
    }

    /**
     * Attempts to refine provided camera using a slow but more accurate and
     * stable algorithm by first doing a Powell optimization and then
     * obtaining covariance using Levenberg/Marquardt if needed.
     *
     * @param pinholeCamera camera to be refined.
     * @param weight        weight for suggestion residual.
     * @return refined camera or provided camera if anything fails.
     */
    private PinholeCamera attemptSlowRefine(final PinholeCamera pinholeCamera, final double weight) {
        final var inliersData = getInliersData();
        if ((refineResult || keepCovariance) && inliersData != null) {
            final var refiner = new DecomposedPointCorrespondencePinholeCameraRefiner(pinholeCamera, keepCovariance,
                    inliersData, points3D, points2D, getRefinementStandardDeviation());
            try {
                if (refineResult) {
                    refiner.setMinSuggestionWeight(weight);
                    refiner.setMaxSuggestionWeight(weight);

                    refiner.setSuggestSkewnessValueEnabled(suggestSkewnessValueEnabled);
                    refiner.setSuggestedSkewnessValue(suggestedSkewnessValue);
                    refiner.setSuggestHorizontalFocalLengthEnabled(suggestHorizontalFocalLengthEnabled);
                    refiner.setSuggestedHorizontalFocalLengthValue(suggestedHorizontalFocalLengthValue);
                    refiner.setSuggestVerticalFocalLengthEnabled(suggestVerticalFocalLengthEnabled);
                    refiner.setSuggestedVerticalFocalLengthValue(suggestedVerticalFocalLengthValue);
                    refiner.setSuggestAspectRatioEnabled(suggestAspectRatioEnabled);
                    refiner.setSuggestedAspectRatioValue(suggestedAspectRatioValue);
                    refiner.setSuggestPrincipalPointEnabled(suggestPrincipalPointEnabled);
                    refiner.setSuggestedPrincipalPointValue(suggestedPrincipalPointValue);
                    refiner.setSuggestRotationEnabled(suggestRotationEnabled);
                    refiner.setSuggestedRotationValue(suggestedRotationValue);
                    refiner.setSuggestCenterEnabled(suggestCenterEnabled);
                    refiner.setSuggestedCenterValue(suggestedCenterValue);
                }

                final var result = new PinholeCamera();
                final var improved = refiner.refine(result);

                if (keepCovariance) {
                    // keep covariance
                    covariance = refiner.getCovariance();
                }

                return improved ? result : pinholeCamera;

            } catch (final Exception e) {
                return pinholeCamera;
            }
        } else {
            covariance = null;
            return pinholeCamera;
        }
    }

    /**
     * Attempts to refine provided camera using a fast algorithm based on
     * Levenberg/Marquardt.
     *
     * @param pinholeCamera camera to be refined.
     * @param weight        weight for suggestion residual.
     * @return refined camera or provided camera if anything fails.
     */
    private PinholeCamera attemptFastRefine(final PinholeCamera pinholeCamera, final double weight) {
        final var inliersData = getInliersData();
        if (refineResult && inliersData != null) {
            final var refiner = new NonDecomposedPointCorrespondencePinholeCameraRefiner(pinholeCamera, keepCovariance,
                    inliersData, points3D, points2D, getRefinementStandardDeviation());

            try {
                refiner.setSuggestionErrorWeight(weight);

                refiner.setSuggestSkewnessValueEnabled(suggestSkewnessValueEnabled);
                refiner.setSuggestedSkewnessValue(suggestedSkewnessValue);
                refiner.setSuggestHorizontalFocalLengthEnabled(suggestHorizontalFocalLengthEnabled);
                refiner.setSuggestedHorizontalFocalLengthValue(suggestedHorizontalFocalLengthValue);
                refiner.setSuggestVerticalFocalLengthEnabled(suggestVerticalFocalLengthEnabled);
                refiner.setSuggestedVerticalFocalLengthValue(suggestedVerticalFocalLengthValue);
                refiner.setSuggestAspectRatioEnabled(suggestAspectRatioEnabled);
                refiner.setSuggestedAspectRatioValue(suggestedAspectRatioValue);
                refiner.setSuggestPrincipalPointEnabled(suggestPrincipalPointEnabled);
                refiner.setSuggestedPrincipalPointValue(suggestedPrincipalPointValue);
                refiner.setSuggestRotationEnabled(suggestRotationEnabled);
                refiner.setSuggestedRotationValue(suggestedRotationValue);
                refiner.setSuggestCenterEnabled(suggestCenterEnabled);
                refiner.setSuggestedCenterValue(suggestedCenterValue);

                final var result = new PinholeCamera();
                final var improved = refiner.refine(result);

                if (keepCovariance) {
                    // keep covariance
                    covariance = refiner.getCovariance();
                }

                return improved ? result : pinholeCamera;
            } catch (final Exception e) {
                // refinement failed, so we return input value
                return pinholeCamera;
            }
        } else {
            return pinholeCamera;
        }
    }
}
