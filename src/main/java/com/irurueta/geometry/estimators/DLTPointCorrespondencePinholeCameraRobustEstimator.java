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
 * for collections of matched 3D/2D points using DLT (Direct Linear Transform)
 * algorithm.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class DLTPointCorrespondencePinholeCameraRobustEstimator
        extends PointCorrespondencePinholeCameraRobustEstimator {

    /**
     * Constructor.
     */
    protected DLTPointCorrespondencePinholeCameraRobustEstimator() {
        super();
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
    protected DLTPointCorrespondencePinholeCameraRobustEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(points3D, points2D);
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected DLTPointCorrespondencePinholeCameraRobustEstimator(final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
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
     * @param points3D lists of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    protected DLTPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D) {
        super(listener, points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate the best
     *               pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator();
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator();
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator();
        };
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided 2D/3D points and robust estimator method.
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
        };
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
        };
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
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
        };
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using provided quality scores and robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (6 samples).
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(final double[] qualityScores,
                                                                            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator();
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator();
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(qualityScores);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(qualityScores);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator();
        };
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
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                    points3D, points2D, qualityScores);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                    points3D, points2D, qualityScores);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);
        };
    }

    /**
     * Creates a pinhole camera robust estimator based on point
     * correspondences and using provided listener and quality scores.
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener, qualityScores);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener, qualityScores);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener);
        };
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
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
            case MSAC -> new MSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
            case PROSAC -> new PROSACDLTPointCorrespondencePinholeCameraRobustEstimator(
                    listener, points3D, points2D, qualityScores);
            case PROMEDS -> new PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator(
                    listener, points3D, points2D, qualityScores);
            default -> new RANSACDLTPointCorrespondencePinholeCameraRobustEstimator(listener, points3D, points2D);
        };
    }

    /**
     * Creates a pinhole camera robust estimator based on point correspondences
     * and using default robust estimator method.
     *
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create() {
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(final double[] qualityScores) {
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores) {
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final double[] qualityScores) {
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
    public static DLTPointCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return create(listener, points3D, points2D, qualityScores, DEFAULT_ROBUST_METHOD);
    }
}
