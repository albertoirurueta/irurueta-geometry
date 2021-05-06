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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Base abstract class for algorithms to robustly find the best pinhole camera
 * for collections of matched planes and lines using DLT algorithm.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class DLTLinePlaneCorrespondencePinholeCameraRobustEstimator
        extends LinePlaneCorrespondencePinholeCameraRobustEstimator {

    /**
     * Constructor.
     */
    protected DLTLinePlaneCorrespondencePinholeCameraRobustEstimator() {
        super();
    }

    /**
     * Constructor with lists of matched planes and 2D lines to estimate a
     * pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     *
     * @param planes list of planes used to estimate a pinhole camera.
     * @param lines  list of corresponding projected 2D lines used to estimate
     *               a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size or their size is smaller than required minimum size (4 matches).
     */
    protected DLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final List<Plane> planes, final List<Line2D> lines) {
        super(planes, lines);
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected DLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor with listener and lists of matched planes and 2D lines to
     * estimate a pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   list of planes used to estimate a pinhole camera.
     * @param lines    list of corresponding projected 2D lines used to estimate
     *                 a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size or their size is smaller than required minimum size (4 matches).
     */
    protected DLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Plane> planes, final List<Line2D> lines) {
        super(listener, planes, lines);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate best
     *               pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided planes and lines and provided robust
     * estimator method.
     *
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines  list of corresponding projected 2D lines used to estimate a
     *               pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     *               pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     *                                  don't have the same size or their size is smaller than required minimum
     *                                  size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final List<Plane> planes, final List<Line2D> lines,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener and robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener, planes and lines, and robust
     * estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   list of 3D planes to estimate a pinhole camera.
     * @param lines    list of corresponding projected 2D lines used to estimate a
     *                 pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     *                                  don't have the same size or their size is smaller than required minimum
     *                                  size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes,
            final List<Line2D> lines, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided quality scores and robust estimator
     * method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (4 samples).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided planes and lines, quality scores and
     * provided robust estimator method.
     *
     * @param planes        list of 3D planes to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate a
     *                      pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     *                                  quality scores don't have the same size of their size is smaller than
     *                                  required minimum size ($ correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final List<Plane> planes, final List<Line2D> lines, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines, qualityScores);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines, qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planes, lines);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/list
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (4 samples).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener, planes and lines, and robust
     * estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        list of 3D planes to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes,
            final List<Line2D> lines, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
            case MSAC:
                return new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
            case PROSAC:
                return new PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines, qualityScores);
            case PROMedS:
                return new PROMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines, qualityScores);
            case RANSAC:
            default:
                return new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        listener, planes, lines);
        }
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using default robust estimator method.
     *
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided planes and lines and default robust
     * estimator method.
     *
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines  list of corresponding projected 2D lines used to estimate
     *               a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     *                                  don't have the same size or their size is smaller than required minimum
     *                                  size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final List<Plane> planes, final List<Line2D> lines) {
        return create(planes, lines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener and default estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener, planes and lines, and
     * default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   list of 3D planes to estimate a pinhole camera.
     * @param lines    list of corresponding projected 2D lines used to estimate a
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     *                                  don't have the same size or their size is smaller than required minimum
     *                                  size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Plane> planes, final List<Line2D> lines) {
        return create(listener, planes, lines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided quality scores and default robust
     * estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required minimum size (4 samples).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided planes and lines, quality scores and
     * default robust estimator method.
     *
     * @param planes        list of 3D planes to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final List<Plane> planes, final List<Line2D> lines, final double[] qualityScores) {
        return create(planes, lines, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size (4 samples).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a pinhole camera robust estimator based on plane/line
     * correspondences and using provided listener, planes and lines, and
     * default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        list of 3D planes to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static DLTLinePlaneCorrespondencePinholeCameraRobustEstimator create(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes,
            final List<Line2D> lines, final double[] qualityScores) {
        return create(listener, planes, lines, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }
}
