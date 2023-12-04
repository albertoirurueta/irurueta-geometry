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

import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.refiners.PlaneCorrespondenceProjectiveTransformation3DRefiner;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best projective
 * 3D transformation for collections of matching planes.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class PlaneCorrespondenceProjectiveTransformation3DRobustEstimator
        extends ProjectiveTransformation3DRobustEstimator {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMEDS;

    /**
     * List of planes to be used to estimate a projective 3D transformation.
     * Each line in the list of input lines must be matched with the
     * corresponding line in the list of output lines located at the same
     * position. Hence, both input lines and output lines must have the
     * same size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Plane> mInputPlanes;

    /**
     * List of planes to be used to estimate a projective 3D transformation.
     * Each point in the list of output lines must be matched with the
     * corresponding line in the list of input lines located at the same
     * position. Hence, both input lines and output lines must have the
     * same size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Plane> mOutputPlanes;

    /**
     * Constructor.
     */
    protected PlaneCorrespondenceProjectiveTransformation3DRobustEstimator() {
        super();
    }

    /**
     * Constructor with lists of planes to be used to estimate a projective 3D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes ot be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected PlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        super();
        internalSetPlanes(inputPlanes, outputPlanes);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected PlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            final ProjectiveTransformation3DRobustEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor with listener and lists of planes to be used to estimate
     * projective 3D transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected PlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        super(listener);
        internalSetPlanes(inputPlanes, outputPlanes);
    }

    /**
     * Returns list of input planes to be used to estimate a projective 3D
     * transformation.
     * Each plane in the list of input planes must be matched with the
     * corresponding planes in the list of output planes located at the same
     * position. Hence, both input planes and output planes must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE
     *
     * @return list of input planes to be used to estimate an affine 3D
     * transformation.
     */
    public List<Plane> getInputPlanes() {
        return mInputPlanes;
    }

    /**
     * Returns list of output planes to be used to estimate a projective 3D
     * transformation.
     * Each plane in the list of output planes must be matched with the
     * corresponding plane in the list of input planes located at the same
     * position. Hence, both input planes and output planes must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of output planes to be used to estimate a projective 3D
     * transformation.
     */
    public List<Plane> getOutputPlanes() {
        return mOutputPlanes;
    }

    /**
     * Sets lists of planes to be used to estimate a projective 3D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    protected final void setPlanes(final List<Plane> inputPlanes,
                                   final List<Plane> outputPlanes) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPlanes(inputPlanes, outputPlanes);
    }

    /**
     * Indicates if estimator is ready to start the projective 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched planes) are provided
     * and a minimum of MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputPlanes != null && mOutputPlanes != null &&
                mInputPlanes.size() == mOutputPlanes.size() &&
                mInputPlanes.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each pair of matched planes.
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
     * Sets quality scores corresponding to each pair of matched planes.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate
     *               best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method.
     *
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate
     *                     best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D line
     * correspondences and using provided robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input lines to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output lines to be used to estimate a
     *                     projective 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate best
     *                     projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, List<Plane> outputPlanes,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D plane
     * correspondences and using provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        qualityScores);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param inputPlanes   list of input planes to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes, qualityScores);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPlanes, outputPlanes);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, qualityScores);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPlanes   list of input planes to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMEDS:
                return new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case MSAC:
                return new MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
            case PROSAC:
                return new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes, qualityScores);
            case PROMEDS:
                return new PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes, qualityScores);
            case RANSAC:
            default:
                return new RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPlanes, outputPlanes);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        return create(inputPlanes, outputPlanes, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        return create(listener, inputPlanes, outputPlanes,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D line
     * correspondences and using default robust estimator method.
     *
     * @param inputPlanes   list of input planes to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      planes.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores) {
        return create(inputPlanes, outputPlanes, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D line
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPlanes   list of input planes to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PlaneCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores) {
        return create(listener, inputPlanes, outputPlanes, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Internal method to set lists of planes to be used to estimate a
     * projective 3D transformation.
     * This method does not check whether estimator is locked or not
     *
     * @param inputPlanes  list of input planes to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    private void internalSetPlanes(final List<Plane> inputPlanes,
                                   final List<Plane> outputPlanes) {
        if (inputPlanes.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        if (inputPlanes.size() != outputPlanes.size()) {
            throw new IllegalArgumentException();
        }
        mInputPlanes = inputPlanes;
        mOutputPlanes = outputPlanes;
    }

    /**
     * Computes residual by comparing two lines algebraically by doing the
     * dot product of their parameters.
     * A residual of 0 indicates that dot product was 1 or -1 and lines were
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were
     * orthogonal.
     * If dot product was -1, then although their director vectors are opposed,
     * lines are considered equal, since sign changes are not taken into account.
     *
     * @param plane            originally sampled output plane.
     * @param transformedPlane estimated output plane obtained after using
     *                         estimated transformation.
     * @return computed residual.
     */
    @SuppressWarnings("DuplicatedCode")
    protected static double getResidual(final Plane plane, final Plane transformedPlane) {
        plane.normalize();
        transformedPlane.normalize();

        final double dotProduct = Math.abs(plane.getA() * transformedPlane.getA() +
                plane.getB() * transformedPlane.getB() +
                plane.getC() * transformedPlane.getC() +
                plane.getD() * transformedPlane.getD());
        return 1.0 - dotProduct;
    }

    /**
     * Attempts to refine provided solution if refinement is requested.
     * This method returns a refined solution of the same provided solution
     * if refinement is not requested or has failed.
     * If refinement is enabled, and it is requested to keep covariance, this
     * method will also keep covariance of refined transformation.
     *
     * @param transformation transformation estimated by a robust estimator
     *                       without refinement.
     * @return solution after refinement (if requested) or the provided
     * non-refined solution if not requested or refinement failed.
     */
    @SuppressWarnings("DuplicatedCode")
    protected ProjectiveTransformation3D attemptRefine(
            final ProjectiveTransformation3D transformation) {
        if (mRefineResult) {
            final PlaneCorrespondenceProjectiveTransformation3DRefiner refiner =
                    new PlaneCorrespondenceProjectiveTransformation3DRefiner(
                            transformation, mKeepCovariance, getInliersData(),
                            mInputPlanes, mOutputPlanes,
                            getRefinementStandardDeviation());

            try {
                final ProjectiveTransformation3D result =
                        new ProjectiveTransformation3D();
                final boolean improved = refiner.refine(result);

                if (mKeepCovariance) {
                    // keep covariance
                    mCovariance = refiner.getCovariance();
                }

                return improved ? result : transformation;
            } catch (final Exception e) {
                // refinement failed, so we return input value
                return transformation;
            }
        } else {
            return transformation;
        }
    }
}
