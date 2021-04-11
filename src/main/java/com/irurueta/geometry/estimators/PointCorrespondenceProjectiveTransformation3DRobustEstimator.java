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

import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.refiners.PointCorrespondenceProjectiveTransformation3DRefiner;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best projective
 * 3D transformation for collection of matching 3D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class PointCorrespondenceProjectiveTransformation3DRobustEstimator
        extends ProjectiveTransformation3DRobustEstimator {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * List of points to be used to estimate a projective 3D transformation.
     * Each point in the list of input points must be matched with the
     * corresponding point in the list of output points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Point3D> mInputPoints;

    /**
     * List of points to be used to estimate a projective 3D transformation.
     * Each point in the list of output points must be matched with the
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Point3D> mOutputPoints;

    /**
     * Constructor.
     */
    protected PointCorrespondenceProjectiveTransformation3DRobustEstimator() {
        super();
    }

    /**
     * Constructor with lists of points to be used to estimate a projective 3D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected PointCorrespondenceProjectiveTransformation3DRobustEstimator(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        super();
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 stars, ends or its progress significantly changes.
     */
    protected PointCorrespondenceProjectiveTransformation3DRobustEstimator(
            final ProjectiveTransformation3DRobustEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor with listener and lists of points to be used to estimate a
     * projective 3D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected PointCorrespondenceProjectiveTransformation3DRobustEstimator(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        super(listener);
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Returns list of input points to be used to estimate a projective 3D
     * transformation.
     * Each point in the list of input points must be matched with the
     * corresponding point in the list of output points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of input points to be used to estimate a projective 3D
     * transformation.
     */
    public List<Point3D> getInputPoints() {
        return mInputPoints;
    }

    /**
     * Returns list of output points to be used to estimate a projective 3D
     * transformation.
     * Each point in the list of output points must be matched with the
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of output points to be used to estimate an affine 2D
     * transformation.
     */
    public List<Point3D> getOutputPoints() {
        return mOutputPoints;
    }

    /**
     * Sets lists of points to be used to estimate a projective 3D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public final void setPoints(final List<Point3D> inputPoints,
                                final List<Point3D> outputPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Indicates if estimator is ready to start the projective 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points) are provided
     * and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputPoints != null && mOutputPoints != null &&
                mInputPoints.size() == mOutputPoints.size() &&
                mInputPoints.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each pair of matched points.
     * The larger the score value the betther the quality of the matching.
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
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Creates a projective 3D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate
     *               best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate
     *                     best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate
     *                 best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate
     *                     best affine 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate
     *                      best projective 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator();
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        }
    }

    /**
     * Creates an projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param inputPoints   list of input points to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPoints  list of output points to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate
     *                      best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints, qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints, qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        inputPoints, outputPoints);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate
     *                      best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPoints   list of input points to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPoints  list of output points to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate
     *                      best projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case MSAC:
                return new MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROSAC:
                return new PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores);
            case PROMedS:
                return new PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores);
            case RANSAC:
            default:
                return new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                        listener, inputPoints, outputPoints);
        }
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        return create(inputPoints, outputPoints, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        return create(listener, inputPoints, outputPoints,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints   list of input points to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPoints  list of output points to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores) {
        return create(inputPoints, outputPoints, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of projective 3D transformation estimator.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a projective 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPoints   list of input points to be used to estimate a
     *                      projective 3D transformation.
     * @param outputPoints  list of output points to be used to estimate a
     *                      projective 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of projective 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static PointCorrespondenceProjectiveTransformation3DRobustEstimator create(
            final ProjectiveTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores) {
        return create(listener, inputPoints, outputPoints, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Attempts to refine provided solution if refinement is requested.
     * This method returns a refined solution of the same provided solution
     * if refinement is not requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this
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
            final PointCorrespondenceProjectiveTransformation3DRefiner refiner =
                    new PointCorrespondenceProjectiveTransformation3DRefiner(
                            transformation, mKeepCovariance, getInliersData(),
                            mInputPoints, mOutputPoints,
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

    /**
     * Internal method to set lists of points to be used to estimate a
     * projective 3D transformation.
     * This method does not check whether estimator is locked or not.
     *
     * @param inputPoints  list of input points to be used to estimate a
     *                     projective 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     projective 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    private void internalSetPoints(final List<Point3D> inputPoints,
                                   final List<Point3D> outputPoints) {
        if (inputPoints.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        if (inputPoints.size() != outputPoints.size()) {
            throw new IllegalArgumentException();
        }
        mInputPoints = inputPoints;
        mOutputPoints = outputPoints;
    }
}
