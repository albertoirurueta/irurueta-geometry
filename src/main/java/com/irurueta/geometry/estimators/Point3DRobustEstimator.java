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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.refiners.HomogeneousPoint3DRefiner;
import com.irurueta.geometry.refiners.InhomogeneousPoint3DRefiner;
import com.irurueta.geometry.refiners.Point3DRefiner;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best 3D point
 * that intersects in a collection of 3D planes.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class Point3DRobustEstimator {
    /**
     * Minimum number of 3D planes required to estimate a point.
     */
    public static final int MINIMUM_SIZE = 3;

    /**
     * Default amount of progress variation before notifying a change in
     * estimation progress. By default this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen subsamples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Indicates that result is refined by default using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is not kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = false;

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected Point3DRobustEstimatorListener mListener;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected volatile boolean mLocked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float mProgressDelta;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double mConfidence;

    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int mMaxIterations;

    /**
     * List of lines to be used to estimate a 3D point. Provided list must have
     * a size greater or equal than MINIMUM_SIZE.
     */
    protected List<Plane> mPlanes;

    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData mInliersData;

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     */
    protected boolean mRefineResult;

    /**
     * Coordinates type to use for refinement. When using inhomogeneous
     * coordinates a 3x3 covariance matrix is estimated. When using homogeneous
     * coordinates a 4x4 covariance matrix is estimated.
     */
    private CoordinatesType mRefinementCoordinatesType =
            CoordinatesType.INHOMOGENEOUS_COORDINATES;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance;

    /**
     * Estimated covariance of estimated 3D point.
     * This is only available when result has been refined and covariance is
     * kept.
     */
    private Matrix mCovariance;

    /**
     * Constructor.
     */
    protected Point3DRobustEstimator() {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected Point3DRobustEstimator(final Point3DRobustEstimatorListener listener) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor with lines.
     *
     * @param planes 3D planes to estimate a 3D point.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected Point3DRobustEstimator(final List<Plane> planes) {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPlanes(planes);
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor.
     *
     * @param planes   3D planes to estimate a 3D point.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected Point3DRobustEstimator(final Point3DRobustEstimatorListener listener,
                                     final List<Plane> planes) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPlanes(planes);
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;
    }


    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public Point3DRobustEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final Point3DRobustEstimatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return mListener != null;
    }

    /**
     * Indicates if this instance is locked because estimation is being computed
     *
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @param progressDelta amount of progress variation before notifying a
     *                      progress change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     *                                  greater than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     * @throws LockedException          if this estimator is locked because an estimator
     *                                  is being computed.
     */
    public void setConfidence(final double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling
     * estimate(), a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mMaxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mInliersData;
    }

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     *
     * @return true to refine result, false to simply use result found by
     * robust estimator without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result
     *                     found by robust estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefineResult = refineResult;
    }

    /**
     * Gets coordinates type to use for refinement. When using inhomogeneous
     * coordinates a 3x3 covariance matrix is estimated. When using homogeneous
     * coordinates a 4x4 covariance matrix is estimated.
     *
     * @return coordinates type to use for refinement.
     */
    public CoordinatesType getRefinementCoordinatesType() {
        return mRefinementCoordinatesType;
    }

    /**
     * Sets coordinates type to use for refinement. When using inhomogeneous
     * coordinates a 3x3 covariance matrix is estimated. When using homogeneous
     * coordinates a 4x4 covariance matrix is estimated.
     *
     * @param refinementCoordinatesType coordinates type to use for refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementCoordinatesType(
            final CoordinatesType refinementCoordinatesType) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefinementCoordinatesType = refinementCoordinatesType;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false
     * otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining
     *                       result, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }

    /**
     * Returns list of planes to be used to estimate a 3D point.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @return list of planes to be used to estimate a 3D point.
     */
    public List<Plane> getPlanes() {
        return mPlanes;
    }

    /**
     * Sets list of planes to be used to estimate a 3D point.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @param planes list of planes to be used to estimate a 3D point.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setPlanes(final List<Plane> planes) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPlanes(planes);
    }

    /**
     * Indicates if estimator is ready to start the 3D point estimation.
     * This is true when a minimum if MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mPlanes != null && mPlanes.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each line.
     * The larger the score value the better the quality of the line measure.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each point.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each line.
     * The larger the score value the better the quality of the line measure.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each sampled line.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Gets estimated covariance of estimated 3D point if available.
     * This is only available when result has been refined and covariance is
     * kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Creates a 3D point robust estimator based on 3D line samples and using
     * provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate best
     *               3D point.
     * @return an instance of a 3D point robust estimator.
     */
    public static Point3DRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator();
            case MSAC:
                return new MSACPoint3DRobustEstimator();
            case PROSAC:
                return new PROSACPoint3DRobustEstimator();
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator();
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided planes and robust estimator method.
     *
     * @param planes 3D planes to estimate a 3D point.
     * @param method method of a robust estimator algorithm to estimate best
     *               3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final List<Plane> planes,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator(planes);
            case MSAC:
                return new MSACPoint3DRobustEstimator(planes);
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(planes);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(planes);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator(planes);
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 3D point.
     * @return an instance of a 3D point robust estimator.
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator(listener);
            case MSAC:
                return new MSACPoint3DRobustEstimator(listener);
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(listener);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(listener);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator(listener);
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener and planes.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   3D planes to estimate a 3D point.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener, List<Plane> planes,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator(listener, planes);
            case MSAC:
                return new MSACPoint3DRobustEstimator(listener, planes);
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(listener, planes);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(listener, planes);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator(listener, planes);
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 lines).
     */
    public static Point3DRobustEstimator create(
            final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator();
            case MSAC:
                return new MSACPoint3DRobustEstimator();
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(qualityScores);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(qualityScores);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator();
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided planes and robust estimator method.
     *
     * @param planes        3D planes to estimate a 3D point.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines doesn't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final List<Plane> planes, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator(planes);
            case MSAC:
                return new MSACPoint3DRobustEstimator(planes);
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(planes, qualityScores);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(planes, qualityScores);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator(planes);
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 lines).
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener,
            final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator(listener);
            case MSAC:
                return new MSACPoint3DRobustEstimator(listener);
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(listener, qualityScores);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator(listener);
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener and planes.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        3D planes to estimate a 3D point.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener,
            final List<Plane> planes, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSPoint3DRobustEstimator(listener, planes);
            case MSAC:
                return new MSACPoint3DRobustEstimator(listener, planes);
            case PROSAC:
                return new PROSACPoint3DRobustEstimator(listener, planes,
                        qualityScores);
            case PROMedS:
                return new PROMedSPoint3DRobustEstimator(listener, planes,
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACPoint3DRobustEstimator(listener, planes);
        }
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * default robust estimator method.
     *
     * @return an instance of a 3D point robust estimator.
     */
    public static Point3DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided planes and default robust estimator method.
     *
     * @param planes 3D planes to estimate a 3D point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines doesn't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(final List<Plane> planes) {
        return create(planes, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a 3D point robust estimator.
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener and planes and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   3D planes to estimate a point.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener,
            final List<Plane> planes) {
        return create(listener, planes, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided plane
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 planes).
     */
    public static Point3DRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided planes and default estimator method.
     *
     * @param planes        3D planes to estimate a 3D point.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final List<Plane> planes, final double[] qualityScores) {
        return create(planes, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided plane
     * @return an instance of a circle robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 planes).
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 3D point robust estimator based on 3D plane samples and using
     * provided listener and planes and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        3D planes to estimate a 3D point.
     * @param qualityScores quality scores corresponding to each provided plane
     * @return an instance of a 3D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point3DRobustEstimator create(
            final Point3DRobustEstimatorListener listener,
            final List<Plane> planes, final double[] qualityScores) {
        return create(listener, planes, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Estimates a 3D point using a robust estimator and the best set of 3D
     * planes that intersect into the estimated 3D point.
     *
     * @return a 3D point.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract Point3D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Computes the residual between a 3D point and a plane.
     *
     * @param p     a 3D point.
     * @param plane a 3D plane.
     * @return residual.
     */
    protected double residual(final Point3D p, final Plane plane) {
        p.normalize();
        plane.normalize();

        return Math.abs(plane.signedDistance(p));
    }

    /**
     * Attempts to refine provided solution if refinement is requested.
     * This method returns a refined solution or the same provided solution
     * if refinement is not requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this
     * method will also keep covariance of refined point.
     *
     * @param point point estimated by a robust estimator without refinement.
     * @return solution after refinement (if requested) or the provided non-
     * refined solution if not requested or if refinement failed.
     */
    protected Point3D attemptRefine(final Point3D point) {
        if (mRefineResult) {
            try {
                Point3DRefiner<? extends Point3D> refiner;
                Point3D result;
                final boolean improved;
                switch (mRefinementCoordinatesType) {
                    case HOMOGENEOUS_COORDINATES:
                        HomogeneousPoint3D homP;
                        if (point.getType() == CoordinatesType.HOMOGENEOUS_COORDINATES) {
                            homP = (HomogeneousPoint3D) point;
                        } else {
                            homP = new HomogeneousPoint3D(point);
                        }
                        final HomogeneousPoint3DRefiner homRefiner = new HomogeneousPoint3DRefiner(
                                homP, mKeepCovariance, getInliersData(), mPlanes,
                                getRefinementStandardDeviation());
                        refiner = homRefiner;
                        final HomogeneousPoint3D homResult = new HomogeneousPoint3D();
                        improved = homRefiner.refine(homResult);
                        result = homResult;
                        break;

                    case INHOMOGENEOUS_COORDINATES:
                    default:
                        InhomogeneousPoint3D inhomP;
                        if (point.getType() == CoordinatesType.INHOMOGENEOUS_COORDINATES) {
                            inhomP = (InhomogeneousPoint3D) point;
                        } else {
                            inhomP = new InhomogeneousPoint3D(point);
                        }
                        final InhomogeneousPoint3DRefiner inhomRefiner = new InhomogeneousPoint3DRefiner(
                                inhomP, mKeepCovariance, getInliersData(), mPlanes,
                                getRefinementStandardDeviation());
                        refiner = inhomRefiner;
                        final InhomogeneousPoint3D inhomResult = new InhomogeneousPoint3D();
                        improved = inhomRefiner.refine(inhomResult);
                        result = inhomResult;
                        break;
                }

                if (mKeepCovariance) {
                    // keep covariance
                    mCovariance = refiner.getCovariance();
                }

                return improved ? result : point;
            } catch (final Exception e) {
                // refinement failed, so we return input value
                return point;
            }
        } else {
            return point;
        }
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     *
     * @return standard deviation used for refinement.
     */
    protected abstract double getRefinementStandardDeviation();

    /**
     * Internal method to set list of 3D planes to be used to estimate a 3D
     * point.
     * This method does not check whether estimator is locked or not.
     *
     * @param planes list of planes to be used to estimate a 3D point.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    private void internalSetPlanes(final List<Plane> planes) {
        if (planes.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        mPlanes = planes;
    }
}
