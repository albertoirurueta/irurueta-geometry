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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.refiners.EuclideanTransformation2DRefiner;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class to robustly find the best euclidean transformation
 * for collections of matching 2D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class EuclideanTransformation2DRobustEstimator {

    /**
     * Minimum number of matched points required to estimate an euclidean 2D
     * transformation.
     */
    public static final int MINIMUM_SIZE =
            EuclideanTransformation2DEstimator.MINIMUM_SIZE;

    /**
     * For some point configurations a solution can be found with only 2 points.
     */
    public static final int WEAK_MINIMUM_SIZE =
            EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE;

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
     * Indicates that is refined by default using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is not kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = false;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected EuclideanTransformation2DRobustEstimatorListener mListener;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected boolean mLocked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float mProgressDelta;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exaclty 1.0.
     */
    protected double mConfidence;

    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int mMaxIterations;

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
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    protected boolean mKeepCovariance;

    /**
     * Estimated covariance of estimated 2D euclidean transformation.
     * This is only available when result has been refined and covariance is
     * kept.
     */
    protected Matrix mCovariance;

    /**
     * List of points to be used to estimate an euclidean 2D transformation.
     * Each point in the list of input points must be matched with the
     * corresponding point in the list of outputp oints located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Point2D> mInputPoints;

    /**
     * List of points to be used to estimate an eculidean 2D transformation.
     * Each point in the lis tof output points must be matched with the
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output ponits must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Point2D> mOutputPoints;

    /**
     * Indicates whether estimation can start with only 2 points or not.
     * True allows 2 points, false requires 3.
     */
    private boolean mWeakMinimumSizeAllowed;


    /**
     * Constructor.
     */
    protected EuclideanTransformation2DRobustEstimator() {
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
    protected EuclideanTransformation2DRobustEstimator(
            final EuclideanTransformation2DRobustEstimatorListener listener) {
        this();
        mListener = listener;
    }

    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected EuclideanTransformation2DRobustEstimator(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        this();
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected EuclideanTransformation2DRobustEstimator(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        this(listener);
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     */
    protected EuclideanTransformation2DRobustEstimator(
            final boolean weakMinimumSizeAllowed) {
        this();
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Constructor.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     */
    protected EuclideanTransformation2DRobustEstimator(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final boolean weakMinimumSizeAllowed) {
        this();
        mListener = listener;
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected EuclideanTransformation2DRobustEstimator(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed) {
        this();
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected EuclideanTransformation2DRobustEstimator(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed) {
        this(listener);
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Returns list of input points to be used to estimate an euclidean 2D
     * transformation.
     * Each point in the list of input points must be matched with the
     * corresponding point in the list of output points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of input points to be used to estimate an euclidean 2D
     * transformation.
     */
    public List<Point2D> getInputPoints() {
        return mInputPoints;
    }

    /**
     * Returns list of output points to be used to estimate an euclidean 2D
     * transformation.
     * Each point in the list of output points must be matched with the
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of output points to be used to estimate an euclidean 2D
     * transformation.
     */
    public List<Point2D> getOutputPoints() {
        return mOutputPoints;
    }

    /**
     * Sets list of points to be used to estimate an euclidean 2D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setPoints(final List<Point2D> inputPoints,
                          final List<Point2D> outputPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Indicates if estimator is ready to start the euclidean 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points) are provided
     * and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputPoints != null && mOutputPoints != null &&
                mInputPoints.size() == mOutputPoints.size() &&
                mInputPoints.size() >= getMinimumPoints();
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
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public EuclideanTransformation2DRobustEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(
            final EuclideanTransformation2DRobustEstimatorListener listener)
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
     * Indicates whether estimation can start with only 2 points or not.
     *
     * @return true allows 2 points, false requires 3.
     */
    public boolean isWeakMinimumSizeAllowed() {
        return mWeakMinimumSizeAllowed;
    }

    /**
     * Specifies whether estimation can start with only 2 points or not.
     *
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws LockedException if estimator is locked.
     */
    public void setWeakMinimumSizeAllowed(final boolean weakMinimumSizeAllowed)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Required minimum number of point correspondences to start the estimation.
     * Can be either 2 or 3.
     *
     * @return minimum number of point correspondences.
     */
    public int getMinimumPoints() {
        return mWeakMinimumSizeAllowed ? WEAK_MINIMUM_SIZE : MINIMUM_SIZE;
    }

    /**
     * Indicates if this instance is locked because estimation is being
     * computed.
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
     * @return maximum allowed number of itertions.
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
     * If ture, inliers will be computed and kept in any implementation
     * regardless of the settings.
     *
     * @return true to refine result, false to simply use result found by
     * robust estimator without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using LEvenberg-Marquardt
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
     * Estimates an euclidean 2D transformation using a robust estimator and the
     * best set of matched 2D point correspondences found using the robust
     * estimator.
     *
     * @return an euclidean 2D transformation.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract EuclideanTransformation2D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();


    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate
     *               best euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator();
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator();
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator();
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator();
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided estimator method.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param method       method of a robust estimator algorithm to estimate best
     *                     euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust esitmator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 eculidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param method       method of a robust estimator algorithm to estimate best
     *                     euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 matched points).
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator();
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator();
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator();
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param inputPoints   list of input points to be used to estimate an
     *                      euclidean 2D transformation.
     * @param outputPoints  list of output points to be used to estimate an
     *                      euclidean 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points or scores
     *                                  don't have the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, qualityScores);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, qualityScores);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPoints   list of input points to be used to estimate an
     *                      euclidean 2D transformation.
     * @param outputPoints  list of output points to be used to estimate an
     *                      euclidean 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size of their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate
     *                               best euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final boolean weakMinimumSizeAllowed, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided estimator method.
     *
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust esitmator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               eculidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final boolean weakMinimumSizeAllowed, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 matched points).
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final double[] qualityScores, final boolean weakMinimumSizeAllowed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        qualityScores, weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        qualityScores, weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points or scores
     *                                  don't have the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores, final boolean weakMinimumSizeAllowed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, qualityScores,
                        weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, qualityScores,
                        weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        inputPoints, outputPoints, weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided quality scores don't have
     *                                  the required minimum size.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores, final boolean weakMinimumSizeAllowed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, qualityScores, weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, qualityScores, weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener, weakMinimumSizeAllowed);
        }
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @param method                 method of a robust estimator algorithm to estimate best
     *                               euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size of their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores, final boolean weakMinimumSizeAllowed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
            case MSAC:
                return new MSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
            case PROSAC:
                return new PROSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores,
                        weakMinimumSizeAllowed);
            case PROMedS:
                return new PROMedSEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints, qualityScores,
                        weakMinimumSizeAllowed);
            case RANSAC:
            default:
                return new RANSACEuclideanTransformation2DRobustEstimator(
                        listener, inputPoints, outputPoints,
                        weakMinimumSizeAllowed);
        }
    }


    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size of their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        return create(inputPoints, outputPoints, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        return create(listener, inputPoints, outputPoints,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints   list of input points to be used to estimate an
     *                      euclidean 2D transformation.
     * @param outputPoints  list of output points ot be used to estimate an
     *                      euclidean 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of points.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores) {
        return create(inputPoints, outputPoints, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPoints   list of input points to be used to estimate an
     *                      euclidean 2D transformation.
     * @param outputPoints  list of output points ot be used to estimate an
     *                      euclidean 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores) {
        return create(listener, inputPoints, outputPoints, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final boolean weakMinimumSizeAllowed) {
        return create(weakMinimumSizeAllowed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size of their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed) {
        return create(inputPoints, outputPoints, weakMinimumSizeAllowed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final boolean weakMinimumSizeAllowed) {
        return create(listener, weakMinimumSizeAllowed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed) {
        return create(listener, inputPoints, outputPoints,
                weakMinimumSizeAllowed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final double[] qualityScores, final boolean weakMinimumSizeAllowed) {
        return create(qualityScores, weakMinimumSizeAllowed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points ot be used to estimate an
     *                               euclidean 2D transformation.
     * @param qualityScores          quality scores corresponding to each pair of points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores, final boolean weakMinimumSizeAllowed) {
        return create(inputPoints, outputPoints, qualityScores,
                weakMinimumSizeAllowed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores, final boolean weakMinimumSizeAllowed) {
        return create(listener, qualityScores, weakMinimumSizeAllowed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an euclidean 2D transformation estimator based on 2D point
     * correspondences and using default robust estimator method.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param inputPoints            list of input points to be used to estimate an
     *                               euclidean 2D transformation.
     * @param outputPoints           list of output points ot be used to estimate an
     *                               euclidean 2D transformation.
     * @param qualityScores          quality scores corresponding to each pair of matched
     *                               points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @return an instance of euclidean 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static EuclideanTransformation2DRobustEstimator create(
            final EuclideanTransformation2DRobustEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final double[] qualityScores, final boolean weakMinimumSizeAllowed) {
        return create(listener, inputPoints, outputPoints, qualityScores,
                weakMinimumSizeAllowed, DEFAULT_ROBUST_METHOD);
    }


    /**
     * Internal method to set lists of points to be used to estimate an
     * euclidean 2D transformation.
     * This method does not check whether estimator is locked or not.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    private void internalSetPoints(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        if (inputPoints.size() < getMinimumPoints()) {
            throw new IllegalArgumentException();
        }
        if (inputPoints.size() != outputPoints.size()) {
            throw new IllegalArgumentException();
        }
        mInputPoints = inputPoints;
        mOutputPoints = outputPoints;
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
    protected EuclideanTransformation2D attemptRefine(
            final EuclideanTransformation2D transformation) {
        if (mRefineResult) {
            EuclideanTransformation2DRefiner refiner =
                    new EuclideanTransformation2DRefiner(transformation,
                            mKeepCovariance, getInliersData(), mInputPoints,
                            mOutputPoints, getRefinementStandardDeviation());

            try {
                final EuclideanTransformation2D result = new EuclideanTransformation2D();
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
}
