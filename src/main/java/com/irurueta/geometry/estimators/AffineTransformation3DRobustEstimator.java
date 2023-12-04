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
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best affine
 * 3D transformation for collections of matching 3D points, or 3D planes.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class AffineTransformation3DRobustEstimator {

    /**
     * Minimum number of matched points or matched planes required to estimate
     * an affine 3D transformation.
     */
    public static final int MINIMUM_SIZE = 4;

    /**
     * Default amount of progress variation before notifying a change in
     * estimation progress. By default, this is set to 5%.
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
     * accurate because chosen sub-samples will be inliers.
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
    protected AffineTransformation3DRobustEstimatorListener mListener;

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
     * Estimated covariance of estimated 2D affine transformation.
     * This is only available when result has been refined and covariance is
     * kept.
     */
    protected Matrix mCovariance;

    /**
     * Constructor.
     */
    protected AffineTransformation3DRobustEstimator() {
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
     *                 stars, ends or its progress significantly changes.
     */
    protected AffineTransformation3DRobustEstimator(
            final AffineTransformation3DRobustEstimatorListener listener) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes
     *
     * @return listener to be notified of events.
     */
    public AffineTransformation3DRobustEstimatorListener getListener() {
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
            final AffineTransformation3DRobustEstimatorListener listener)
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
    public void setProgressDelta(final float progressDelta)
            throws LockedException {
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
     * @param confidence confidence to be set as a value between 0.0 and 1.0
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     * @throws LockedException          if this estimator is locked because an estimator
     *                                  is being computed.
     */
    public void setConfidence(final double confidence)
            throws LockedException {
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
    public void setMaxIterations(final int maxIterations)
            throws LockedException {
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
     * Estimates an affine 3D transformation using a robust estimator and
     * the best set of matched 3D point correspondences found using the robust
     * estimator.
     *
     * @return an affine 3D transformation.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract AffineTransformation3D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     affine 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate
     *                     the best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final RobustEstimatorMethod method) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate an
     *                     affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     affine 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate
     *                     the best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final RobustEstimatorMethod method) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on 2D point
     * correspondences and using provided robust estimator method.
     *
     * @param inputPoints   list of input points to be used to estimate an
     *                      affine 3D transformation.
     * @param outputPoints  list of output points to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate
     *                      the best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPoints   list of input points to be used to estimate an
     *                      affine 3D transformation.
     * @param outputPoints  list of output points to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate
     *                      the best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.
                create(inputPoints, outputPoints);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPoints  list of input points to be used to estimate an
     *                     affine 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param inputPoints   list of input points to be used to estimate an
     *                      affine 3D transformation.
     * @param outputPoints  list of output points to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPoints, outputPoints, qualityScores);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D point
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPoints   list of input points to be used to estimate an
     *                      affine 3D transformation.
     * @param outputPoints  list of output points to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPoints(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints,
            final double[] qualityScores) {
        return PointCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPoints, outputPoints, qualityScores);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param inputPlanes  list of input planes to be used to estimate an
     *                     affine 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an
     *                     affine 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate
     *                     the best affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final RobustEstimatorMethod method) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPlanes, outputPlanes, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input planes to be used to estimate an affine
     *                     3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an
     *                     affine 3D transformation.
     * @param method       method of a robust estimator algorithm to estimate the best
     *                     affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final RobustEstimatorMethod method) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPlanes, outputPlanes, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param inputPlanes   list of input planes to be used to estimate an
     *                      affine 3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPlanes, outputPlanes, qualityScores, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPlanes   list of input planes to be used to estimate an affine
     *                      3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPlanes, outputPlanes, qualityScores, method);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param inputPlanes  list of input planes to be used to estimate an
     *                     affine 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an
     *                     affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPlanes, outputPlanes);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input planes to be used to estimate an affine
     *                     3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an
     *                     affine 3D transformation.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPlanes, outputPlanes);
    }

    /**
     * Creates an affine 3D transformation estimator based on plane
     * correspondences and using default robust estimator method.
     *
     * @param inputPlanes   list of input planes to be used to estimate an affine
     *                      3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                inputPlanes, outputPlanes, qualityScores);
    }

    /**
     * Creates an affine 3D transformation estimator based on 3D line
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputPlanes   list of input planes to be used to estimate an affine
     *                      3D transformation.
     * @param outputPlanes  list of output planes to be used to estimate an
     *                      affine 3D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @return an instance of affine 3D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static AffineTransformation3DRobustEstimator createFromPlanes(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes,
            final double[] qualityScores) {
        return PlaneCorrespondenceAffineTransformation3DRobustEstimator.create(
                listener, inputPlanes, outputPlanes, qualityScores);
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
    protected abstract double getRefinementStandardDeviation();
}
