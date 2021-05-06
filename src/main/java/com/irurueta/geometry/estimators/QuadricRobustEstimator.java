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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quadric;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best quadric
 * that fits in a collection of 3D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class QuadricRobustEstimator {

    /**
     * Minimum number of 3D points required to estimate a quadric.
     */
    public static final int MINIMUM_SIZE = 9;

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
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected QuadricRobustEstimatorListener mListener;

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
     * List of points to be used to estimate a quadric. Provided list must have
     * a size greater or equal than MINIMUM_SIZE.
     */
    protected List<Point3D> mPoints;

    /**
     * Matrix representation of a 3D point to be reused when computing
     * residuals.
     */
    private Matrix mTestPoint;

    /**
     * Matrix representation of a quadric to be reused when computing
     * residuals.
     */
    private Matrix mTestQ;

    /**
     * Constructor.
     */
    protected QuadricRobustEstimator() {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 stars, ends or its progress significantly changes.
     */
    protected QuadricRobustEstimator(final QuadricRobustEstimatorListener listener) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor with points.
     *
     * @param points 3D points to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected QuadricRobustEstimator(final List<Point3D> points) {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPoints(points);
    }

    /**
     * Constructor.
     *
     * @param points   3D points to estimate a quadric.
     * @param listener listener to be notified of events such as when estimation
     *                 stars, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected QuadricRobustEstimator(final QuadricRobustEstimatorListener listener,
                                     final List<Point3D> points) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPoints(points);
    }


    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public QuadricRobustEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final QuadricRobustEstimatorListener listener)
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
     * Indicates if this instance is locked because estimation is being computed.
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
     * Returns list of points to be used to estimate a quadric.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @return list of points to be used to estimate a quadric.
     */
    public List<Point3D> getPoints() {
        return mPoints;
    }

    /**
     * Sets list of points to be used to estimate a quadric.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @param points list of points to be used to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setPoints(final List<Point3D> points) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(points);
    }

    /**
     * Indicates if estimator is ready to start the quadric estimation.
     * This is true when a minimum if MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mPoints != null && mPoints.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each point.
     * The larger the score value the better the quality of the point measure.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each point.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each point.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate best
     *               quadric.
     * @return an instance of a quadric robust estimator.
     */
    public static QuadricRobustEstimator create(final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator();
            case MSAC:
                return new MSACQuadricRobustEstimator();
            case PROSAC:
                return new PROSACQuadricRobustEstimator();
            case PROMedS:
                return new PROMedSQuadricRobustEstimator();
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator();
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided points and robust estimator method.
     *
     * @param points 3D points to estimate a quadric.
     * @param method method of a robust estimator algorithm to estimate best
     *               quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(final List<Point3D> points,
                                                final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator(points);
            case MSAC:
                return new MSACQuadricRobustEstimator(points);
            case PROSAC:
                return new PROSACQuadricRobustEstimator(points);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(points);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator(points);
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 quadric.
     * @return an instance of a quadric robust estimator.
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator(listener);
            case MSAC:
                return new MSACQuadricRobustEstimator(listener);
            case PROSAC:
                return new PROSACQuadricRobustEstimator(listener);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(listener);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator(listener);
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener and points.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   3D points to estimate a quadric.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener, final List<Point3D> points,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator(listener, points);
            case MSAC:
                return new MSACQuadricRobustEstimator(listener, points);
            case PROSAC:
                return new PROSACQuadricRobustEstimator(listener, points);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(listener, points);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator(listener, points);
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 points).
     */
    public static QuadricRobustEstimator create(final double[] qualityScores,
                                                final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator();
            case MSAC:
                return new MSACQuadricRobustEstimator();
            case PROSAC:
                return new PROSACQuadricRobustEstimator(qualityScores);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(qualityScores);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator();
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided points and robust estimator method.
     *
     * @param points        3D points to estimate a quadric.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(
            final List<Point3D> points, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator(points);
            case MSAC:
                return new MSACQuadricRobustEstimator(points);
            case PROSAC:
                return new PROSACQuadricRobustEstimator(points, qualityScores);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(points, qualityScores);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator(points);
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 points).
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator(listener);
            case MSAC:
                return new MSACQuadricRobustEstimator(listener);
            case PROSAC:
                return new PROSACQuadricRobustEstimator(listener, qualityScores);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator(listener);
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener and points.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        3D points to estimate a quadric.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener, final List<Point3D> points,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSQuadricRobustEstimator(listener, points);
            case MSAC:
                return new MSACQuadricRobustEstimator(listener, points);
            case PROSAC:
                return new PROSACQuadricRobustEstimator(listener, points,
                        qualityScores);
            case PROMedS:
                return new PROMedSQuadricRobustEstimator(listener, points,
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACQuadricRobustEstimator(listener, points);
        }
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * default robust estimator method.
     *
     * @return an instance of a quadric robust estimator.
     */
    public static QuadricRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided points and default robust estimator method.
     *
     * @param points 3D points to estimate a quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(final List<Point3D> points) {
        return create(points, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a quadric robust estimator.
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener and points and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   3D points to estimate a quadric.
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener,
            final List<Point3D> points) {
        return create(listener, points, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 points).
     */
    public static QuadricRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided points and default estimator method.
     *
     * @param points        3D points to estimate a quadric.
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(final List<Point3D> points,
                                                final double[] qualityScores) {
        return create(points, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 points).
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a quadric robust estimator based on 3D point samples and using
     * provided listener and points and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        3D points to estimate a quadric.
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a quadric robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static QuadricRobustEstimator create(
            final QuadricRobustEstimatorListener listener,
            final List<Point3D> points,
            final double[] qualityScores) {
        return create(listener, points, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Estimates a quadric using a robust estimator and the best set of 3D
     * points that fit into the locus of the estimated quadric found using the
     * robust estimator.
     *
     * @return a quadric.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract Quadric estimate() throws LockedException,
            NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internal method to set lists of points to be used to estimate a quadric.
     * This method does not check whether estimator is locked or not.
     *
     * @param points list of points to be used to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    private void internalSetPoints(final List<Point3D> points) {
        if (points.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        mPoints = points;
    }

    /**
     * Computes the residual between a quadric and a point.
     *
     * @param q     a quadric.
     * @param point a 3D point.
     * @return residual.
     */
    protected double residual(final Quadric q, final Point3D point) {
        q.normalize();
        try {
            if (mTestQ == null) {
                mTestQ = q.asMatrix();
            } else {
                q.asMatrix(mTestQ);
            }

            if (mTestPoint == null) {
                mTestPoint = new Matrix(
                        Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH,
                        1);
            }
            point.normalize();
            mTestPoint.setElementAt(0, 0, point.getHomX());
            mTestPoint.setElementAt(1, 0, point.getHomY());
            mTestPoint.setElementAt(2, 0, point.getHomZ());
            mTestPoint.setElementAt(3, 0, point.getHomW());
            final Matrix locusMatrix = mTestPoint.transposeAndReturnNew();
            locusMatrix.multiply(mTestQ);
            locusMatrix.multiply(mTestPoint);
            return Math.abs(locusMatrix.getElementAt(0, 0));
        } catch (final AlgebraException e) {
            return Double.MAX_VALUE;
        }
    }
}
