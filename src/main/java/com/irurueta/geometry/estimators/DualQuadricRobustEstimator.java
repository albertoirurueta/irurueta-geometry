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
import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best dual
 * quadric that fits in a collection of 3D planes.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class DualQuadricRobustEstimator {
    /**
     * Minimum number of 3D planes required to estimate a Dual Quadric.
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
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected DualQuadricRobustEstimatorListener mListener;

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
     * List of planes to be used to estimate a dual quadric. Provided list must
     * have a size greater or equal than MINIMUM_SIZE.
     */
    protected List<Plane> mPlanes;

    /**
     * Matrix representation of a 3D plane to be reused when computing
     * residuals
     */
    protected Matrix mTestPlane;

    /**
     * Matrix representation of a dual quadric to be reused when computing
     * residuals.
     */
    private Matrix mTestDualQ;

    /**
     * Constructor.
     */
    protected DualQuadricRobustEstimator() {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected DualQuadricRobustEstimator(
            final DualQuadricRobustEstimatorListener listener) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor with lines.
     *
     * @param planes planes to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected DualQuadricRobustEstimator(final List<Plane> planes) {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPlanes(planes);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   planes to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    protected DualQuadricRobustEstimator(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPlanes(planes);
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public DualQuadricRobustEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final DualQuadricRobustEstimatorListener listener)
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
     * @return amount of progress variation before notiying a progres change
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
     * (which is equivalent to 100%). The amount of confidence indicates that
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
     * Returns list of planes to be used to estimate a dual quadric.
     * Provided list have a size greater or equal than MINIMUM_SIZE.
     *
     * @return list of planes to be used to estimate a dual quadric.
     */
    public List<Plane> getPlanes() {
        return mPlanes;
    }

    /**
     * Sets list of planes to be used to estimate a dual quadric.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @param planes list of planes to be used to estimate a dual quadric.
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
     * Indicates if estimator is ready to start the dual quadric estimation.
     * This is true when MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mPlanes != null && mPlanes.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each plane.
     * The larger the score value the better the quality of the plane measure.
     * This implementation always return null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores coresponding to each plane.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each plane.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each plane.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate bes dual
     *               quadric.
     * @return an instance of a dual quadric robust estimator.
     */
    public static DualQuadricRobustEstimator create(final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator();
            case MSAC:
                return new MSACDualQuadricRobustEstimator();
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator();
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator();
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator();
        }
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided planes and robust estimator method.
     *
     * @param planes 3D planes to estimate a dual quadric.
     * @param method method of a robust estimator algorithm to estimate best
     *               dual quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided list of planes don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final List<Plane> planes, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator(planes);
            case MSAC:
                return new MSACDualQuadricRobustEstimator(planes);
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(planes);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(planes);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator(planes);
        }
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 dual quadric.
     * @return an instance of a dual quadric robust estimator.
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator(listener);
            case MSAC:
                return new MSACDualQuadricRobustEstimator(listener);
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(listener);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(listener);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator(listener);
        }
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener and planes.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   3D planes to estimate a dual quadric.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 dual quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided list of planes don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator(listener, planes);
            case MSAC:
                return new MSACDualQuadricRobustEstimator(listener, planes);
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(listener, planes);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(listener, planes);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator(listener, planes);
        }
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate bes dual
     *                      quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 planes).
     */
    public static DualQuadricRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator();
            case MSAC:
                return new MSACDualQuadricRobustEstimator();
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(qualityScores);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(qualityScores);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator();
        }
    }

    /**
     * Creates a dual quadric robust estimator method based on plane samples and
     * using provided planes and robust estimator method.
     *
     * @param planes        3D planes to estimate a dual quadric.
     * @param qualityScores quality scores correspondign to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      dual quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final List<Plane> planes, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator(planes);
            case MSAC:
                return new MSACDualQuadricRobustEstimator(planes);
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(planes,
                        qualityScores);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(planes,
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator(planes);
        }
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      dual quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 planes).
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator(listener);
            case MSAC:
                return new MSACDualQuadricRobustEstimator(listener);
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(listener,
                        qualityScores);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(listener,
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator(listener);
        }
    }

    /**
     * Creates a dual conic robust estimator based on 3D plane samples and using
     * provied listener and planes.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        3D planes to estimate a dual quadric.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      dual quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  the same size as the list of provided qualitys cores, or it their size is
     *                                  not greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSDualQuadricRobustEstimator(listener, planes);
            case MSAC:
                return new MSACDualQuadricRobustEstimator(listener, planes);
            case PROSAC:
                return new PROSACDualQuadricRobustEstimator(listener, planes,
                        qualityScores);
            case PROMedS:
                return new PROMedSDualQuadricRobustEstimator(listener, planes,
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACDualQuadricRobustEstimator(listener, planes);
        }
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * default robust estimator method.
     *
     * @return an instance of a dual quadric robust estimator.
     */
    public static DualQuadricRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided planes and default robust estimator method.
     *
     * @param planes 3D planes to estimate a dual quadric.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(final List<Plane> planes) {
        return create(planes, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a dual quadric robust estimator.
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener and lines and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   3D planes to estimate a dual quadric.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes) {
        return create(listener, planes, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on 3D plane samples and
     * using default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided plane.
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 planes).
     */
    public static DualQuadricRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided planes and default estimator method.
     *
     * @param planes        3D planes to estimate a dual quadric.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @return an instance of adual conic robust estimator.
     * @throws IllegalArgumentException if provided list of planes don't have the
     *                                  same size as the list of provided quality scores, or if their size is not
     *                                  greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final List<Plane> planes, final double[] qualityScores) {
        return create(planes, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided plane
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 planes).
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener, final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual quadric robust estimator based on plane samples and using
     * provided listener and planes and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        3D planes to estimate a dual quadric.
     * @param qualityScores quality scores corresponding to each provided plane
     * @return an instance of a dual quadric robust estimator.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  the same size as the list of provided quality scores, or if their size is
     *                                  not greater or equal than MINIMUM_SIZE.
     */
    public static DualQuadricRobustEstimator create(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes,
            final double[] qualityScores) {
        return create(listener, planes, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Estimates a dual conic using a robust estimator and the best set of 3D
     * planes that fit into the locus of the estimated dual quadric found using
     * the robust estimator.
     *
     * @return a dual quadric.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enouth to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any readon (i.e.
     *                                  numerical instability, no solution available, etc).
     */
    public abstract DualQuadric estimate() throws LockedException,
            NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internal method to set list of planes to be used to estimate a dual
     * quadric.
     * This method does not check whether estimator is locked or not.
     *
     * @param planes list of planes to be used to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    private void internalSetPlanes(final List<Plane> planes) {
        if (planes.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        mPlanes = planes;
    }

    /**
     * Computes the residual between a dual quadric and a 3D plane.
     *
     * @param dq    a dual quadric.
     * @param plane a 3D plane.
     * @return residual.
     */
    protected double residual(final DualQuadric dq, final Plane plane) {
        dq.normalize();
        try {
            if (mTestDualQ == null) {
                mTestDualQ = dq.asMatrix();
            } else {
                dq.asMatrix(mTestDualQ);
            }

            if (mTestPlane == null) {
                mTestPlane = new Matrix(Plane.PLANE_NUMBER_PARAMS, 1);
            }
            plane.normalize();
            mTestPlane.setElementAt(0, 0, plane.getA());
            mTestPlane.setElementAt(1, 0, plane.getB());
            mTestPlane.setElementAt(2, 0, plane.getC());
            mTestPlane.setElementAt(3, 0, plane.getD());
            final Matrix locusMatrix = mTestPlane.transposeAndReturnNew();
            locusMatrix.multiply(mTestDualQ);
            locusMatrix.multiply(mTestPlane);
            return Math.abs(locusMatrix.getElementAt(0, 0));
        } catch (final AlgebraException e) {
            return Double.MAX_VALUE;
        }
    }
}
