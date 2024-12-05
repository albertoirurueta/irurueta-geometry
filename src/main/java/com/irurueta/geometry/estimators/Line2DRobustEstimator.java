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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best 2D line
 * that passes through a collection of 2D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class Line2DRobustEstimator {
    /**
     * Minimum number of 2D points required to estimate a line.
     */
    public static final int MINIMUM_SIZE = 2;

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
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.PROMEDS;

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected Line2DRobustEstimatorListener listener;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected volatile boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float progressDelta;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double confidence;

    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int maxIterations;

    /**
     * List of points to be used to estimate a 2D line. Provided list must have
     * a size greater or equal than MINIMUM_SIZE.
     */
    protected List<Point2D> points;

    /**
     * Constructor.
     */
    protected Line2DRobustEstimator() {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected Line2DRobustEstimator(final Line2DRobustEstimatorListener listener) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor with points.
     *
     * @param points 2D points to estimate a 2D line.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected Line2DRobustEstimator(final List<Point2D> points) {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPoints(points);
    }

    /**
     * Constructor.
     *
     * @param points   2D points to estimate a 2D line.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected Line2DRobustEstimator(final Line2DRobustEstimatorListener listener, final List<Point2D> points) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetPoints(points);
    }


    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public Line2DRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final Line2DRobustEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return listener != null;
    }

    /**
     * Indicates if this instance is locked because estimation is being computed.
     *
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return progressDelta;
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
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
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
        return confidence;
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
        this.confidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling
     * estimate(), a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return maxIterations;
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
        this.maxIterations = maxIterations;
    }

    /**
     * Returns list of points to be used to estimate a 2D line.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @return list of points to be used to estimate a 2D line.
     */
    public List<Point2D> getPoints() {
        return points;
    }

    /**
     * Sets list of points to be used to estimate a 2D line.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @param points list of points to be used to estimate a 2D line.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setPoints(final List<Point2D> points) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(points);
    }

    /**
     * Indicates if estimator is ready to start the 2D line estimation.
     * This is true when a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return points != null && points.size() >= MINIMUM_SIZE;
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
     * The larger the score value the better the quality of the point measure.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each sampled point.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate the best
     *               2D line.
     * @return an instance of a 2D line robust estimator.
     */
    public static Line2DRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator();
            case MSAC -> new MSACLine2DRobustEstimator();
            case PROSAC -> new PROSACLine2DRobustEstimator();
            case PROMEDS -> new PROMedSLine2DRobustEstimator();
            default -> new RANSACLine2DRobustEstimator();
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided points and robust estimator method.
     *
     * @param points 2D points to estimate a 2D line.
     * @param method method of a robust estimator algorithm to estimate the best
     *               2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(final List<Point2D> points, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator(points);
            case MSAC -> new MSACLine2DRobustEstimator(points);
            case PROSAC -> new PROSACLine2DRobustEstimator(points);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(points);
            default -> new RANSACLine2DRobustEstimator(points);
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 2D line.
     * @return an instance of a 2D line robust estimator.
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator(listener);
            case MSAC -> new MSACLine2DRobustEstimator(listener);
            case PROSAC -> new PROSACLine2DRobustEstimator(listener);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(listener);
            default -> new RANSACLine2DRobustEstimator(listener);
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener and points.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   2D points to estimate a 2D line.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final List<Point2D> points,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator(listener, points);
            case MSAC -> new MSACLine2DRobustEstimator(listener, points);
            case PROSAC -> new PROSACLine2DRobustEstimator(listener, points);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(listener, points);
            default -> new RANSACLine2DRobustEstimator(listener, points);
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 points).
     */
    public static Line2DRobustEstimator create(final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator();
            case MSAC -> new MSACLine2DRobustEstimator();
            case PROSAC -> new PROSACLine2DRobustEstimator(qualityScores);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(qualityScores);
            default -> new RANSACLine2DRobustEstimator();
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided points and robust estimator method.
     *
     * @param points        2D points to estimate a 2D line.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(
            final List<Point2D> points, final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator(points);
            case MSAC -> new MSACLine2DRobustEstimator(points);
            case PROSAC -> new PROSACLine2DRobustEstimator(points, qualityScores);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(points, qualityScores);
            default -> new RANSACLine2DRobustEstimator(points);
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 points).
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator(listener);
            case MSAC -> new MSACLine2DRobustEstimator(listener);
            case PROSAC -> new PROSACLine2DRobustEstimator(listener, qualityScores);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(listener, qualityScores);
            default -> new RANSACLine2DRobustEstimator(listener);
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener and points.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        2D points to estimate a 2D line.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final List<Point2D> points, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSLine2DRobustEstimator(listener, points);
            case MSAC -> new MSACLine2DRobustEstimator(listener, points);
            case PROSAC -> new PROSACLine2DRobustEstimator(listener, points, qualityScores);
            case PROMEDS -> new PROMedSLine2DRobustEstimator(listener, points, qualityScores);
            default -> new RANSACLine2DRobustEstimator(listener, points);
        };
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * default robust estimator method.
     *
     * @return an instance of a 2D line robust estimator.
     */
    public static Line2DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided points and default robust estimator method.
     *
     * @param points 2D points to estimate a 2D line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(final List<Point2D> points) {
        return create(points, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a 2D line robust estimator.
     */
    public static Line2DRobustEstimator create(final Line2DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener and lines and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   2D points to estimate a line.
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final List<Point2D> points) {
        return create(listener, points, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 points).
     */
    public static Line2DRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided points and default estimator method.
     *
     * @param points        2D points to estimate a 2D line.
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(final List<Point2D> points, final double[] qualityScores) {
        return create(points, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a circle robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 points).
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D line robust estimator based on 2D point samples and using
     * provided listener and points and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        2D points to estimate a 2D line.
     * @param qualityScores quality scores corresponding to each provided point
     * @return an instance of a 2D line robust estimator.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Line2DRobustEstimator create(
            final Line2DRobustEstimatorListener listener, final List<Point2D> points, final double[] qualityScores) {
        return create(listener, points, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Estimates a 2D line using a robust estimator and the best set of 2D
     * points that pass through the estimated 2D line (i.e. belong to its locus)
     *
     * @return a 2D line.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract Line2D estimate() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internal method to set list of 2D points to be used to estimate a 2D
     * line.
     * This method does not check whether estimator is locked or not.
     *
     * @param points list of points to be used to estimate a 2D line.
     * @throws IllegalArgumentException if provided list of points doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    private void internalSetPoints(final List<Point2D> points) {
        if (points.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        this.points = points;
    }

    /**
     * Computes the residual between a 2D point and a line.
     *
     * @param l     a 2D line.
     * @param point a 2D point.
     * @return residual.
     */
    protected double residual(final Line2D l, final Point2D point) {
        l.normalize();
        point.normalize();

        return Math.abs(l.signedDistance(point));
    }
}
