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
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.refiners.HomogeneousPoint2DRefiner;
import com.irurueta.geometry.refiners.InhomogeneousPoint2DRefiner;
import com.irurueta.geometry.refiners.Point2DRefiner;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best 3D point
 * that intersects in a collection of 2D lines.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class Point2DRobustEstimator {
    /**
     * Minimum number of 2D lines required to estimate a point.
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
    protected Point2DRobustEstimatorListener listener;

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
     * List of lines to be used to estimate a 2D point. Provided list must have
     * a size greater or equal than MINIMUM_SIZE.
     */
    protected List<Line2D> lines;

    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData inliersData;

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     */
    protected boolean refineResult;

    /**
     * Coordinates type to use for refinement. When using inhomogeneous
     * coordinates a 3x3 covariance matrix is estimated. When using homogeneous
     * coordinates a 4x4 covariance matrix is estimated.
     */
    private CoordinatesType refinementCoordinatesType = CoordinatesType.INHOMOGENEOUS_COORDINATES;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean keepCovariance;

    /**
     * Estimated covariance of estimated 2D point.
     * This is only available when result has been refined and covariance is
     * kept.
     */
    private Matrix covariance;

    /**
     * Constructor.
     */
    protected Point2DRobustEstimator() {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected Point2DRobustEstimator(final Point2DRobustEstimatorListener listener) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor with lines.
     *
     * @param lines 2D lines to estimate a 2D point.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected Point2DRobustEstimator(final List<Line2D> lines) {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetLines(lines);
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor.
     *
     * @param lines    2D lines to estimate a 2D point.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected Point2DRobustEstimator(final Point2DRobustEstimatorListener listener, final List<Line2D> lines) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetLines(lines);
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }


    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public Point2DRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final Point2DRobustEstimatorListener listener) throws LockedException {
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
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return inliersData;
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
        return refineResult;
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
        this.refineResult = refineResult;
    }

    /**
     * Gets coordinates type to use for refinement. When using inhomogeneous
     * coordinates a 3x3 covariance matrix is estimated. When using homogeneous
     * coordinates a 4x4 covariance matrix is estimated.
     *
     * @return coordinates type to use for refinement.
     */
    public CoordinatesType getRefinementCoordinatesType() {
        return refinementCoordinatesType;
    }

    /**
     * Sets coordinates type to use for refinement. When using inhomogeneous
     * coordinates a 3x3 covariance matrix is estimated. When using homogeneous
     * coordinates a 4x4 covariance matrix is estimated.
     *
     * @param refinementCoordinatesType coordinates type to use for refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementCoordinatesType(final CoordinatesType refinementCoordinatesType) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refinementCoordinatesType = refinementCoordinatesType;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false
     * otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining
     *                       result, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }

    /**
     * Returns list of lines to be used to estimate a 2D point.
     * Provided list must have a size greater or equal than MINIMUM_SIZE
     *
     * @return list of lines to be used to estimate a 2D point.
     */
    public List<Line2D> getLines() {
        return lines;
    }

    /**
     * Sets list of lines to be used to estimate a 2D point.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @param lines list of lines to be used to estimate a 2D point.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setLines(final List<Line2D> lines) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetLines(lines);
    }

    /**
     * Indicates if estimator is ready to start the 2D point estimation.
     * This is true when a minimum if MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return lines != null && lines.size() >= MINIMUM_SIZE;
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
     *                                  smaller than MINIMUM_SIZE (i.e. 2 samples).
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
        return covariance;
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate the best
     *               2D point.
     * @return an instance of a 2D point robust estimator.
     */
    public static Point2DRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator();
            case MSAC -> new MSACPoint2DRobustEstimator();
            case PROSAC -> new PROSACPoint2DRobustEstimator();
            case PROMEDS -> new PROMedSPoint2DRobustEstimator();
            default -> new RANSACPoint2DRobustEstimator();
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided lines and robust estimator method.
     *
     * @param lines  2D lines to estimate a 2D point.
     * @param method method of a robust estimator algorithm to estimate the best
     *               2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(final List<Line2D> lines, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator(lines);
            case MSAC -> new MSACPoint2DRobustEstimator(lines);
            case PROSAC -> new PROSACPoint2DRobustEstimator(lines);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(lines);
            default -> new RANSACPoint2DRobustEstimator(lines);
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 2D point.
     * @return an instance of a 2D point robust estimator.
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator(listener);
            case MSAC -> new MSACPoint2DRobustEstimator(listener);
            case PROSAC -> new PROSACPoint2DRobustEstimator(listener);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(listener);
            default -> new RANSACPoint2DRobustEstimator(listener);
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener and lines.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a 2D point.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final List<Line2D> lines,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator(listener, lines);
            case MSAC -> new MSACPoint2DRobustEstimator(listener, lines);
            case PROSAC -> new PROSACPoint2DRobustEstimator(listener, lines);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(listener, lines);
            default -> new RANSACPoint2DRobustEstimator(listener, lines);
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 lines).
     */
    public static Point2DRobustEstimator create(final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator();
            case MSAC -> new MSACPoint2DRobustEstimator();
            case PROSAC -> new PROSACPoint2DRobustEstimator(qualityScores);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(qualityScores);
            default -> new RANSACPoint2DRobustEstimator();
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided lines and robust estimator method.
     *
     * @param lines         2D lines to estimate a 2D point.
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(
            final List<Line2D> lines, final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator(lines);
            case MSAC -> new MSACPoint2DRobustEstimator(lines);
            case PROSAC -> new PROSACPoint2DRobustEstimator(lines, qualityScores);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(lines, qualityScores);
            default -> new RANSACPoint2DRobustEstimator(lines);
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 lines).
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator(listener);
            case MSAC -> new MSACPoint2DRobustEstimator(listener);
            case PROSAC -> new PROSACPoint2DRobustEstimator(listener, qualityScores);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(listener, qualityScores);
            default -> new RANSACPoint2DRobustEstimator(listener);
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener and lines.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param lines         2D lines to estimate a 2D point.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final List<Line2D> lines, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSPoint2DRobustEstimator(listener, lines);
            case MSAC -> new MSACPoint2DRobustEstimator(listener, lines);
            case PROSAC -> new PROSACPoint2DRobustEstimator(listener, lines, qualityScores);
            case PROMEDS -> new PROMedSPoint2DRobustEstimator(listener, lines, qualityScores);
            default -> new RANSACPoint2DRobustEstimator(listener, lines);
        };
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * default robust estimator method.
     *
     * @return an instance of a 2D point robust estimator.
     */
    public static Point2DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided lines and default robust estimator method.
     *
     * @param lines 2D lines to estimate a 2D point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(final List<Line2D> lines) {
        return create(lines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a 2D point robust estimator.
     */
    public static Point2DRobustEstimator create(final Point2DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener and lines and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a point.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final List<Line2D> lines) {
        return create(listener, lines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided line
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 lines).
     */
    public static Point2DRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided lines and default estimator method.
     *
     * @param lines         2D lines to estimate a 2D point.
     * @param qualityScores quality scores corresponding to each provided line.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(final List<Line2D> lines, final double[] qualityScores) {
        return create(lines, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided line
     * @return an instance of a circle robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 2 lines).
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a 2D point robust estimator based on 2D line samples and using
     * provided listener and lines and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param lines         2D lines to estimate a 2D point.
     * @param qualityScores quality scores corresponding to each provided line.
     * @return an instance of a 2D point robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static Point2DRobustEstimator create(
            final Point2DRobustEstimatorListener listener, final List<Line2D> lines, final double[] qualityScores) {
        return create(listener, lines, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Estimates a 2D point using a robust estimator and the best set of 2D
     * lines that intersect into the estimated 2D point.
     *
     * @return a 2D point.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract Point2D estimate() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Computes the residual between a 2D point and a line.
     *
     * @param p    a 2D point.
     * @param line a 2D line.
     * @return residual.
     */
    protected double residual(final Point2D p, final Line2D line) {
        p.normalize();
        line.normalize();

        return Math.abs(line.signedDistance(p));
    }

    /**
     * Attempts to refine provided solution if refinement is requested.
     * This method returns a refined solution or the same provided solution
     * if refinement is not requested or has failed.
     * If refinement is enabled, and it is requested to keep covariance, this
     * method will also keep covariance of refined point.
     *
     * @param point point estimated by a robust estimator without refinement.
     * @return solution after refinement (if requested) or the provided non-refined
     * solution if not requested or if refinement failed.
     */
    protected Point2D attemptRefine(final Point2D point) {
        if (refineResult) {
            try {
                final Point2DRefiner<? extends Point2D> refiner;
                final Point2D result;
                final boolean improved;
                switch (refinementCoordinatesType) {
                    case HOMOGENEOUS_COORDINATES:
                        final HomogeneousPoint2D homP;
                        if (point.getType() == CoordinatesType.HOMOGENEOUS_COORDINATES) {
                            homP = (HomogeneousPoint2D) point;
                        } else {
                            homP = new HomogeneousPoint2D(point);
                        }
                        final var homRefiner = new HomogeneousPoint2DRefiner(homP, keepCovariance, getInliersData(),
                                lines, getRefinementStandardDeviation());
                        refiner = homRefiner;
                        final var homResult = new HomogeneousPoint2D();
                        improved = homRefiner.refine(homResult);
                        result = homResult;
                        break;
                    case INHOMOGENEOUS_COORDINATES:
                    default:
                        InhomogeneousPoint2D inhomP;
                        if (point.getType() == CoordinatesType.INHOMOGENEOUS_COORDINATES) {
                            inhomP = (InhomogeneousPoint2D) point;
                        } else {
                            inhomP = new InhomogeneousPoint2D(point);
                        }
                        final var inhomRefiner = new InhomogeneousPoint2DRefiner(inhomP, keepCovariance,
                                getInliersData(), lines, getRefinementStandardDeviation());
                        refiner = inhomRefiner;
                        final var inhomResult = new InhomogeneousPoint2D();
                        improved = inhomRefiner.refine(inhomResult);
                        result = inhomResult;
                        break;
                }

                if (keepCovariance) {
                    // keep covariance
                    covariance = refiner.getCovariance();
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
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     *
     * @return standard deviation used for refinement.
     */
    protected abstract double getRefinementStandardDeviation();

    /**
     * Internal method to set list of 2D lines to be used to estimate a 2D
     * point.
     * This method does not check whether estimator is locked or not
     *
     * @param lines list of lines to be used to estimate a 2D point
     * @throws IllegalArgumentException if provided list of lines doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    private void internalSetLines(final List<Line2D> lines) {
        if (lines.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        this.lines = lines;
    }
}
