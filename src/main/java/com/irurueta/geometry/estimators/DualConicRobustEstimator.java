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
import com.irurueta.geometry.DualConic;
import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best dual conic
 * that fits in a collection of 2D lines.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class DualConicRobustEstimator {

    /**
     * Minimum number of 2D lines required to estimate a Dual Conic.
     */
    public static final int MINIMUM_SIZE = 5;

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
    protected DualConicRobustEstimatorListener listener;

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
     * List of lines to be used to estimate a dual conic. Provided list must
     * have a size greater or equal than MINIMUM_SIZE.
     */
    protected List<Line2D> lines;

    /**
     * Matrix representation of a 2D line to be reused when computing
     * residuals.
     */
    private Matrix testLine;

    /**
     * Matrix representation of a dual conic to be reused when computing
     * residuals.
     */
    private Matrix testDualC;

    /**
     * Constructor.
     */
    protected DualConicRobustEstimator() {
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
    protected DualConicRobustEstimator(final DualConicRobustEstimatorListener listener) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
    }

    /**
     * Constructor with lines.
     *
     * @param lines 2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    protected DualConicRobustEstimator(final List<Line2D> lines) {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetLines(lines);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    protected DualConicRobustEstimator(final DualConicRobustEstimatorListener listener, final List<Line2D> lines) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        internalSetLines(lines);
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public DualConicRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final DualConicRobustEstimatorListener listener) throws LockedException {
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
     * Indicates if this instance is locked because estimation is being computed
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
     * (which is equivalent to 100%). The amount of confidence indicates that
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
     * be close to 1.0, but not exactly 1.0
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
     * Returns list of lines to be used to estimate a dual conic.
     * Provided list have a size greater or equal than MINIMUM_SIZE.
     *
     * @return list of lines to be used to estimate a dual conic.
     */
    public List<Line2D> getLines() {
        return lines;
    }

    /**
     * Sets list of lines to be used to estimate a dual conic.
     * Provided list must have a size greater or equal than MINIMUM_SIZE.
     *
     * @param lines list of lines to be used to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines doesn't have
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
     * Indicates if estimator is ready to start the dual conic estimation.
     * This is true when MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return lines != null && lines.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each line.
     * The larger the score value the better the quality of the line measure.
     * This implementation always return null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each line.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each line.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each line.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 samples).
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate bes dual
     *               conic.
     * @return an instance of a dual conic robust estimator.
     */
    public static DualConicRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator();
            case MSAC -> new MSACDualConicRobustEstimator();
            case PROSAC -> new PROSACDualConicRobustEstimator();
            case PROMEDS -> new PROMedSDualConicRobustEstimator();
            default -> new RANSACDualConicRobustEstimator();
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided lines and robust estimator method.
     *
     * @param lines  2D lines to estimate a dual conic.
     * @param method method of a robust estimator algorithm to estimate the best
     *               dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(final List<Line2D> lines, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator(lines);
            case MSAC -> new MSACDualConicRobustEstimator(lines);
            case PROSAC -> new PROSACDualConicRobustEstimator(lines);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(lines);
            default -> new RANSACDualConicRobustEstimator(lines);
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 dual conic.
     * @return an instance of a dual conic robust estimator.
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator(listener);
            case MSAC -> new MSACDualConicRobustEstimator(listener);
            case PROSAC -> new PROSACDualConicRobustEstimator(listener);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(listener);
            default -> new RANSACDualConicRobustEstimator(listener);
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener and lines.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a dual conic.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final List<Line2D> lines,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator(listener, lines);
            case MSAC -> new MSACDualConicRobustEstimator(listener, lines);
            case PROSAC -> new PROSACDualConicRobustEstimator(listener, lines);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(listener, lines);
            default -> new RANSACDualConicRobustEstimator(listener, lines);
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate bes dual
     *                      conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public static DualConicRobustEstimator create(final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator();
            case MSAC -> new MSACDualConicRobustEstimator();
            case PROSAC -> new PROSACDualConicRobustEstimator(qualityScores);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(qualityScores);
            default -> new RANSACDualConicRobustEstimator();
        };
    }

    /**
     * Creates a dual conic robust estimator method based on 2D line samples and
     * using provided lines and robust estimator method.
     *
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(
            final List<Line2D> lines, final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator(lines);
            case MSAC -> new MSACDualConicRobustEstimator(lines);
            case PROSAC -> new PROSACDualConicRobustEstimator(lines, qualityScores);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(lines, qualityScores);
            default -> new RANSACDualConicRobustEstimator(lines);
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator(listener);
            case MSAC -> new MSACDualConicRobustEstimator(listener);
            case PROSAC -> new PROSACDualConicRobustEstimator(listener, qualityScores);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(listener, qualityScores);
            default -> new RANSACDualConicRobustEstimator(listener);
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener and lines.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have
     *                                  the same size as the list of provided quality scores, or it their size is
     *                                  not greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final List<Line2D> lines, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSDualConicRobustEstimator(listener, lines);
            case MSAC -> new MSACDualConicRobustEstimator(listener, lines);
            case PROSAC -> new PROSACDualConicRobustEstimator(listener, lines, qualityScores);
            case PROMEDS -> new PROMedSDualConicRobustEstimator(listener, lines, qualityScores);
            default -> new RANSACDualConicRobustEstimator(listener, lines);
        };
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * default robust estimator method.
     *
     * @return an instance of a dual conic robust estimator.
     */
    public static DualConicRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided lines and default robust estimator method.
     *
     * @param lines 2D lines to estimate a dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines doesn't have a
     *                                  size greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(final List<Line2D> lines) {
        return create(lines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of a dual conic robust estimator.
     */
    public static DualConicRobustEstimator create(final DualConicRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener and lines and default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param lines    2D lines to estimate a dual conic.
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final List<Line2D> lines) {
        return create(listener, lines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each provided line
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public static DualConicRobustEstimator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided lines and default estimator method.
     *
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have the
     *                                  same size as the list of provided quality scores, or if their size is not
     *                                  greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(
            final List<Line2D> lines, final double[] qualityScores) {
        return create(lines, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided line
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual conic robust estimator based on 2D line samples and using
     * provided listener and lines and default estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param lines         2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line
     * @return an instance of a dual conic robust estimator.
     * @throws IllegalArgumentException if provided list of lines don't have the
     *                                  same size as the list of provided quality scores, or if their size is not
     *                                  greater or equal than MINIMUM_SIZE.
     */
    public static DualConicRobustEstimator create(
            final DualConicRobustEstimatorListener listener, final List<Line2D> lines, final double[] qualityScores) {
        return create(listener, lines, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Estimates a dual conic using a robust estimator and the best set of 2D
     * lines that fit into the locus of the estimated dual conic found using the
     * robust estimator.
     *
     * @return a dual conic.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason (i.e.
     *                                  numerical instability, no solution available, etc).
     */
    public abstract DualConic estimate() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Internal method to set list of lines to be used to estimate a dual conic.
     * This method does not check whether estimator is locked or not.
     *
     * @param lines list of lines to be used to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines doesn't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    private void internalSetLines(final List<Line2D> lines) {
        if (lines.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        this.lines = lines;
    }

    /**
     * Computes the residual between a dual conic and a 2D line.
     *
     * @param dc   a dual conic.
     * @param line a 2D line.
     * @return residual.
     */
    protected double residual(final DualConic dc, final Line2D line) {
        dc.normalize();
        try {
            if (testDualC == null) {
                testDualC = dc.asMatrix();
            } else {
                dc.asMatrix(testDualC);
            }

            if (testLine == null) {
                testLine = new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);
            }
            line.normalize();
            testLine.setElementAt(0, 0, line.getA());
            testLine.setElementAt(1, 0, line.getB());
            testLine.setElementAt(2, 0, line.getC());
            final var locusMatrix = testLine.transposeAndReturnNew();
            locusMatrix.multiply(testDualC);
            locusMatrix.multiply(testLine);
            return Math.abs(locusMatrix.getElementAt(0, 0));
        } catch (final AlgebraException e) {
            return Double.MAX_VALUE;
        }
    }
}
