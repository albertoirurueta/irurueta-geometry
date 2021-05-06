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

import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.refiners.LineCorrespondenceAffineTransformation2DRefiner;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best affine
 * 2D transformation for collections of matching 2D lines.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class LineCorrespondenceAffineTransformation2DRobustEstimator extends
        AffineTransformation2DRobustEstimator {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * List of lines to be used to estimate an affine 2D transformation.
     * Each line in the list of input lines must be matched with the
     * corresponding line in the list of output lines located at the same
     * position. Hence, both input lines and output lines must have the
     * same size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Line2D> mInputLines;

    /**
     * List of lines to be used to estimate an affine 2D transformation.
     * Each point in the list of output lines must be matched with the
     * corresponding line in the list of input lines located at the same
     * position. Hence, both input lines and output lines must have the
     * same size, and their size must be greater or equal than MINIMUM_SIZE.
     */
    protected List<Line2D> mOutputLines;

    /**
     * Constructor.
     */
    protected LineCorrespondenceAffineTransformation2DRobustEstimator() {
        super();
    }

    /**
     * Constructor with lists of lines to be used to estimate an affine 2D
     * transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputLines  list of input lines to be used to estimate an affine 2D
     *                    transformation.
     * @param outputLines list of output lines ot be used to estimate an affine
     *                    2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected LineCorrespondenceAffineTransformation2DRobustEstimator(
            final List<Line2D> inputLines, final List<Line2D> outputLines) {
        super();
        internalSetLines(inputLines, outputLines);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 stars, ends or its progress significantly changes.
     */
    protected LineCorrespondenceAffineTransformation2DRobustEstimator(
            final AffineTransformation2DRobustEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor with listener and lists of lines to be used to estimate
     * affine 2D transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener    listener to be notified of events such as when estimation
     *                    starts, ends or its progress significantly changes.
     * @param inputLines  list of input lines to be used to estimate an affine
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate an affine
     *                    2D transformation.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    protected LineCorrespondenceAffineTransformation2DRobustEstimator(
            final AffineTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines) {
        super(listener);
        internalSetLines(inputLines, outputLines);
    }

    /**
     * Returns list of input lines to be used to estimate an affine 2D
     * transformation.
     * Each line in the list of input lines must be matched with the
     * corresponding line in the list of output lines located at the same
     * position. Hence, both input lines and output lines must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of input lines to be used to estimate an affine 2D
     * transformation.
     */
    public List<Line2D> getInputLines() {
        return mInputLines;
    }

    /**
     * Returns list of output lines to be used to estimate an affine 2D
     * transformation.
     * Each line in the list of output lines must be matched with the
     * corresponding line in the list of input lines located at the same
     * position. Hence, both input lines and output lines must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of output lines to be used to estimate an affine 2D
     * transformation.
     */
    public List<Line2D> getOutputLines() {
        return mOutputLines;
    }

    /**
     * Sets lists of lines to be used to estimate an affine 2D transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputLines  list of input lines to be used to estimate an affine
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate an affine
     *                    2D transformation.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public final void setLines(final List<Line2D> inputLines,
                               final List<Line2D> outputLines) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetLines(inputLines, outputLines);
    }

    /**
     * Indicates if estimator is ready to start the affine 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched lines) are provided
     * and a minimum of MINIMUM_SIZE lines are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputLines != null && mOutputLines != null &&
                mInputLines.size() == mOutputLines.size() &&
                mInputLines.size() >= MINIMUM_SIZE;
    }

    /**
     * Returns quality scores corresponding to each pair of matched lines.
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
     * Sets quality scores corresponding to each pair of matched lines.
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
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences an using provided robust estimator method.
     *
     * @param method method of a robust estimator algorithm to estimate
     *               best affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator();
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator();
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator();
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param inputLines  list of input lines to be used to estimate an
     *                    affine 2D transformation.
     * @param outputLines list of output lines to be used to estimate an
     *                    affine 2D transformation.
     * @param method      method of a robust estimator algorithm to estimate
     *                    best affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   method of a robust estimator algorithm to estimate best
     *                 affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param listener    listener to be notified of events such as when estimation
     *                    starts, ends or its progress significantly changes.
     * @param inputLines  list of input lines to be used to estimate an affine
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate an affine
     *                    2D transformation.
     * @param method      method of a robust estimator algorithm to estimate best
     *                    affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator();
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator();
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        qualityScores);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        qualityScores);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param inputLines    list of input lines to be used to estimate an
     *                      affine 2D transformation.
     * @param outputLines   list of output lines to be used to estimate an
     *                      affine 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines, qualityScores);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines, qualityScores);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        inputLines, outputLines);
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, qualityScores);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, qualityScores);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener);
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using provided robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputLines    list of input lines to be used to estimate an affine
     *                      2D transformation.
     * @param outputLines   list of output lines to be used to estimate an affine
     *                      2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
            case MSAC:
                return new MSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
            case PROSAC:
                return new PROSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines, qualityScores);
            case PROMedS:
                return new PROMedSLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines, qualityScores);
            case RANSAC:
            default:
                return new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                        listener, inputLines, outputLines);
        }
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param inputLines  list of input lines to be used to estimate an
     *                    affine 2D transformation.
     * @param outputLines list of output lines to be used to estimate an
     *                    affine 2D transformation.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final List<Line2D> inputLines, final List<Line2D> outputLines) {
        return create(inputLines, outputLines, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param listener    listener to be notified of events such as when estimation
     *                    starts, ends or its progress significantly changes.
     * @param inputLines  list of input lines to be used to estimate an affine
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate an affine
     *                    2D transformation.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines) {
        return create(listener, inputLines, outputLines,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param inputLines    list of input lines to be used to estimate an affine
     *                      2D transformation.
     * @param outputLines   list of output lines to be used to estimate an affine
     *                      2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final double[] qualityScores) {
        return create(inputLines, outputLines, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of affine 2D transformation estimator.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an affine 2D transformation estimator based on 2D line
     * correspondences and using default robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param inputLines    list of input lines to be used to estimate an affine
     *                      2D transformation.
     * @param outputLines   list of output lines to be used to estimate an affine
     *                      2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      lines.
     * @return an instance of affine 2D transformation estimator.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public static LineCorrespondenceAffineTransformation2DRobustEstimator create(
            final AffineTransformation2DRobustEstimatorListener listener,
            final List<Line2D> inputLines, final List<Line2D> outputLines,
            final double[] qualityScores) {
        return create(listener, inputLines, outputLines, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Internal method to set lists of lines to be used to estimate an affine
     * 2D transformation.
     * This method does not check whether estimator is locked or not.
     *
     * @param inputLines  list of input lines to be used to estimate an affine
     *                    2D transformation.
     * @param outputLines list of output lines to be used to estimate an affine
     *                    2D transformation.
     * @throws IllegalArgumentException if provided lists of lines don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    private void internalSetLines(final List<Line2D> inputLines,
                                  final List<Line2D> outputLines) {
        if (inputLines.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        if (inputLines.size() != outputLines.size()) {
            throw new IllegalArgumentException();
        }
        mInputLines = inputLines;
        mOutputLines = outputLines;
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
     * @param line            originally sampled output line line.
     * @param transformedLine estimated output line obtained after using
     *                        estimated transformation.
     * @return computed residual.
     */
    protected static double getResidual(final Line2D line, final Line2D transformedLine) {
        line.normalize();
        transformedLine.normalize();

        final double dotProduct = Math.abs(line.getA() * transformedLine.getA() +
                line.getB() * transformedLine.getB() +
                line.getC() * transformedLine.getC());
        return 1.0 - dotProduct;
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
    protected AffineTransformation2D attemptRefine(
            final AffineTransformation2D transformation) {
        if (mRefineResult) {
            final LineCorrespondenceAffineTransformation2DRefiner refiner =
                    new LineCorrespondenceAffineTransformation2DRefiner(
                            transformation, mKeepCovariance, getInliersData(),
                            mInputLines, mOutputLines,
                            getRefinementStandardDeviation());

            try {
                final AffineTransformation2D result = new AffineTransformation2D();
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
