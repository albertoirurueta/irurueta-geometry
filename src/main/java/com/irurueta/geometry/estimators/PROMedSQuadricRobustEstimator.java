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

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quadric;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best quadric for provided collection of 3D points using PROMedS
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROMedSQuadricRobustEstimator extends QuadricRobustEstimator {
    /**
     * Default value to be used for stop threshold. Stop threshold can be used
     * to keep the algorithm iterating in case that best estimated threshold
     * using median of residuals is not small enough. Once a solution is found
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-6;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double stopThreshold;

    /**
     * Quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROMedSQuadricRobustEstimator() {
        super();
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with points.
     *
     * @param points 3D points to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROMedSQuadricRobustEstimator(final List<Point3D> points) {
        super(points);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSQuadricRobustEstimator(final QuadricRobustEstimatorListener listener) {
        super(listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points   3D points to estimate a quadric.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROMedSQuadricRobustEstimator(final QuadricRobustEstimatorListener listener, final List<Point3D> points) {
        super(listener, points);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 points).
     */
    public PROMedSQuadricRobustEstimator(final double[] qualityScores) {
        super();
        stopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points.
     *
     * @param points        3D points to estimate a quadric.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROMedSQuadricRobustEstimator(final List<Point3D> points, final double[] qualityScores) {
        super(points);

        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }

        stopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 points).
     */
    public PROMedSQuadricRobustEstimator(final QuadricRobustEstimatorListener listener, final double[] qualityScores) {
        super(listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }


    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points        3D points to estimate a quadric.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROMedSQuadricRobustEstimator(
            final QuadricRobustEstimatorListener listener, final List<Point3D> points, final double[] qualityScores) {
        super(listener, points);

        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }

        stopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * As in LMedS, the stop threshold can be used to prevent the PROMedS
     * algorithm iterating too many times in cases where samples have a very
     * similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return stopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * As in LMedS, the stop threshold can be used to prevent the PROMedS
     * algorithm iterating too many times in cases where samples have a very
     * similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        this.stopThreshold = stopThreshold;
    }

    /**
     * Returns quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     *
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     *
     * @param qualityScores quality scores corresponding to each point.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 5 samples).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if estimator is ready to start the quadric estimation.
     * This is true when input data (i.e. 3D points and quality scores) are
     * provided and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == points.size();
    }

    /**
     * Estimates a quadric using a robust estimator and the best set of 3D points
     * that fit into the locus of the estimated quadric found using the robust
     * estimator.
     *
     * @return a quadric.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Quadric estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROMedSRobustEstimator<>(new PROMedSRobustEstimatorListener<Quadric>() {

            @Override
            public double getThreshold() {
                return stopThreshold;
            }

            @Override
            public int getTotalSamples() {
                return points.size();
            }

            @Override
            public int getSubsetSize() {
                return QuadricRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Quadric> solutions) {
                final var point1 = points.get(samplesIndices[0]);
                final var point2 = points.get(samplesIndices[1]);
                final var point3 = points.get(samplesIndices[2]);
                final var point4 = points.get(samplesIndices[3]);
                final var point5 = points.get(samplesIndices[4]);
                final var point6 = points.get(samplesIndices[5]);
                final var point7 = points.get(samplesIndices[6]);
                final var point8 = points.get(samplesIndices[7]);
                final var point9 = points.get(samplesIndices[8]);

                try {
                    final var quadric = new Quadric(point1, point2, point3, point4, point5, point6, point7, point8,
                            point9);
                    solutions.add(quadric);
                } catch (final CoincidentPointsException e) {
                    // if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(final Quadric currentEstimation, final int i) {
                return residual(currentEstimation, points.get(i));
            }

            @Override
            public boolean isReady() {
                return PROMedSQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Quadric> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(PROMedSQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Quadric> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(PROMedSQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Quadric> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(PROMedSQuadricRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Quadric> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(PROMedSQuadricRobustEstimator.this, progress);
                }
            }

            @Override
            public double[] getQualityScores() {
                return qualityScores;
            }
        });

        try {
            locked = true;
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            return innerEstimator.estimate();
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMEDS;
    }

    /**
     * Sets quality scores corresponding to each provided point.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
