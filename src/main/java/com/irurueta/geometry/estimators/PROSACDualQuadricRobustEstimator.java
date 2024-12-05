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

import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best dual quadric for provided collection of 3D planes using PROSAC
 * algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROSACDualQuadricRobustEstimator extends DualQuadricRobustEstimator {
    /**
     * Constant defining default threshold to determine whether planes are
     * inliers or not.
     * Threshold is defined by the equation abs(trans(P) * dQ * P) &lt; t, where
     * trans is the transposition, P is a plane, dQ is a dual quadric and t is a
     * threshold.
     * This equation determines the planes P belonging to the locus of a dual
     * quadric dQ up to a certain threshold.
     */
    public static final double DEFAULT_THRESHOLD = 1e-7;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether planes are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of algebraic error a possible
     * solution has on a given line.
     */
    private double threshold;

    /**
     * Quality scores corresponding to each plane.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACDualQuadricRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with planes.
     *
     * @param planes 3D planes to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualQuadricRobustEstimator(final List<Plane> planes) {
        super(planes);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACDualQuadricRobustEstimator(final DualQuadricRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }


    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   3D planes to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualQuadricRobustEstimator(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes) {
        super(listener, planes);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 planes).
     */
    public PROSACDualQuadricRobustEstimator(final double[] qualityScores) {
        super();
        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with planes.
     *
     * @param planes        3D planes to estimate a dual quadric.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided list of planes don't have
     *                                  the same size as the list of provided quality scores, or if their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualQuadricRobustEstimator(final List<Plane> planes, final double[] qualityScores) {
        super(planes);

        if (qualityScores.length != planes.size()) {
            throw new IllegalArgumentException();
        }

        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 planes).
     */
    public PROSACDualQuadricRobustEstimator(
            final DualQuadricRobustEstimatorListener listener, final double[] qualityScores) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }


    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        3D planes to estimate a dual quadric.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided list of points don't have
     *                                  the same size as the list of provided quality scores, or it their size
     *                                  is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualQuadricRobustEstimator(
            final DualQuadricRobustEstimatorListener listener, final List<Plane> planes, final double[] qualityScores) {
        super(listener, planes);

        if (qualityScores.length != planes.size()) {
            throw new IllegalArgumentException();
        }

        threshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a
     * given plane.
     *
     * @return threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of algebraic error a possible
     * solution has on a given plane.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     *                                  zero.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        this.threshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each provided plane.
     * The larger the score value the better the quality of the sampled plane.
     *
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided plane.
     * The larger the score value the better the quality of the sampled plane.
     *
     * @param qualityScores quality scores corresponding to each plane.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 9 samples).
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
     * This is true when input data (i.e. 3D planes and quality scores) are
     * provided and a minimum of MINIMUM_SIZE planes are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == planes.size();
    }

    /**
     * Estimates a dual quadric using a robust estimator and the best set of 3D
     * planes that fit into the locus of the estimated dual quadric found using
     * the robust estimator.
     *
     * @return a dual quadric.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public DualQuadric estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<DualQuadric>() {

            @Override
            public double getThreshold() {
                return threshold;
            }

            @Override
            public int getTotalSamples() {
                return planes.size();
            }

            @Override
            public int getSubsetSize() {
                return DualQuadricRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<DualQuadric> solutions) {
                final var plane1 = planes.get(samplesIndices[0]);
                final var plane2 = planes.get(samplesIndices[1]);
                final var plane3 = planes.get(samplesIndices[2]);
                final var plane4 = planes.get(samplesIndices[3]);
                final var plane5 = planes.get(samplesIndices[4]);
                final var plane6 = planes.get(samplesIndices[5]);
                final var plane7 = planes.get(samplesIndices[6]);
                final var plane8 = planes.get(samplesIndices[7]);
                final var plane9 = planes.get(samplesIndices[8]);

                try {
                    final var dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5, plane6, plane7,
                            plane8, plane9);
                    solutions.add(dualQuadric);
                } catch (final CoincidentPlanesException e) {
                    // if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(final DualQuadric currentEstimation, final int i) {
                return residual(currentEstimation, planes.get(i));
            }

            @Override
            public boolean isReady() {
                return PROSACDualQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<DualQuadric> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(PROSACDualQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<DualQuadric> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(PROSACDualQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<DualQuadric> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(PROSACDualQuadricRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<DualQuadric> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(PROSACDualQuadricRobustEstimator.this, progress);
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
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each provided plane.
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
