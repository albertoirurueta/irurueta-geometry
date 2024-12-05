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
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Finds the best affine 3D transformation for provided collections of matched
 * planes using RANSAC algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator
        extends PlaneCorrespondenceAffineTransformation3DRobustEstimator {

    /**
     * Constant defining default threshold to determine whether planes are
     * inliers or not.
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and planes were
     * equal.
     * A residual of 1 indicates that dot product was 0 and planes were
     * orthogonal.
     * If dot product between planes is -1, then although their director vectors
     * are opposed, planes are considered equal, since sign changes are not
     * taken into account and their residuals will be 0.
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether planes are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a
     * matched pair of lines.
     */
    private double threshold;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean computeAndKeepInliers;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean computeAndKeepResiduals;

    /**
     * Constructor.
     */
    public RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor with lists of planes to be used to estimate an affine 3D
     * transformation.
     * Lines in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPlanes  list of input planes to be used to estimate an affine
     *                     3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an affine
     *                     3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        super(inputPlanes, outputPlanes);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
            final AffineTransformation3DRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor with listener and lists of planes to be used to estimate an
     * affine 3D transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param inputPlanes  list of input planes to be used to estimate an affine
     *                     3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an
     *                     affine 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     */
    public RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
            final AffineTransformation3DRobustEstimatorListener listener,
            final List<Plane> inputPlanes, final List<Plane> outputPlanes) {
        super(listener, inputPlanes, outputPlanes);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Returns threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and lines were
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were
     * orthogonal.
     * If dot product between planes is -1, then although their director vectors
     * are opposed, planes are considered equal, since sign changes are not
     * taken into account and their residuals will be 0.
     *
     * @return threshold to determine whether matched planes are inliers or not.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and planes were
     * equal.
     * A residual of 1 indicates that dot product was 0 and planes were
     * orthogonal.
     * If dot product between planes is -1, then although their director vectors
     * are opposed, planes are considered equal, since sign changes are not
     * taken into account and their residuals will be 0.
     *
     * @param threshold threshold to determine whether matched planes are
     *                  inliers or not.
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
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers only
     * need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return computeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.computeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResidualsEnabled() {
        return computeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and
     *                                kept, false if residuals only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.computeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Estimates an affine 3D transformation using a robust estimator and
     * the best set of matched 3D lines correspondences found using the robust
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
    @Override
    public AffineTransformation3D estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new RANSACRobustEstimator<>(
                new RANSACRobustEstimatorListener<AffineTransformation3D>() {

                    // plane to be reused when computing residuals
                    private final Plane testPlane = new Plane();

                    @Override
                    public double getThreshold() {
                        return threshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return inputPlanes.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return AffineTransformation3DRobustEstimator.MINIMUM_SIZE;
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices, final List<AffineTransformation3D> solutions) {
                        final var inputPlane1 = inputPlanes.get(samplesIndices[0]);
                        final var inputPlane2 = inputPlanes.get(samplesIndices[1]);
                        final var inputPlane3 = inputPlanes.get(samplesIndices[2]);
                        final var inputPlane4 = inputPlanes.get(samplesIndices[3]);

                        final var outputPlane1 = outputPlanes.get(samplesIndices[0]);
                        final var outputPlane2 = outputPlanes.get(samplesIndices[1]);
                        final var outputPlane3 = outputPlanes.get(samplesIndices[2]);
                        final var outputPlane4 = outputPlanes.get(samplesIndices[3]);

                        try {
                            final var transformation = new AffineTransformation3D(inputPlane1, inputPlane2, inputPlane3,
                                    inputPlane4, outputPlane1, outputPlane2, outputPlane3, outputPlane4);
                            solutions.add(transformation);
                        } catch (final CoincidentPlanesException e) {
                            // if lines are coincident, no solution is added
                        }
                    }

                    @Override
                    public double computeResidual(final AffineTransformation3D currentEstimation, final int i) {
                        final var inputPlane = inputPlanes.get(i);
                        final var outputPlane = outputPlanes.get(i);

                        // transform input plane and store result in mTestPlane
                        try {
                            currentEstimation.transform(inputPlane, testPlane);

                            return getResidual(outputPlane, testPlane);
                        } catch (final AlgebraException e) {
                            // this happens when internal matrix of affine transformation
                            // cannot be reversed (i.e. transformation is not well-defined,
                            // numerical instabilities, etc.)
                            return Double.MAX_VALUE;
                        }
                    }

                    @Override
                    public boolean isReady() {
                        return RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<AffineTransformation3D> estimator) {
                        if (listener != null) {
                            listener.onEstimateStart(
                                    RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<AffineTransformation3D> estimator) {
                        if (listener != null) {
                            listener.onEstimateEnd(
                                    RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<AffineTransformation3D> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onEstimateNextIteration(
                                    RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<AffineTransformation3D> estimator, final float progress) {
                        if (listener != null) {
                            listener.onEstimateProgressChange(
                                    RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this,
                                    progress);
                        }
                    }
                });

        try {
            locked = true;
            inliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(computeAndKeepInliers || refineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(computeAndKeepResiduals || refineResult);
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            final var transformation = innerEstimator.estimate();
            inliersData = innerEstimator.getInliersData();
            return attemptRefine(transformation);
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
        return RobustEstimatorMethod.RANSAC;
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
    @Override
    protected double getRefinementStandardDeviation() {
        return threshold;
    }
}
