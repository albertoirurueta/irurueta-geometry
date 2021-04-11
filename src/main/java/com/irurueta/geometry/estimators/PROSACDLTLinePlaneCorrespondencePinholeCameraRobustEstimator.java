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
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best pinhole camera for provided collections of matched lines and
 * planes using PROSAC algorithm.
 */
@SuppressWarnings("DuplicatedCode")
public class PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator
        extends DLTLinePlaneCorrespondencePinholeCameraRobustEstimator {

    /**
     * Constant defining default threshold to determine whether planes are
     * inliers or not.
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and lines were
     * equal.
     * A residual of 1 indicates that dot product was 0 and lines were
     * orthogonal.
     * If dot product between lines is -1, then although their director vectors
     * are opposed, lines are considered equal, since sign changes are not taken
     * into account and their residuals will be 0.
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
     * plane respect the backprojected plane of a line using estimated camera.
     */
    private double mThreshold;

    /**
     * Quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     */
    private double[] mQualityScores;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean mComputeAndKeepInliers;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean mComputeAndKeepResiduals;

    /**
     * Constructor.
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor with lists of matched planes and 2D lines to estimate a
     * pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     *
     * @param planes list of planes used to estimate a pinhole camera.
     * @param lines  list of corresponding projected 2D lines used to estimate
     *               a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size or their size is smaller than required minimum size (4 matches).
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final List<Plane> planes, final List<Line2D> lines) {
        super(planes, lines);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor with listener and lists of matched planes and 2D lines to
     * estimate a pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   list of planes used to estimate a pinhole camera.
     * @param lines    list of corresponding projected 2D lines used to estimate
     *                 a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size or their size is smaller than required minimum size (4 matches).
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Plane> planes, final List<Line2D> lines) {
        super(listener, planes, lines);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES (i.e. 4 samples).
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final double[] qualityScores) {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with lists of matched planes and 2D lines to estimate a
     * pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     *
     * @param planes        list of planes used to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided lists or quality scores
     *                                  don't have the same size or their size is smaller than required minimum
     *                                  size (4 matches).
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final List<Plane> planes, final List<Line2D> lines, final double[] qualityScores) {
        super(planes, lines);

        if (qualityScores.length != planes.size()) {
            throw new IllegalArgumentException();
        }

        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with listener.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES (i.e. 4 samples).
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final double[] qualityScores) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with listener and lists of matched planes and 2D lines to
     * estimate a pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        list of planes used to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size or their size is smaller than required minimum size (4 matches).
     */
    public PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Plane> planes, final List<Line2D> lines, final double[] qualityScores) {
        super(listener, planes, lines);

        if (qualityScores.length != planes.size()) {
            throw new IllegalArgumentException();
        }

        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a
     * plane respect the backprojected plane of a line using estimated camera
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
     * @return threshold to determine whether matched planes are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a
     * plane respect the backprojected plane of a line using estimated camera
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
        mThreshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     *
     * @return quality scores corresponding to each pair of matched points.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES (i.e. 4 samples).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if estimator is ready to start the affine 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points and quality
     * scores) are provided and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mPlanes.size();
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return mComputeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResidualsEnabled() {
        return mComputeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and
     *                                kept, false if residuals only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(
            final boolean computeAndKeepResiduals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Estimates a pinhole camera using a robust estimator and
     * the best set of matched 2D line/3D plane correspondences found using the
     * robust estimator.
     *
     * @return a pinhole camera.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public PinholeCamera estimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        // pinhole camera estimator using DLT (Direct Linear Transform) algorithm
        final DLTLinePlaneCorrespondencePinholeCameraEstimator nonRobustEstimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        nonRobustEstimator.setLMSESolutionAllowed(false);

        // suggestions
        nonRobustEstimator.setSuggestSkewnessValueEnabled(
                isSuggestSkewnessValueEnabled());
        nonRobustEstimator.setSuggestedSkewnessValue(
                getSuggestedSkewnessValue());
        nonRobustEstimator.setSuggestHorizontalFocalLengthEnabled(
                isSuggestHorizontalFocalLengthEnabled());
        nonRobustEstimator.setSuggestedHorizontalFocalLengthValue(
                getSuggestedHorizontalFocalLengthValue());
        nonRobustEstimator.setSuggestVerticalFocalLengthEnabled(
                isSuggestVerticalFocalLengthEnabled());
        nonRobustEstimator.setSuggestedVerticalFocalLengthValue(
                getSuggestedVerticalFocalLengthValue());
        nonRobustEstimator.setSuggestAspectRatioEnabled(
                isSuggestAspectRatioEnabled());
        nonRobustEstimator.setSuggestedAspectRatioValue(
                getSuggestedAspectRatioValue());
        nonRobustEstimator.setSuggestPrincipalPointEnabled(
                isSuggestPrincipalPointEnabled());
        nonRobustEstimator.setSuggestedPrincipalPointValue(
                getSuggestedPrincipalPointValue());
        nonRobustEstimator.setSuggestRotationEnabled(
                isSuggestRotationEnabled());
        nonRobustEstimator.setSuggestedRotationValue(
                getSuggestedRotationValue());
        nonRobustEstimator.setSuggestCenterEnabled(isSuggestCenterEnabled());
        nonRobustEstimator.setSuggestedCenterValue(getSuggestedCenterValue());

        final PROSACRobustEstimator<PinholeCamera> innerEstimator =
                new PROSACRobustEstimator<>(
                        new PROSACRobustEstimatorListener<PinholeCamera>() {

                            // 3D planes for a subset of samples
                            private final List<Plane> mSubsetPlanes = new ArrayList<>();

                            // 2D lines for a subset of samples
                            private final List<Line2D> mSubsetLines = new ArrayList<>();

                            @Override
                            public double getThreshold() {
                                return mThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mPlanes.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return LinePlaneCorrespondencePinholeCameraEstimator.
                                        MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<PinholeCamera> solutions) {
                                mSubsetPlanes.clear();
                                mSubsetPlanes.add(mPlanes.get(samplesIndices[0]));
                                mSubsetPlanes.add(mPlanes.get(samplesIndices[1]));
                                mSubsetPlanes.add(mPlanes.get(samplesIndices[2]));
                                mSubsetPlanes.add(mPlanes.get(samplesIndices[3]));

                                mSubsetLines.clear();
                                mSubsetLines.add(mLines.get(samplesIndices[0]));
                                mSubsetLines.add(mLines.get(samplesIndices[1]));
                                mSubsetLines.add(mLines.get(samplesIndices[2]));
                                mSubsetLines.add(mLines.get(samplesIndices[3]));

                                try {
                                    nonRobustEstimator.setLists(mSubsetPlanes, mSubsetLines);

                                    final PinholeCamera cam = nonRobustEstimator.estimate();
                                    solutions.add(cam);
                                } catch (final Exception e) {
                                    // if lines/planes configuration is degenerate, no solution
                                    // is added
                                }
                            }

                            @Override
                            public double computeResidual(final PinholeCamera currentEstimation,
                                                          final int i) {
                                final Line2D inputLine = mLines.get(i);
                                final Plane inputPlane = mPlanes.get(i);

                                return singleBackprojectionResidual(currentEstimation,
                                        inputLine, inputPlane);
                            }

                            @Override
                            public boolean isReady() {
                                return PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                                        this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<PinholeCamera> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<PinholeCamera> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<PinholeCamera> estimator, final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<PinholeCamera> estimator, final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(PROSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this,
                                            progress);
                                }
                            }

                            @Override
                            public double[] getQualityScores() {
                                return mQualityScores;
                            }
                        });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(
                    mComputeAndKeepInliers || mRefineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(
                    mComputeAndKeepResiduals || mRefineResult);
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final PinholeCamera result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result,
                    nonRobustEstimator.getMaxSuggestionWeight());

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
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
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     *
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        return mThreshold;
    }

    /**
     * Sets quality scores corresponding to each pair of matched points.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
