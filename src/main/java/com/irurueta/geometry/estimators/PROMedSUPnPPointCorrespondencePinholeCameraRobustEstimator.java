/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best pinhole camera for provided collections of matched 2D/3D
 * points using PROMedS + UPnP algorithms.
 */
@SuppressWarnings("DuplicatedCode")
public class PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator extends
        UPnPPointCorrespondencePinholeCameraRobustEstimator {

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
    public static final double DEFAULT_STOP_THRESHOLD = 1.0;

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
    private double mStopThreshold;

    /**
     * Quality scores corresponding to each pair of matched points.
     * The larger the score value the betther the quality of the matching.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(points3D, points2D);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with listener and lists of points to be used ot estimate a
     * pinhole camera.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        super(listener, points3D, points2D);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final double[] qualityScores) {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     *
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided lists of points and array
     *                                  of quality scores don't have the same size or their size is smaller than
     *                                  6 correspondences.
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D,
            final double[] qualityScores) {
        super(points3D, points2D);

        if (qualityScores.length != points3D.size()) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final double[] qualityScores) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with listener and lists of points to be used ot estimate a
     * pinhole camera.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than
     *                                  MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     */
    public PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D,
            final double[] qualityScores) {
        super(listener, points3D, points2D);

        if (qualityScores.length != points3D.size()) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = DEFAULT_STOP_THRESHOLD;
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
        return mStopThreshold;
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

        mStopThreshold = stopThreshold;
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
     *                                  smaller than MINIMUM_SIZE (i.e. 6 samples).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if eatimator is ready to start the affine 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points and quality
     * scores) are provided and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mPoints3D.size();
    }

    /**
     * Estimates an affine 2D transformation using a robust estimator and
     * the best set of matched 2D point correspondences found using the robust
     * estimator.
     *
     * @return an affine 2D transformation.
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

        // pinhole camera estimator using UPnP (Uncalibrated PErspective-n-Point)
        // algorithm
        final UPnPPointCorrespondencePinholeCameraEstimator nonRobustEstimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();

        nonRobustEstimator.setPlanarConfigurationAllowed(
                mPlanarConfigurationAllowed);
        nonRobustEstimator.setNullspaceDimension2Allowed(
                mNullspaceDimension2Allowed);
        nonRobustEstimator.setPlanarThreshold(mPlanarThreshold);
        nonRobustEstimator.setSkewness(mSkewness);
        nonRobustEstimator.setHorizontalPrincipalPoint(
                mHorizontalPrincipalPoint);
        nonRobustEstimator.setVerticalPrincipalPoint(mVerticalPrincipalPoint);

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
        nonRobustEstimator.setSuggestCenterEnabled(
                isSuggestCenterEnabled());
        nonRobustEstimator.setSuggestedCenterValue(
                getSuggestedCenterValue());

        final PROMedSRobustEstimator<PinholeCamera> innerEstimator =
                new PROMedSRobustEstimator<>(
                        new PROMedSRobustEstimatorListener<PinholeCamera>() {

                            // point to be reused when computing residuals
                            private final Point2D mTestPoint = Point2D.create(
                                    CoordinatesType.HOMOGENEOUS_COORDINATES);

                            // 3D points for a subset of samples
                            private final List<Point3D> mSubset3D = new ArrayList<>();

                            // 2D points for a subset of samples
                            private final List<Point2D> mSubset2D = new ArrayList<>();

                            @Override
                            public double getThreshold() {
                                return mStopThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mPoints3D.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return PointCorrespondencePinholeCameraRobustEstimator.
                                        MIN_NUMBER_OF_POINT_CORRESPONDENCES;
                            }

                            @Override
                            public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                                    final List<PinholeCamera> solutions) {
                                mSubset3D.clear();
                                mSubset3D.add(mPoints3D.get(samplesIndices[0]));
                                mSubset3D.add(mPoints3D.get(samplesIndices[1]));
                                mSubset3D.add(mPoints3D.get(samplesIndices[2]));
                                mSubset3D.add(mPoints3D.get(samplesIndices[3]));
                                mSubset3D.add(mPoints3D.get(samplesIndices[4]));
                                mSubset3D.add(mPoints3D.get(samplesIndices[5]));

                                mSubset2D.clear();
                                mSubset2D.add(mPoints2D.get(samplesIndices[0]));
                                mSubset2D.add(mPoints2D.get(samplesIndices[1]));
                                mSubset2D.add(mPoints2D.get(samplesIndices[2]));
                                mSubset2D.add(mPoints2D.get(samplesIndices[3]));
                                mSubset2D.add(mPoints2D.get(samplesIndices[4]));
                                mSubset2D.add(mPoints2D.get(samplesIndices[5]));

                                try {
                                    nonRobustEstimator.setLists(mSubset3D, mSubset2D);

                                    final PinholeCamera cam = nonRobustEstimator.estimate();
                                    solutions.add(cam);
                                } catch (final Exception e) {
                                    // if points configuration is degenerate, no solution is
                                    // added
                                }
                            }

                            @Override
                            public double computeResidual(final PinholeCamera currentEstimation,
                                                          final int i) {
                                // pick i-th points
                                final Point3D point3D = mPoints3D.get(i);
                                final Point2D point2D = mPoints2D.get(i);

                                // project point3D into test point
                                currentEstimation.project(point3D, mTestPoint);

                                // compare test point and 2D point
                                return mTestPoint.distanceTo(point2D);
                            }

                            @Override
                            public boolean isReady() {
                                return PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.
                                        this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<PinholeCamera> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateStart(
                                            PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<PinholeCamera> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(
                                            PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<PinholeCamera> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<PinholeCamera> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator.this,
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
        return RobustEstimatorMethod.PROMedS;
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
        final PROMedSRobustEstimator.PROMedSInliersData inliersData =
                (PROMedSRobustEstimator.PROMedSInliersData) getInliersData();
        return inliersData.getEstimatedThreshold();
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
        if (qualityScores.length < MIN_NUMBER_OF_POINT_CORRESPONDENCES) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
