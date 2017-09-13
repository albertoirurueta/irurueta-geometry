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

package com.irurueta.geometry.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.*;
import com.irurueta.geometry.calib3d.DualImageOfAbsoluteConic;
import com.irurueta.geometry.calib3d.ImageOfAbsoluteConic;
import com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicEstimator;
import com.irurueta.geometry.calib3d.estimators.KruppaDualImageOfAbsoluteConicEstimator;
import com.irurueta.geometry.calib3d.estimators.LMSEImageOfAbsoluteConicEstimator;
import com.irurueta.geometry.epipolar.Corrector;
import com.irurueta.geometry.epipolar.EpipolarException;
import com.irurueta.geometry.epipolar.EssentialMatrix;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.epipolar.estimators.*;
import com.irurueta.geometry.estimators.*;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences for multiple views.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 */
public abstract class BaseSparseReconstructor<C extends BaseSparseReconstructorConfiguration,
        R extends BaseSparseReconstructor> {

    /**
     * Minimum required number of views.
     */
    public static final int MIN_NUMBER_OF_VIEWS = 2;

    /**
     * Default scale.
     */
    protected static final double DEFAULT_SCALE = 1.0;

    /**
     * Current estimated fundamental matrix.
     */
    protected EstimatedFundamentalMatrix mCurrentEstimatedFundamentalMatrix;

    /**
     * Current estimated camera in a metric stratum (i.e. up to scale).
     */
    protected EstimatedCamera mCurrentMetricEstimatedCamera;

    /**
     * Previous estimated camera in a metric stratum (i.e. up to scale).
     */
    protected EstimatedCamera mPreviousMetricEstimatedCamera;

    /**
     * Reconstructed 3D points which still remain active to match next view in a metric stratum (i.e. up to scale).
     */
    protected List<ReconstructedPoint3D> mActiveMetricReconstructedPoints;

    /**
     * Current estimated scale. This will typically converge to 1.0 as more views are processed.
     * The closer this value is to one, the more likely the scale of estimated cameras is accurate.
     */
    protected double mCurrentScale = DEFAULT_SCALE;

    /**
     * Current estimated camera in euclidean stratum (i.e. with actual scale).
     */
    protected EstimatedCamera mCurrentEuclideanEstimatedCamera;

    /**
     * Previous estimated camera in euclidean stratum (i.e. with actual scale).
     */
    protected EstimatedCamera mPreviousEuclideanEstimatedCamera;

    /**
     * Reconstructed 3D points which still remain active to match next view in euclidean stratum (i.e. with actual
     * scale).
     */
    protected List<ReconstructedPoint3D> mActiveEuclideanReconstructedPoints;

    /**
     * Configuration for this reconstructor.
     */
    protected C mConfiguration;

    /**
     * Listener in charge of handling events such as when reconstruction starts, ends,
     * when certain data is needed or when estimation of data has been computed.
     */
    protected BaseSparseReconstructorListener<R> mListener;

    /**
     * Indicates whether reconstruction has failed or not.
     */
    protected volatile boolean mFailed;

    /**
     * Indicates whether reconstruction is running or not.
     */
    protected volatile boolean mRunning;

    /**
     * Indicates whether reconstruction has been cancelled or not.
     */
    private volatile boolean mCancelled;

    /**
     * Counter of number of processed views.
     */
    private int mViewCount;

    /**
     * Indicates whether reconstruction has finished or not.
     */
    private boolean mFinished = false;

    /**
     * Samples on previous view.
     */
    private List<Sample2D> mPreviousViewSamples = null;

    /**
     * Samples on last processed view (i.e. current view).
     */
    private List<Sample2D> mCurrentViewSamples;

    /**
     * Active matches between current and previous views.
     */
    private List<MatchedSamples> mMatches = new ArrayList<>();

    /**
     * Id of previous view.
     */
    private int mPreviousViewId = 0;

    /**
     * Id of current view.
     */
    private int mCurrentViewId;

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not provided.
     */
    public BaseSparseReconstructor(C configuration,
            BaseSparseReconstructorListener<R> listener)
            throws NullPointerException {
        if (configuration == null || listener == null) {
            throw new NullPointerException();
        }
        mConfiguration = configuration;
        mListener = listener;
    }

    /**
     * Gets configuration for this reconstructor.
     * @return configuration for this reconstructor.
     */
    public C getConfiguration() {
        return mConfiguration;
    }

    /**
     * Gets listener in charge of handling events such as when reconstruction starts,
     * ends, when certain data is needed or when estimation of data has been computed.
     * @return listener in charge of handling events.
     */
    public BaseSparseReconstructorListener<R> getListener() {
        return mListener;
    }

    /**
     * Indicates whether reconstruction is running or not.
     * @return true if reconstruction is running, false if reconstruction has stopped
     * for any reason.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether reconstruction has been cancelled or not.
     * @return true if reconstruction has been cancelled, false otherwise.
     */
    public boolean isCancelled() {
        return mCancelled;
    }

    /**
     * Indicates whether reconstruction has failed or not.
     * @return true if reconstruction has failed, false otherwise.
     */
    public boolean hasFailed() {
        return mFailed;
    }

    /**
     * Indicates whether the reconstruction has finished.
     * @return true if reconstruction has finished, false otherwise.
     */
    public boolean isFinished() {
        return mFinished;
    }

    /**
     * Gets counter of number of processed views.
     * @return counter of number of processed views.
     */
    public int getViewCount() {
        return mViewCount;
    }

    /**
     * Gets estimated fundamental matrix for current view.
     * This fundamental matrix relates current view with the previously processed one.
     * @return current estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getCurrentEstimatedFundamentalMatrix() {
        return mCurrentEstimatedFundamentalMatrix;
    }

    /**
     * Gets estimated camera for current view.
     * @return current estimated camera.
     */
    public EstimatedCamera getCurrentEstimatedCamera() {
        return mCurrentMetricEstimatedCamera;
    }

    /**
     * Gets estimated camera for previous view.
     * @return previous estimated cameras.
     */
    public EstimatedCamera getPreviousEstimatedCamera() {
        return mPreviousMetricEstimatedCamera;
    }

    /**
     * Gets reconstructed 3D points which still remain active to match next view.
     * @return active reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getActiveReconstructedPoints() {
        return mActiveMetricReconstructedPoints;
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the
     * reconstruction step by step, one view at a time.
     * This method is useful when data is gathered on real time from a camera and the
     * number of views is unknown.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    public boolean processOneView() {
        if (mViewCount == 0) {
            if (mRunning) {
                //already started
                return true;
            }

            reset();
            mRunning = true;

            mListener.onStart((R)this);
        }

        if(!mListener.hasMoreViewsAvailable((R)this)) {
            mListener.onFinish((R)this);
            mRunning = false;
            mFinished = true;
            return false;
        }

        mCurrentEstimatedFundamentalMatrix = null;
        mCurrentViewSamples = new ArrayList<Sample2D>();
        mListener.onRequestSamplesForCurrentView((R)this, mViewCount,
                mCurrentViewSamples);

        if (mPreviousViewSamples == null) {
            //for first view we simply keep samples (if enough are provided)
            processFirstView();
        } else {

            if (mCurrentEstimatedFundamentalMatrix == null) {
                //for second view, check that we have enough samples
                processSecondView();
            } else {
                processAdditionalView();
            }
        }

        mViewCount++;

        if (mCancelled) {
            mListener.onCancel((R)this);
        }

        return !mFinished;
    }

    /**
     * Starts reconstruction of all available data to reconstruct the whole scene.
     * If reconstruction has already started and is running, calling this method has
     * no effect.
     * This method is useful when all data is available before starting the reconstruction.
     */
    public void start() {
        while(processOneView()) {
            if (mCancelled) {
                break;
            }
        }
    }

    /**
     * Cancels reconstruction.
     * If reconstruction has already been cancelled, calling this method has no effect.
     */
    public void cancel() {
        if (mCancelled) {
            //already cancelled
            return;
        }

        mCancelled = true;
    }

    /**
     * Resets this instance so that a reconstruction can be started from the beginning without cancelling current one.
     */
    public void reset() {
        mPreviousViewSamples = mCurrentViewSamples = null;

        mCancelled = mFailed = false;
        mViewCount = 0;
        mRunning = false;

        mCurrentEstimatedFundamentalMatrix = null;
        mCurrentMetricEstimatedCamera = mPreviousMetricEstimatedCamera = null;
        mActiveMetricReconstructedPoints = null;

        mFinished = false;
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on
     * those implementations where scale can be measured or is already known.
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if post processing succeeded, false otherwise.
     */
    protected abstract boolean postProcessOne(boolean isInitialPairOfViews);

    /**
     * Processes data for first view.
     */
    private void processFirstView() {
        if(hasEnoughSamplesForFundamentalMatrixEstimation(mCurrentViewSamples)) {
            mListener.onSamplesAccepted((R)this, mViewCount,
                    mCurrentViewSamples);
            mPreviousViewSamples = mCurrentViewSamples;
            mPreviousViewId = mViewCount;
        }
    }

    /**
     * Processes data for second view.
     */
    private void processSecondView() {
        if (hasEnoughSamplesForFundamentalMatrixEstimation(mCurrentViewSamples)) {

            //find matches
            mMatches.clear();
            mListener.onRequestMatches((R) this, mPreviousViewSamples,
                    mCurrentViewSamples, mPreviousViewId, mViewCount,
                    mMatches);

            if (hasEnoughMatchesForFundamentalMatrixEstimation(mMatches)) {
                //if enough matches are retrieved, attempt to compute
                //fundamental matrix
                if ((mConfiguration.isGeneralSceneAllowed() &&
                        estimateFundamentalMatrix(mMatches, mPreviousViewId,
                                mViewCount, true)) ||
                        (mConfiguration.isPlanarSceneAllowed() &&
                                estimatePlanarFundamentalMatrix(mMatches,
                                        mPreviousViewId, mViewCount, true))) {
                    //fundamental matrix could be estimated
                    mListener.onSamplesAccepted((R) this, mViewCount,
                            mCurrentViewSamples);
                    mCurrentViewId = mViewCount;

                    mListener.onFundamentalMatrixEstimated((R) this,
                            mCurrentEstimatedFundamentalMatrix);

                    if (estimateInitialCamerasAndPoints()) {
                        //cameras and points have been estimated
                        mListener.onMetricCameraEstimated((R) this,
                                mPreviousViewId, mCurrentViewId,
                                mPreviousMetricEstimatedCamera, mCurrentMetricEstimatedCamera);
                        mListener.onMetricReconstructedPointsEstimated(
                                (R) this, mMatches, mActiveMetricReconstructedPoints);

                        if (!postProcessOne(false)) {
                            //something failed
                            mFailed = true;
                            mListener.onFail((R)this);
                        } else {
                            //post processing succeeded
                            mListener.onEuclideanCameraEstimated((R)this, mPreviousViewId, mCurrentViewId,
                                    mCurrentScale, mPreviousEuclideanEstimatedCamera, mCurrentEuclideanEstimatedCamera);
                            mListener.onEuclideanReconstructedPointsEstimated((R)this, mCurrentScale,
                                    mActiveEuclideanReconstructedPoints);
                        }
                    } else {
                        //initial cameras failed
                        mFailed = true;
                        mListener.onFail((R) this);
                    }
                } else {
                    //estimation of fundamental matrix failed
                    mListener.onSamplesRejected((R) this, mViewCount,
                            mCurrentViewSamples);
                }
            }
        }
    }

    /**
     * Processes data for one additional view.
     */
    private void processAdditionalView() {
        //find matches
        mMatches.clear();
        mListener.onRequestMatches((R) this, mPreviousViewSamples,
                mCurrentViewSamples, mPreviousViewId, mViewCount,
                mMatches);

        List<Point3D> points3D = new ArrayList<>();
        List<Point2D> points2D = new ArrayList<>();
        double[] qualityScores = setUpCameraEstimatorMatches(points3D, points2D);

        if (hasEnoughSamplesForCameraEstimation(points3D, points2D)) {
            //enough matches available.
            PinholeCamera currentCamera = null;
            Matrix currentCameraCovariance = null;
            if (mConfiguration.getUseEPnPForAdditionalCamerasEstimation()) {
                //use EPnP for additional cameras estimation.
                //EPnP requires knowledge of camera intrinsics

                PinholeCameraIntrinsicParameters intrinsicParameters = null;
                if (mConfiguration.getUseDAQForAdditionalCamerasIntrinsics() ||
                        mConfiguration.getUseDIACForAdditionalCamerasIntrinsics()) {

                    //compute fundamental matrix to estimate intrinsics
                    if((mConfiguration.isGeneralSceneAllowed() &&
                            estimateFundamentalMatrix(mMatches, mPreviousViewId, mViewCount, false)) ||
                            (mConfiguration.isPlanarSceneAllowed() &&
                            estimatePlanarFundamentalMatrix(mMatches, mPreviousViewId, mViewCount, false))) {
                        //fundamental matrix could be estimated
                        mListener.onSamplesAccepted((R)this, mViewCount, mCurrentViewSamples);
                        mCurrentViewId = mViewCount;

                        mListener.onFundamentalMatrixEstimated((R)this, mCurrentEstimatedFundamentalMatrix);

                    } else {
                        //fundamental matrix estimation failed
                        mFailed = true;
                        mListener.onFail((R)this);
                    }

                    //use fundamental matrix to estimate intrinsics using DIAC or DAQ
                    if (mConfiguration.getUseDIACForAdditionalCamerasIntrinsics()) {
                        intrinsicParameters = estimateIntrinsicsDIAC();
                    } else if (mConfiguration.getUseDAQForAdditionalCamerasIntrinsics()) {
                        intrinsicParameters = estimateIntrinsicsDAQ();
                    }

                } else if (mConfiguration.getAdditionalCamerasIntrinsics() != null) {
                    //use configuration provided intrinsics
                    intrinsicParameters = mConfiguration.getAdditionalCamerasIntrinsics();
                }

                if (intrinsicParameters == null) {
                    //something failed or bad configuration
                    mFailed = true;
                    mListener.onFail((R)this);
                }

                try {
                    //use EPnP for additional cameras estimation
                    EPnPPointCorrespondencePinholeCameraRobustEstimator cameraEstimator =
                            EPnPPointCorrespondencePinholeCameraRobustEstimator.create(intrinsicParameters, points3D,
                                    points2D, qualityScores,
                                    mConfiguration.getAdditionalCamerasRobustEstimationMethod());
                    cameraEstimator.setPlanarConfigurationAllowed(
                            mConfiguration.getAdditionalCamerasAllowPlanarConfiguration());
                    cameraEstimator.setNullspaceDimension2Allowed(
                            mConfiguration.getAdditionalCamerasAllowNullspaceDimension2());
                    cameraEstimator.setNullspaceDimension3Allowed(
                            mConfiguration.getAdditionalCamerasAllowNullspaceDimension3());
                    cameraEstimator.setPlanarThreshold(mConfiguration.getAdditionalCamerasPlanarThreshold());
                    cameraEstimator.setResultRefined(mConfiguration.areAdditionalCamerasRefined());
                    cameraEstimator.setCovarianceKept(mConfiguration.isAdditionalCamerasCovarianceKept());
                    cameraEstimator.setFastRefinementUsed(mConfiguration.getAdditionalCamerasUseFastRefinement());
                    cameraEstimator.setConfidence(mConfiguration.getAdditionalCamerasConfidence());
                    cameraEstimator.setMaxIterations(mConfiguration.getAdditionalCamerasMaxIterations());

                    cameraEstimator.setSuggestSkewnessValueEnabled(
                            mConfiguration.isAdditionalCamerasSuggestSkewnessValueEnabled());
                    cameraEstimator.setSuggestedSkewnessValue(
                            mConfiguration.getAdditionalCamerasSuggestedSkewnessValue());

                    cameraEstimator.setSuggestHorizontalFocalLengthEnabled(
                            mConfiguration.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
                    cameraEstimator.setSuggestedHorizontalFocalLengthValue(
                            mConfiguration.getAdditionalCamerasSuggestedHorizontalFocalLengthValue());

                    cameraEstimator.setSuggestVerticalFocalLengthEnabled(
                            mConfiguration.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
                    cameraEstimator.setSuggestedVerticalFocalLengthValue(
                            mConfiguration.getAdditionalCamerasSuggestedVerticalFocalLengthValue());

                    cameraEstimator.setSuggestAspectRatioEnabled(
                            mConfiguration.isAdditionalCamerasSuggestAspectRatioEnabled());
                    cameraEstimator.setSuggestedAspectRatioValue(
                            mConfiguration.getAdditionalCamerasSuggestedAspectRatioValue());

                    cameraEstimator.setSuggestPrincipalPointEnabled(
                            mConfiguration.isAdditionalCamerasSuggestPrincipalPointEnabled());
                    cameraEstimator.setSuggestedPrincipalPointValue(
                            mConfiguration.getAdditionalCamerasSuggestedPrincipalPointValue());

                    currentCamera = cameraEstimator.estimate();
                    currentCameraCovariance = cameraEstimator.getCovariance();

                } catch (Exception e) {
                    //something failed
                    mFailed = true;
                    mListener.onFail((R)this);
                }

            } else if (mConfiguration.getUseUPnPForAdditionalCamerasEstimation()) {
                mListener.onSamplesAccepted((R)this, mViewCount, mCurrentViewSamples);
                mCurrentViewId = mViewCount;

                try {
                    //use UPnP for additional cameras estimation
                    UPnPPointCorrespondencePinholeCameraRobustEstimator cameraEstimator =
                            UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D,
                                    qualityScores, mConfiguration.getAdditionalCamerasRobustEstimationMethod());
                    cameraEstimator.setPlanarConfigurationAllowed(
                            mConfiguration.getAdditionalCamerasAllowPlanarConfiguration());
                    cameraEstimator.setNullspaceDimension2Allowed(
                            mConfiguration.getAdditionalCamerasAllowNullspaceDimension2());
                    cameraEstimator.setPlanarThreshold(mConfiguration.getAdditionalCamerasPlanarThreshold());
                    cameraEstimator.setResultRefined(mConfiguration.areAdditionalCamerasRefined());
                    cameraEstimator.setCovarianceKept(mConfiguration.isAdditionalCamerasCovarianceKept());
                    cameraEstimator.setFastRefinementUsed(mConfiguration.getAdditionalCamerasUseFastRefinement());
                    cameraEstimator.setConfidence(mConfiguration.getAdditionalCamerasConfidence());
                    cameraEstimator.setMaxIterations(mConfiguration.getAdditionalCamerasMaxIterations());

                    cameraEstimator.setSkewness(mConfiguration.getAdditionalCamerasSkewness());
                    cameraEstimator.setHorizontalPrincipalPoint(
                            mConfiguration.getAdditionalCamerasHorizontalPrincipalPoint());
                    cameraEstimator.setVerticalPrincipalPoint(
                            mConfiguration.getAdditionalCamerasVerticalPrincipalPoint());

                    cameraEstimator.setSuggestSkewnessValueEnabled(
                            mConfiguration.isAdditionalCamerasSuggestSkewnessValueEnabled());
                    cameraEstimator.setSuggestedSkewnessValue(
                            mConfiguration.getAdditionalCamerasSuggestedSkewnessValue());

                    cameraEstimator.setSuggestHorizontalFocalLengthEnabled(
                            mConfiguration.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
                    cameraEstimator.setSuggestedHorizontalFocalLengthValue(
                            mConfiguration.getAdditionalCamerasSuggestedHorizontalFocalLengthValue());

                    cameraEstimator.setSuggestVerticalFocalLengthEnabled(
                            mConfiguration.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
                    cameraEstimator.setSuggestedVerticalFocalLengthValue(
                            mConfiguration.getAdditionalCamerasSuggestedVerticalFocalLengthValue());

                    cameraEstimator.setSuggestAspectRatioEnabled(
                            mConfiguration.isAdditionalCamerasSuggestAspectRatioEnabled());
                    cameraEstimator.setSuggestedAspectRatioValue(
                            mConfiguration.getAdditionalCamerasSuggestedAspectRatioValue());

                    cameraEstimator.setSuggestPrincipalPointEnabled(
                            mConfiguration.isAdditionalCamerasSuggestPrincipalPointEnabled());
                    cameraEstimator.setSuggestedPrincipalPointValue(
                            mConfiguration.getAdditionalCamerasSuggestedPrincipalPointValue());

                    currentCamera = cameraEstimator.estimate();
                    currentCameraCovariance = cameraEstimator.getCovariance();

                } catch (Exception e) {
                    //something failed
                    mFailed = true;
                    mListener.onFail((R)this);
                }

            } else {
                mListener.onSamplesAccepted((R)this, mViewCount, mCurrentViewSamples);
                mCurrentViewId = mViewCount;

                try {
                    //use DLT for additional cameras estimation
                    DLTPointCorrespondencePinholeCameraRobustEstimator cameraEstimator =
                            DLTPointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores,
                                    mConfiguration.getAdditionalCamerasRobustEstimationMethod());
                    cameraEstimator.setResultRefined(mConfiguration.areAdditionalCamerasRefined());
                    cameraEstimator.setCovarianceKept(mConfiguration.isAdditionalCamerasCovarianceKept());
                    cameraEstimator.setFastRefinementUsed(mConfiguration.getAdditionalCamerasUseFastRefinement());
                    cameraEstimator.setConfidence(mConfiguration.getAdditionalCamerasConfidence());
                    cameraEstimator.setMaxIterations(mConfiguration.getAdditionalCamerasMaxIterations());

                    cameraEstimator.setSuggestSkewnessValueEnabled(
                            mConfiguration.isAdditionalCamerasSuggestSkewnessValueEnabled());
                    cameraEstimator.setSuggestedSkewnessValue(
                            mConfiguration.getAdditionalCamerasSuggestedSkewnessValue());

                    cameraEstimator.setSuggestHorizontalFocalLengthEnabled(
                            mConfiguration.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
                    cameraEstimator.setSuggestedHorizontalFocalLengthValue(
                            mConfiguration.getAdditionalCamerasSuggestedHorizontalFocalLengthValue());

                    cameraEstimator.setSuggestVerticalFocalLengthEnabled(
                            mConfiguration.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
                    cameraEstimator.setSuggestedVerticalFocalLengthValue(
                            mConfiguration.getAdditionalCamerasSuggestedVerticalFocalLengthValue());

                    cameraEstimator.setSuggestAspectRatioEnabled(
                            mConfiguration.isAdditionalCamerasSuggestAspectRatioEnabled());
                    cameraEstimator.setSuggestedAspectRatioValue(
                            mConfiguration.getAdditionalCamerasSuggestedAspectRatioValue());

                    cameraEstimator.setSuggestPrincipalPointEnabled(
                            mConfiguration.isAdditionalCamerasSuggestPrincipalPointEnabled());
                    cameraEstimator.setSuggestedPrincipalPointValue(
                            mConfiguration.getAdditionalCamerasSuggestedPrincipalPointValue());

                    currentCamera = cameraEstimator.estimate();
                    currentCameraCovariance = cameraEstimator.getCovariance();

                } catch (Exception e) {
                    //something failed
                    mFailed = true;
                    mListener.onFail((R)this);
                }
            }

            mPreviousMetricEstimatedCamera = mCurrentMetricEstimatedCamera;

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(currentCamera);
            mCurrentMetricEstimatedCamera.setId(String.valueOf(mCurrentViewId));
            mCurrentMetricEstimatedCamera.setCovariance(currentCameraCovariance);


            //notify camera estimation
            mListener.onMetricCameraEstimated((R) this,
                    mPreviousViewId, mCurrentViewId,
                    mPreviousMetricEstimatedCamera, mCurrentMetricEstimatedCamera);


            //reconstruct all matches and refine existing reconstructed points
            reconstructAndRefineMatches();

            //notify reconstruction update
            mListener.onMetricReconstructedPointsEstimated((R)this, mMatches, mActiveMetricReconstructedPoints);

            if (!postProcessOne(false)) {
                //something failed
                mFailed = true;
                mListener.onFail((R)this);
            } else {
                //post processing succeeded
                mListener.onEuclideanCameraEstimated((R)this, mPreviousViewId, mCurrentViewId,
                        mCurrentScale, mPreviousEuclideanEstimatedCamera, mCurrentEuclideanEstimatedCamera);
                mListener.onEuclideanReconstructedPointsEstimated((R)this, mCurrentScale,
                        mActiveEuclideanReconstructedPoints);
            }
        }
    }

    /**
     * Reconstructs new 3D points or refines existing ones taking into account existing matches and estimated cameras
     */
    private void reconstructAndRefineMatches() {
        if (mMatches == null || mMatches.isEmpty()) {
            return;
        }

        try {
            RobustSinglePoint3DTriangulator robustTriangulator = null;
            SinglePoint3DTriangulator triangulator = null;
            boolean qualityScoresRequired = false;
            if (mConfiguration.getAdditionalCamerasRobustEstimationMethod() != null) {
                robustTriangulator = RobustSinglePoint3DTriangulator.create(
                        mConfiguration.getAdditionalCamerasRobustEstimationMethod());
                robustTriangulator.setConfidence(mConfiguration.getPointTriangulatorConfidence());
                robustTriangulator.setMaxIterations(mConfiguration.getPointTriangulatorMaxIterations());

                double threshold = mConfiguration.getPointTriangulatorThreshold();
                switch (mConfiguration.getAdditionalCamerasRobustEstimationMethod()) {
                    case LMedS:
                        ((LMedSRobustSinglePoint3DTriangulator) robustTriangulator).setStopThreshold(threshold);
                        break;
                    case MSAC:
                        ((MSACRobustSinglePoint3DTriangulator) robustTriangulator).setThreshold(threshold);
                        break;
                    case PROMedS:
                        ((PROMedSRobustSinglePoint3DTriangulator) robustTriangulator).setStopThreshold(threshold);
                        qualityScoresRequired = true;
                        break;
                    case PROSAC:
                        ((PROSACRobustSinglePoint3DTriangulator) robustTriangulator).setThreshold(threshold);
                        qualityScoresRequired = true;
                        break;
                    case RANSAC:
                        ((RANSACRobustSinglePoint3DTriangulator) robustTriangulator).setThreshold(threshold);
                        break;
                }

            } else {
                if (mConfiguration.isHomogeneousPointTriangulatorUsed()) {
                    triangulator = SinglePoint3DTriangulator.create(
                            Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);
                } else {
                    triangulator = SinglePoint3DTriangulator.create(
                            Point3DTriangulatorType.
                                    LMSE_INHOMOGENEOUS_TRIANGULATOR);
                }
            }

            mActiveMetricReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D reconstructedPoint;
            int matchPos = 0;
            for (MatchedSamples match : mMatches) {
                Sample2D[] samples = match.getSamples();
                EstimatedCamera[] estimatedCameras = match.getCameras();

                //estimated cameras does not yet contain last estimated camera
                if (samples.length != estimatedCameras.length + 1) {
                    continue;
                }

                List<Point2D> points = new ArrayList<>();
                List<PinholeCamera> cameras = new ArrayList<>();
                BitSet validSamples = new BitSet(samples.length);
                int numValid = 0;
                for (int i = 0; i < samples.length; i++) {
                    Point2D point2D = samples[i].getPoint();
                    PinholeCamera camera = estimatedCameras[i].getCamera();

                    if (point2D == null || camera == null) {
                        validSamples.clear(i);
                        continue;
                    } else {
                        validSamples.set(i);
                        numValid++;
                    }

                    points.add(point2D);
                    cameras.add(camera);
                }

                //also add current camera which is not yet available on estimated cameras array
                cameras.add(mCurrentMetricEstimatedCamera.getCamera());

                if (points.size() < SinglePoint3DTriangulator.MIN_REQUIRED_VIEWS ||
                        points.size() != cameras.size()) {
                    //point cannot be triangulated
                    continue;
                }

                Point3D point3D = null;
                if (robustTriangulator != null) {
                    robustTriangulator.setPointsAndCameras(points, cameras);
                    if(qualityScoresRequired) {
                        //copy quality scores
                        double[] qualityScores = new double[numValid];
                        for (int i = 0, j = 0; i < samples.length; i++) {
                            if(validSamples.get(i)) {
                                qualityScores[j] = samples[i].getQualityScore();
                                j++;
                            }
                        }
                        robustTriangulator.setQualityScores(qualityScores);
                    }

                    point3D = robustTriangulator.triangulate();

                } else if (triangulator != null) {
                    triangulator.setPointsAndCameras(points, cameras);
                    point3D = triangulator.triangulate();

                } else {
                    continue;
                }

                //save triangulated point
                reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(point3D);
                reconstructedPoint.setInlier(true);
                reconstructedPoint.setId(String.valueOf(matchPos));
                mActiveMetricReconstructedPoints.add(reconstructedPoint);

                matchPos++;
            }
        } catch (Exception e) {
            //something failed
            mFailed = true;
            mListener.onFail((R)this);
        }
    }

    /**
     * Set ups current matched 3D/2D points to estimate a pinhole camera.
     * @param points3D 3D matched points.
     * @param points2D 2D matched points.
     * @return quality scores for matched points.
     */
    private double[] setUpCameraEstimatorMatches(List<Point3D> points3D, List<Point2D> points2D) {
        if (mMatches == null || mMatches.isEmpty()) {
            return null;
        }

        points3D.clear();
        points2D.clear();

        boolean qualityScoresRequired =
                mConfiguration.getAdditionalCamerasRobustEstimationMethod() == RobustEstimatorMethod.PROSAC ||
                mConfiguration.getAdditionalCamerasRobustEstimationMethod() == RobustEstimatorMethod.PROMedS;


        int[] positions = null;
        if (qualityScoresRequired) {
            positions = new int[mMatches.size()];
        }

        int numMatches = 0;
        int i = 0;
        for (MatchedSamples match : mMatches) {
            Sample2D[] samples = match.getSamples();
            int[] viewIds = match.getViewIds();
            int pos = getPositionForViewId(viewIds, mViewCount);
            if(pos < 0) {
                continue;
            }
            if (positions != null) {
                positions[i] = pos;
            }

            Sample2D sample = samples[pos];
            ReconstructedPoint3D reconstructedPoint3D = match.getReconstructedPoint();

            if (sample == null || sample.getPoint() == null || reconstructedPoint3D == null ||
                    reconstructedPoint3D.getPoint() == null) {
                positions[i] = -1;
            } else {
                points2D.add(sample.getPoint());
                points3D.add(reconstructedPoint3D.getPoint());
                numMatches++;
            }

            i++;
        }

        //pick quality scores
        double[] qualityScores = null;
        if (qualityScoresRequired) {
            qualityScores = new double[numMatches];
            int j = 0;
            for (i = 0; i < positions.length; i++) {
                if (positions[i] < 0) {
                    continue;
                }

                qualityScores[j] = mMatches.get(i).getQualityScore();
                j++;
            }
        }

        return qualityScores;
    }

    /**
     * Estimates additional camera intrinsics using DIAC (Dual Image of Absolute Conic) method.
     * @return additional camera intrinsics or null if something fails.
     */
    private PinholeCameraIntrinsicParameters estimateIntrinsicsDIAC() {
        FundamentalMatrix fundamentalMatrix =
                mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

        //use inlier points used for fundamental matrix estimation
        List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

        List<Point2D> points1 = new ArrayList<Point2D>();
        List<Point2D> points2 = new ArrayList<Point2D>();
        int length = samples1.size();
        for (int i = 0; i < length; i++) {
            Sample2D sample1 = samples1.get(i);
            Sample2D sample2 = samples2.get(i);

            Point2D point1 = sample1.getPoint();
            Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            KruppaDualImageOfAbsoluteConicEstimator diacEstimator =
                    new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(mConfiguration.getAdditionalCamerasHorizontalPrincipalPoint());
            diacEstimator.setPrincipalPointY(mConfiguration.getAdditionalCamerasVerticalPrincipalPoint());
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(mConfiguration.getAdditionalCamerasAspectRatio());

            DualImageOfAbsoluteConic diac = diacEstimator.estimate();
            return diac.getIntrinsicParameters();

        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Estimates additional cameras intrinsics using DAQ (Dual Absolute Quadric) method.
     * @return additional camera intrinsics or null if something fails.
     */
    private PinholeCameraIntrinsicParameters estimateIntrinsicsDAQ() {
        try {
            FundamentalMatrix fundamentalMatrix =
                    mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
            estimator.setAspectRatio(mConfiguration.getInitialCamerasAspectRatio());
            estimator.estimate();

            PinholeCamera camera = estimator.getEstimatedLeftCamera();
            camera.decompose();
            return camera.getIntrinsicParameters();

        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Indicates whether there are enough matched points to estimate an additional camera.
     * @param points3D 3D matched points to check.
     * @param points2D 2D matched points to check.
     * @return true if there are enough matched points, false otherwise.
     */
    private boolean hasEnoughSamplesForCameraEstimation(List<Point3D> points3D, List<Point2D> points2D) {
        return points3D != null && points2D != null && points3D.size() == points2D.size() &&
                hasEnoughSamplesOrMatchesForCameraEstimation(points3D.size());
    }

    /**
     * Indicates whether there are enough matches or samples to estimate an additional
     * camera.
     * @param count number of matches or samples.
     * @return true if there are enough matches or samples, false otherwise.
     */
    private boolean hasEnoughSamplesOrMatchesForCameraEstimation(int count) {
        if (mConfiguration.getUseDAQForAdditionalCamerasIntrinsics() ||
                mConfiguration.getUseDIACForAdditionalCamerasIntrinsics()) {
            //when DAQ or DIAC is required for additional cameras, fundamental matrix
            //also needs to be computed, which requires 7 or 8 matches.
            return hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(count);
        } else {
            //EPnP, UPnP or DLT is used for additional cameras estimation without fundamental
            //matrix. Only 6 matches are required
            return count >= PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;
        }
    }

    /**
     * Indicates whether there are enough samples to estimate a fundamental matrix.
     * @param samples samples to check.
     * @return true if there are enough samples, false otherwise.
     */
    private boolean hasEnoughSamplesForFundamentalMatrixEstimation(List<Sample2D> samples) {
        return hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(
                samples != null ? samples.size() : 0);
    }

    /**
     * Indicates whether there are enough matches to estimate a fundamental matrix.
     * @param matches matches to check.
     * @return true if there are enough matches, false otherwise.
     */
    private boolean hasEnoughMatchesForFundamentalMatrixEstimation(
            List<MatchedSamples> matches) {
        return hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(
                matches != null ? matches.size() : 0);
    }

    /**
     * Indicates whether there are enough matches or samples to estimate a fundamental
     * matrix.
     * @param count number of matches or samples.
     * @return true if there are enough matches or samples, false otherwise.
     */
    private boolean hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(int count) {
        if (mConfiguration.isGeneralSceneAllowed()) {
            switch (mConfiguration.getNonRobustFundamentalMatrixEstimatorMethod()) {
                case EIGHT_POINTS_ALGORITHM:
                    return count >= EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS;
                case SEVEN_POINTS_ALGORITHM:
                    return count >= SevenPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS;
            }
        } else if (mConfiguration.isPlanarSceneAllowed()) {
            return count >= ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE;
        }
        return false;
    }

    /**
     * Estimates fundamental matrix for provided matches, when 3D points lay in a general non
     * degenerate 3D configuration.
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view being related by estimated fundamental matrix.
     * @param viewId2 id of second view being related by estimated fundamental matrix.
     * @param isInitialPairOfViews true if fundamental matrix needs to be estimated for the initial
     *                             pair of views, false otherwise.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimateFundamentalMatrix(List<MatchedSamples> matches, int viewId1, int viewId2,
                                              boolean isInitialPairOfViews) {
        if (matches == null) {
            return false;
        }

        int count = matches.size();
        List<Sample2D> leftSamples = new ArrayList<>(count);
        List<Sample2D> rightSamples = new ArrayList<>(count);
        List<Point2D> leftPoints = new ArrayList<>(count);
        List<Point2D> rightPoints = new ArrayList<>(count);
        double[] qualityScores = new double[count];
        double principalPointX, principalPointY;
        if (isInitialPairOfViews) {
            if (mConfiguration.getInitialCamerasEstimatorMethod() ==
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC ||
                    mConfiguration.getInitialCamerasEstimatorMethod() ==
                            InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
                principalPointX = mConfiguration.getPrincipalPointX();
                principalPointY = mConfiguration.getPrincipalPointY();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        } else {
            if (mConfiguration.getUseDIACForAdditionalCamerasIntrinsics() ||
                    mConfiguration.getUseDAQForAdditionalCamerasIntrinsics()) {
                principalPointX = mConfiguration.getAdditionalCamerasHorizontalPrincipalPoint();
                principalPointY = mConfiguration.getAdditionalCamerasVerticalPrincipalPoint();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        }

        int i = 0;
        for (MatchedSamples match : matches) {
            Sample2D[] samples = match.getSamples();
            if (samples.length < MIN_NUMBER_OF_VIEWS) {
                return false;
            }

            int[] viewIds = match.getViewIds();
            int pos1 = getPositionForViewId(viewIds, viewId1);
            if(pos1 < 0) {
                return false;
            }

            int pos2 = getPositionForViewId(viewIds, viewId2);
            if (pos2 < 0) {
                return false;
            }

            Sample2D leftSample = samples[pos1];
            Sample2D rightSample = samples[pos2];
            Point2D p1 = leftSample.getPoint();
            Point2D p2 = rightSample.getPoint();

            leftSamples.add(leftSample);
            rightSamples.add(rightSample);

            Point2D leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(p1.getInhomX() - principalPointX,
                    p1.getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            Point2D rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(p2.getInhomX() - principalPointX,
                    p2.getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            FundamentalMatrixRobustEstimator estimator =
                    FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                            mConfiguration.getRobustFundamentalMatrixEstimatorMethod());
            estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                    mConfiguration.getNonRobustFundamentalMatrixEstimatorMethod());
            estimator.setResultRefined(mConfiguration.isFundamentalMatrixRefined());
            estimator.setCovarianceKept(mConfiguration.isFundamentalMatrixCovarianceKept());
            estimator.setConfidence(mConfiguration.getFundamentalMatrixConfidence());
            estimator.setMaxIterations(mConfiguration.getFundamentalMatrixMaxIterations());

            switch (mConfiguration.getRobustFundamentalMatrixEstimatorMethod()) {
                case LMedS:
                    ((LMedSFundamentalMatrixRobustEstimator)estimator).
                            setStopThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case MSAC:
                    ((MSACFundamentalMatrixRobustEstimator)estimator).
                            setThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case PROMedS:
                    ((PROMedSFundamentalMatrixRobustEstimator)estimator).
                            setStopThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case PROSAC:
                    PROSACFundamentalMatrixRobustEstimator prosacEstimator =
                            (PROSACFundamentalMatrixRobustEstimator)estimator;
                    prosacEstimator.setThreshold(
                            mConfiguration.getFundamentalMatrixThreshold());
                    prosacEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepInliers());
                    prosacEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    RANSACFundamentalMatrixRobustEstimator ransacEstimator =
                            (RANSACFundamentalMatrixRobustEstimator)estimator;
                    ransacEstimator.setThreshold(
                            mConfiguration.getFundamentalMatrixThreshold());
                    ransacEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepInliers());
                    ransacEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepResiduals());
                    break;
            }

            FundamentalMatrix fundamentalMatrix = estimator.estimate();

            mCurrentEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mCurrentEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mCurrentEstimatedFundamentalMatrix.setViewId1(viewId1);
            mCurrentEstimatedFundamentalMatrix.setViewId2(viewId2);
            mCurrentEstimatedFundamentalMatrix.setCovariance(
                    estimator.getCovariance());

            //determine quality score and inliers
            InliersData inliersData = estimator.getInliersData();
            if (inliersData != null) {
                int numInliers = inliersData.getNumInliers();
                BitSet inliers = inliersData.getInliers();
                int length = inliers.length();
                double fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        //inlier
                        fundamentalMatrixQualityScore +=
                                qualityScores[i] / numInliers;
                    }
                }
                mCurrentEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mCurrentEstimatedFundamentalMatrix.setInliers(inliers);
            }

            //store left/right samples
            mCurrentEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mCurrentEstimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates fundamental matrix for provided matches, when 3D points lay in a planar 3D scene.
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view being related by estimated fundamental matrix.
     * @param viewId2 id of second view being related by estimated fundamental matrix.
     * @param isInitialPairOfViews true if fundamental matrix needs to be estimated for the initial
     *                             pair of views, false otherwise.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimatePlanarFundamentalMatrix(List<MatchedSamples> matches, int viewId1,
                                                    int viewId2, boolean isInitialPairOfViews) {
        if (matches == null) {
            return false;
        }

        int count = matches.size();
        List<Sample2D> leftSamples = new ArrayList<>(count);
        List<Sample2D> rightSamples = new ArrayList<>(count);
        List<Point2D> leftPoints = new ArrayList<>(count);
        List<Point2D> rightPoints = new ArrayList<>(count);
        double[] qualityScores = new double[count];
        double principalPointX, principalPointY;
        if (isInitialPairOfViews) {
            if (mConfiguration.getInitialCamerasEstimatorMethod() ==
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC ||
                    mConfiguration.getInitialCamerasEstimatorMethod() ==
                            InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
                principalPointX = mConfiguration.getPrincipalPointX();
                principalPointY = mConfiguration.getPrincipalPointY();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        } else {
            if (mConfiguration.getUseDIACForAdditionalCamerasIntrinsics() ||
                    mConfiguration.getUseDAQForAdditionalCamerasIntrinsics()) {
                principalPointX = mConfiguration.getAdditionalCamerasHorizontalPrincipalPoint();
                principalPointY = mConfiguration.getAdditionalCamerasVerticalPrincipalPoint();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        }

        int i = 0;
        for (MatchedSamples match : matches) {
            Sample2D[] samples = match.getSamples();
            if (samples.length < MIN_NUMBER_OF_VIEWS) {
                return false;
            }

            int[] viewIds = match.getViewIds();
            int pos1 = getPositionForViewId(viewIds, viewId1);
            if(pos1 < 0) {
                return false;
            }

            int pos2 = getPositionForViewId(viewIds, viewId2);
            if (pos2 < 0) {
                return false;
            }

            Sample2D leftSample = samples[pos1];
            Sample2D rightSample = samples[pos2];
            Point2D p1 = leftSample.getPoint();
            Point2D p2 = rightSample.getPoint();

            leftSamples.add(leftSample);
            rightSamples.add(rightSample);

            Point2D leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(p1.getInhomX() - principalPointX,
                    p1.getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            Point2D rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(p2.getInhomX() - principalPointX,
                    p2.getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            PointCorrespondenceProjectiveTransformation2DRobustEstimator
                    homographyEstimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                            create(mConfiguration.
                                    getRobustPlanarHomographyEstimatorMethod());
            homographyEstimator.setResultRefined(
                    mConfiguration.isPlanarHomographyRefined());
            homographyEstimator.setCovarianceKept(
                    mConfiguration.isPlanarHomographyCovarianceKept());
            homographyEstimator.setConfidence(
                    mConfiguration.getPlanarHomographyConfidence());
            homographyEstimator.setMaxIterations(
                    mConfiguration.getPlanarHomographyMaxIterations());

            switch(mConfiguration.getRobustPlanarHomographyEstimatorMethod()) {
                case LMedS:
                    ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setStopThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case MSAC:
                    ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case PROMedS:
                    ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setStopThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case PROSAC:
                    PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator prosacHomographyEstimator =
                            (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)homographyEstimator;

                    prosacHomographyEstimator.setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    prosacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepInliers());
                    prosacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator ransacHomographyEstimator =
                            (RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)homographyEstimator;

                    ransacHomographyEstimator.setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    ransacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepInliers());
                    ransacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
            }

            PlanarBestFundamentalMatrixEstimatorAndReconstructor
                    fundamentalMatrixEstimator =
                    new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
            fundamentalMatrixEstimator.setHomographyEstimator(
                    homographyEstimator);
            fundamentalMatrixEstimator.setLeftAndRightPoints(leftPoints,
                    rightPoints);
            fundamentalMatrixEstimator.setQualityScores(qualityScores);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    mConfiguration.getInitialIntrinsic1();
            PinholeCameraIntrinsicParameters intrinsic2 =
                    mConfiguration.getInitialIntrinsic1();
            if (intrinsic1 == null && intrinsic2 == null) {
                //estimate homography
                ProjectiveTransformation2D homography = homographyEstimator.
                        estimate();

                //estimate intrinsic parameters using the Image of Absolute
                //Conic (IAC)
                List<Transformation2D> homographies =
                        new ArrayList<Transformation2D>();
                homographies.add(homography);

                ImageOfAbsoluteConicEstimator iacEstimator =
                        new LMSEImageOfAbsoluteConicEstimator(homographies);
                ImageOfAbsoluteConic iac = iacEstimator.estimate();

                intrinsic1 = intrinsic2 = iac.getIntrinsicParameters();

            } else if (intrinsic1 == null && intrinsic2 != null) {
                intrinsic1 = intrinsic2;
            } else if (intrinsic1 != null && intrinsic2 == null) {
                intrinsic2 = intrinsic1;
            }
            fundamentalMatrixEstimator.setLeftIntrinsics(intrinsic1);
            fundamentalMatrixEstimator.setRightIntrinsics(intrinsic2);

            fundamentalMatrixEstimator.estimateAndReconstruct();

            FundamentalMatrix fundamentalMatrix =
                    fundamentalMatrixEstimator.getFundamentalMatrix();

            mCurrentEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mCurrentEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mCurrentEstimatedFundamentalMatrix.setViewId1(viewId1);
            mCurrentEstimatedFundamentalMatrix.setViewId2(viewId2);

            //determine quality score and inliers
            InliersData inliersData = homographyEstimator.getInliersData();
            if (inliersData != null) {
                int numInliers = inliersData.getNumInliers();
                BitSet inliers = inliersData.getInliers();
                int length = inliers.length();
                double fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        //inlier
                        fundamentalMatrixQualityScore +=
                                qualityScores[i] / numInliers;
                    }
                }
                mCurrentEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mCurrentEstimatedFundamentalMatrix.setInliers(inliers);
            }

            //store left/right samples
            mCurrentEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mCurrentEstimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Gets position of a view id within provided array of view id's.
     * @param viewIds array of view id's where search is done.
     * @param viewId view id to be searched.
     * @return position where view id is found or -1 if not found.
     */
    private int getPositionForViewId(int[] viewIds, int viewId) {
        int length = viewIds.length;
        for (int i = 0; i < length; i++) {
            if (viewIds[i] == viewId) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Estimates initial cameras and reconstructed points.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPoints() {
        switch(mConfiguration.getInitialCamerasEstimatorMethod()) {
            case ESSENTIAL_MATRIX:
                return estimateInitialCamerasAndPointsEssential();
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC:
                return estimateInitialCamerasAndPointsDIAC();
            case DUAL_ABSOLUTE_QUADRIC:
                return estimateInitialCamerasAndPointsDAQ();
            case DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX:
            default:
                return estimateInitialCamerasAndPointsDAQAndEssential();
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric to estimate intrinsic parameters and then use those
     * intrinsic parameters with the essential matrix.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQAndEssential() {
        try {
            FundamentalMatrix fundamentalMatrix =
                    mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.estimate();

            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1 =
                    camera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2 =
                    camera2.getIntrinsicParameters();

            double principalPointX = mConfiguration.getPrincipalPointX();
            double principalPointY = mConfiguration.getPrincipalPointY();

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(
                    intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(
                    intrinsic1.getVerticalPrincipalPoint() + principalPointY);

            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(
                    intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(
                    intrinsic2.getVerticalPrincipalPoint() + principalPointY);

            //fix fundamental matrix to account for principal point different
            //from zero
            fixFundamentalMatrix(fundamentalMatrix,
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            return estimateInitialCamerasAndPointsEssential(intrinsic1,
                    intrinsic2);
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQ() {
        try {
            FundamentalMatrix fundamentalMatrix =
                    mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.estimate();

            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1 =
                    camera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2 =
                    camera2.getIntrinsicParameters();

            double principalPointX = mConfiguration.getPrincipalPointX();
            double principalPointY = mConfiguration.getPrincipalPointY();

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(
                    intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(
                    intrinsic1.getVerticalPrincipalPoint() + principalPointY);
            camera1.setIntrinsicParameters(intrinsic1);

            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(
                    intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(
                    intrinsic2.getVerticalPrincipalPoint() + principalPointY);
            camera2.setIntrinsicParameters(intrinsic2);

            mPreviousMetricEstimatedCamera = new EstimatedCamera();
            mPreviousMetricEstimatedCamera.setCamera(camera1);
            mPreviousMetricEstimatedCamera.setId(String.valueOf(mPreviousViewId));

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(camera2);
            mCurrentMetricEstimatedCamera.setId(String.valueOf(mCurrentViewId));

            //fix fundamental matrix to account for principal point different
            //from zero
            fixFundamentalMatrix(fundamentalMatrix,
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            //triangulate points
            Corrector corrector = null;
            if (mConfiguration.getInitialCamerasCorrectorType() != null) {
                corrector = Corrector.create(fundamentalMatrix,
                        mConfiguration.getInitialCamerasCorrectorType());
            }

            //use all points used for fundamental matrix estimation
            List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
            List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

            List<Point2D> points1 = new ArrayList<Point2D>();
            List<Point2D> points2 = new ArrayList<Point2D>();
            int length = samples1.size();
            for (int i = 0; i < length; i++) {
                Sample2D sample1 = samples1.get(i);
                Sample2D sample2 = samples2.get(i);

                Point2D point1 = sample1.getPoint();
                Point2D point2 = sample2.getPoint();

                points1.add(point1);
                points2.add(point2);
            }

            //correct points if needed
            List<Point2D> correctedPoints1;
            List<Point2D> correctedPoints2;
            if (corrector != null) {
                corrector.setLeftAndRightPoints(points1, points2);
                corrector.correct();

                correctedPoints1 = corrector.getLeftCorrectedPoints();
                correctedPoints2 = corrector.getRightCorrectedPoints();
            } else {
                correctedPoints1 = points1;
                correctedPoints2 = points2;
            }


            //triangulate points
            SinglePoint3DTriangulator triangulator;
            if (mConfiguration.getDaqUseHomogeneousPointTriangulator()) {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);
            } else {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.
                                LMSE_INHOMOGENEOUS_TRIANGULATOR);
            }

            List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
            cameras.add(camera1);
            cameras.add(camera2);

            mActiveMetricReconstructedPoints = new ArrayList<>();
            List<Point2D> points = new ArrayList<Point2D>();
            int numPoints = correctedPoints1.size();
            Point3D triangulatedPoint;
            ReconstructedPoint3D reconstructedPoint;
            for (int i = 0; i < numPoints; i++) {
                points.clear();
                points.add(correctedPoints1.get(i));
                points.add(correctedPoints2.get(i));

                triangulator.setPointsAndCameras(points, cameras);
                triangulatedPoint = triangulator.triangulate();

                reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoint);

                //only points reconstructed in front of both cameras are
                //considered valid
                boolean front1 = camera1.isPointInFrontOfCamera(
                        triangulatedPoint);
                boolean front2 = camera2.isPointInFrontOfCamera(
                        triangulatedPoint);
                reconstructedPoint.setInlier(front1 && front2);

                mActiveMetricReconstructedPoints.add(reconstructedPoint);
            }

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using Dual Image of
     * Absolute Conic.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDIAC() {
        FundamentalMatrix fundamentalMatrix =
                mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

        //use inlier points used for fundamental matrix estimation
        List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

        List<Point2D> points1 = new ArrayList<Point2D>();
        List<Point2D> points2 = new ArrayList<Point2D>();
        int length = samples1.size();
        for (int i = 0; i < length; i++) {
            Sample2D sample1 = samples1.get(i);
            Sample2D sample2 = samples2.get(i);

            Point2D point1 = sample1.getPoint();
            Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            DualImageOfAbsoluteConicInitialCamerasEstimator estimator =
                    new DualImageOfAbsoluteConicInitialCamerasEstimator(
                            fundamentalMatrix, points1, points2);
            estimator.setPrincipalPoint(mConfiguration.getPrincipalPointX(),
                    mConfiguration.getPrincipalPointY());
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.setCorrectorType(
                    mConfiguration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getInitialCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            //store cameras
            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            mPreviousMetricEstimatedCamera = new EstimatedCamera();
            mPreviousMetricEstimatedCamera.setCamera(camera1);

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(camera2);

            //store points
            List<Point3D> triangulatedPoints =
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();

            mActiveMetricReconstructedPoints = new ArrayList<ReconstructedPoint3D>();
            int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mActiveMetricReconstructedPoints.add(reconstructedPoint);
            }

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential() {
        PinholeCameraIntrinsicParameters intrinsic1 =
                mConfiguration.getInitialIntrinsic1();
        PinholeCameraIntrinsicParameters intrinsic2 =
                mConfiguration.getInitialIntrinsic2();
        return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2);
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     * @param intrinsic1 intrinsic parameters of 1st camera.
     * @param intrinsic2 intrinsic parameters of 2nd camera.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential(
            PinholeCameraIntrinsicParameters intrinsic1,
            PinholeCameraIntrinsicParameters intrinsic2) {
        FundamentalMatrix fundamentalMatrix =
                mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

        //use all points used for fundamental matrix estimation
        List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

        List<Point2D> points1 = new ArrayList<Point2D>();
        List<Point2D> points2 = new ArrayList<Point2D>();
        int length = samples1.size();
        for (int i = 0; i < length; i++) {
            Sample2D sample1 = samples1.get(i);
            Sample2D sample2 = samples2.get(i);

            Point2D point1 = sample1.getPoint();
            Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            EssentialMatrixInitialCamerasEstimator estimator =
                    new EssentialMatrixInitialCamerasEstimator(
                            fundamentalMatrix, intrinsic1, intrinsic2,
                            points1, points2);

            estimator.setCorrectorType(
                    mConfiguration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getInitialCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            //store cameras
            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            mPreviousMetricEstimatedCamera = new EstimatedCamera();
            mPreviousMetricEstimatedCamera.setCamera(camera1);
            mPreviousMetricEstimatedCamera.setId(String.valueOf(mPreviousViewId));

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(camera2);
            mCurrentMetricEstimatedCamera.setId(String.valueOf(mCurrentViewId));


            //store points
            List<Point3D> triangulatedPoints =
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();

            mActiveMetricReconstructedPoints = new ArrayList<ReconstructedPoint3D>();
            int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mActiveMetricReconstructedPoints.add(reconstructedPoint);
            }

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Fixes fundamental matrix to account for principal point different from
     * zero when using DAQ estimation.
     * @param fundamentalMatrix fundamental matrix to be fixed.
     * @param intrinsicZeroPrincipalPoint1 intrinsic parameters of camera 1
     * assuming zero principal point.
     * @param intrinsicZeroPrincipalPoint2 intrinsic parameters of camera 2
     * assuming zero principal point.
     * @param intrinsicPrincipalPoint1 intrinsic parameters of camera 1 using
     * proper principal point.
     * @param intrinsicPrincipalPoint2 intrinsic parameters of camera 2 using
     * proper principal point.
     * @throws EpipolarException if something fails.
     * @throws NotReadyException never happens.
     */
    private void fixFundamentalMatrix(FundamentalMatrix fundamentalMatrix,
                                      PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1,
                                      PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2,
                                      PinholeCameraIntrinsicParameters intrinsicPrincipalPoint1,
                                      PinholeCameraIntrinsicParameters intrinsicPrincipalPoint2)
            throws EpipolarException, NotReadyException {

        //first compute essential matrix as E = K2a'F*K1a
        EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix,
                intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2);
        FundamentalMatrix fixedFundamentalMatrix =
                essential.toFundamentalMatrix(intrinsicPrincipalPoint1,
                        intrinsicPrincipalPoint2);
        fixedFundamentalMatrix.normalize();
        mCurrentEstimatedFundamentalMatrix.setFundamentalMatrix(
                fixedFundamentalMatrix);
        mCurrentEstimatedFundamentalMatrix.setCovariance(null);
    }
}
