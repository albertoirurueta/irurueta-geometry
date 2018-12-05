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
import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best 
 * PinholeCamera for provided collections of matched 3D points and their 
 * corresponding projected 2D points, or collections of matched 3D planes and 
 * their corresponding projected 2D lines (depending whether point 
 * correspondence or plane/line correspondence is being used).
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class PinholeCameraRobustEstimator {
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
     * Indicates whether fast refinement is used by default.
     */
    public static final boolean DEFAULT_USE_FAST_REFINEMENT = false;
    
    /**
     * Default amount of progress variation before notifying a change in 
     * estimation progress. By default this is set to 5%.
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
     * accurate because chosen subsamples will be inliers.
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
     * Default value indicating whether skewness value is suggested or not.
     * By default this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED = false;
    
    /**
     * Default value of skewness to be suggested when suggestion is enabled.
     * By default suggested skewness is zero.
     */
    public static final double DEFAULT_SUGGESTED_SKEWNESS_VALUE = 0.0;
    
    /**
     * Default value indicating whether horizontal focal length value is 
     * suggested or not. By default this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED = 
            false;    
    
    /**
     * Default value indicating whether vertical focal length value is suggested
     * or not. By default this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED =
            false;

    /**
     * Default value indicating whether aspect ratio is suggested or not. By
     * default this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED = false;
    
    /**
     * Default value of aspect ratio to be suggested when suggestion is enabled.
     * By default suggested aspect ratio is 1.0, although also -1.0 is a typical
     * value when vertical coordinates increase downwards.
     */
    public static final double DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE = 1.0;
    
    /**
     * Default value indicating whether principal point is suggested or not. By
     * default this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED = false;
    
    /**
     * Default value indicating whether rotation is suggested or not. By default
     * this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_ROTATION_ENABLED = false;
    
    /**
     * Default value indicating whether center is suggested or not. By default
     * this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_CENTER_ENABLED = false;        

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected PinholeCameraRobustEstimatorListener mListener;
    
    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected volatile boolean mLocked;
    
    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float mProgressDelta;
    
    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double mConfidence;
    
    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int mMaxIterations;  
    
    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData mInliersData;    
    
    /**
     * Indicates whether result must be refined using Levenberg-Marquardt 
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation 
     * regardless of the settings.
     */
    protected boolean mRefineResult;
    
    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    protected boolean mKeepCovariance;
    
    /**
     * Indicates whether fast refinement must be used or not.
     * When true Levenberg/Marquardt refinement will be used, when false
     * an initial Powell optimization is done and then Levenberg/Marquard is
     * used for covariance estimation if needed.
     */
    protected boolean mUseFastRefinement;
    
    /**
     * Estimated covariance of estimated fundamental matrix.
     * This is only available when result has been refined and covariance is 
     * kept.
     */
    protected Matrix mCovariance;   
    
    /**
     * Indicates whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested 
     * value.
     */
    protected boolean mSuggestSkewnessValueEnabled = 
            DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED;
    
    /**
     * Suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided
     * value.
     */
    protected double mSuggestedSkewnessValue = DEFAULT_SUGGESTED_SKEWNESS_VALUE;
    
    /**
     * Indicates whether horizontal focal length is suggested or not. When 
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean mSuggestHorizontalFocalLengthEnabled = 
            DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;
    
    /**
     * Suggested horizontal focal length value to be reached when suggestion is
     * enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected double mSuggestedHorizontalFocalLengthValue;
    
    /**
     * Indicates whether vertical focal length is suggested or not. When 
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean mSuggestVerticalFocalLengthEnabled = 
            DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;
    
    /**
     * Suggested vertical focal length value to be reached when suggestion is
     * enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected double mSuggestedVerticalFocalLengthValue;
    
    /**
     * Indicates whether aspect ratio is suggested or not. When enabled, the 
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean mSuggestAspectRatioEnabled = 
            DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED;
    
    /**
     * Suggested aspect ratio value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected double mSuggestedAspectRatioValue = 
            DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE;
    
    /**
     * Indicates whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean mSuggestPrincipalPointEnabled = 
            DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED;
    
    /**
     * Suggested principal point value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected InhomogeneousPoint2D mSuggestedPrincipalPointValue;
    
    /**
     * Indicates whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean mSuggestRotationEnabled = 
            DEFAULT_SUGGEST_ROTATION_ENABLED;
    
    /**
     * Suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected Quaternion mSuggestedRotationValue;
    
    /**
     * Indicates whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean mSuggestCenterEnabled;
    
    /**
     * Suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected InhomogeneousPoint3D mSuggestedCenterValue;    
    
    /**
     * Instance to be reused to compute residual for intrinsic parameters.
     */
    private PinholeCameraIntrinsicParameters mResidualIntrinsic;
    
    /**
     * Instance to be reused to compute residual on principal point.
     */
    private InhomogeneousPoint2D mResidualPrincipalPoint;
    
    /**
     * Instance to be reused to compute residual on rotation.
     */
    private Quaternion mResidualRotation;
    
    /**
     * Instance to be reused to compute center.
     */
    private InhomogeneousPoint3D mResidualCenter;    
            
    /**
     * Constructor.
     */
    public PinholeCameraRobustEstimator() {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;     
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener) {
        mListener = listener;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;   
        mRefineResult = DEFAULT_REFINE_RESULT;
        mKeepCovariance = DEFAULT_KEEP_COVARIANCE;        
    }
    
    /**
     * Returns reference to listener to be notified of events such as when 
     * estimation starts, ends or its progress significantly changes.
     * @return listener to be notified of events.
     */
    public PinholeCameraRobustEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(
            PinholeCameraRobustEstimatorListener listener) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Indicates whether listener has been provided and is available for 
     * retrieval.
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return mListener != null;
    }
    
    /**
     * Indicats whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested 
     * value.
     * @return true if skewness value is suggested, false otherwise.
     */
    public boolean isSuggestSkewnessValueEnabled() {
        return mSuggestSkewnessValueEnabled;
    }
    
    /**
     * Specifies whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestSkewnessValueEnabled true if skewness value is suggested, 
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestSkewnessValueEnabled(
            boolean suggestSkewnessValueEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestSkewnessValueEnabled = suggestSkewnessValueEnabled;
    }
    
    /**
     * Gets suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested skewness value.
     */
    public double getSuggestedSkewnessValue() {
        return mSuggestedSkewnessValue;
    }
    
    /**
     * Sets suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedSkewnessValue suggested skewness value.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedSkewnessValue(double suggestedSkewnessValue) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedSkewnessValue = suggestedSkewnessValue;
    }
    
    /**
     * Indicates whether horizontal focal length is suggested or not. When 
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @return true if horizontal focal length is suggested, false otherwise.
     */
    public boolean isSuggestHorizontalFocalLengthEnabled() {
        return mSuggestHorizontalFocalLengthEnabled;
    }
    
    /**
     * Specifies whether horizontal focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestHorizontalFocalLengthEnabled true if horizontal focal 
     * length is suggested, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestHorizontalFocalLengthEnabled(
            boolean suggestHorizontalFocalLengthEnabled) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestHorizontalFocalLengthEnabled = 
                suggestHorizontalFocalLengthEnabled;
    }
    
    /**
     * Gets suggested horizontal focal length value to be reached when 
     * suggestion is enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested horizontal focal length value.
     */
    public double getSuggestedHorizontalFocalLengthValue() {
        return mSuggestedHorizontalFocalLengthValue;
    }
    
    /**
     * Sets suggested horizontal focal length value to be reached when 
     * suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedHorizontalFocalLengthValue suggested horizontal focal 
     * length value.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedHorizontalFocalLengthValue(
            double suggestedHorizontalFocalLengthValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedHorizontalFocalLengthValue = 
                suggestedHorizontalFocalLengthValue;
    }
    
    /**
     * Indicates whether vertical focal length is suggested or not. When 
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @return true if vertical focal length is suggested, false otherwise.
     */
    public boolean isSuggestVerticalFocalLengthEnabled() {
        return mSuggestVerticalFocalLengthEnabled;
    }
    
    /**
     * Specifies whether vertical focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestVerticalFocalLengthEnabled true if vertical focal length is
     * suggested, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestVerticalFocalLengthEnabled(
            boolean suggestVerticalFocalLengthEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestVerticalFocalLengthEnabled = suggestVerticalFocalLengthEnabled;
    }
    
    /**
     * Gets suggested vertical focal length value to be reached when suggestion 
     * is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested vertical focal length.
     */
    public double getSuggestedVerticalFocalLengthValue() {
        return mSuggestedVerticalFocalLengthValue;
    }
    
    /**
     * Sets suggested vertical focal length value to be reached when suggestion
     * is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedVerticalFocalLengthValue suggested vertical focal length.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedVerticalFocalLengthValue(
            double suggestedVerticalFocalLengthValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedVerticalFocalLengthValue = suggestedVerticalFocalLengthValue;
    }
    
    /**
     * Indicates whether aspect ratio is suggested or not. When enabled, the 
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested 
     * value.
     * @return true if aspect ratio is suggested, false otherwise.
     */
    public boolean isSuggestAspectRatioEnabled() {
        return mSuggestAspectRatioEnabled;
    }
    
    /**
     * Specifies whether aspect ratio is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestAspectRatioEnabled true if aspect ratio is suggested, false 
     * otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestAspectRatioEnabled(boolean suggestAspectRatioEnabled) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestAspectRatioEnabled = suggestAspectRatioEnabled;
    }
    
    /**
     * Gets suggested aspect ratio value to be reached when suggestion is 
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested aspect ratio value.
     */
    public double getSuggestedAspectRatioValue() {
        return mSuggestedAspectRatioValue;
    }
    
    /**
     * Sets suggested aspect ratio value to be reached when suggestion is
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedAspectRatioValue suggested aspect ratio value.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedAspectRatioValue(double suggestedAspectRatioValue) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedAspectRatioValue = suggestedAspectRatioValue;
    }
    
    /**
     * Indicates whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested 
     * value.
     * @return true if principal point is suggested, false otherwise.
     */
    public boolean isSuggestPrincipalPointEnabled() {
        return mSuggestPrincipalPointEnabled;
    }
    
    /**
     * Specifies whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestPrincipalPointEnabled true if principal point is suggested,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestPrincipalPointEnabled(
            boolean suggestPrincipalPointEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestPrincipalPointEnabled = suggestPrincipalPointEnabled;
        if (suggestPrincipalPointEnabled && 
                mSuggestedPrincipalPointValue == null) {
            mSuggestedPrincipalPointValue = new InhomogeneousPoint2D();
        }
    }
    
    /**
     * Gets suggested principal point value to be reached when suggestion is 
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested principal point value to be reached when suggestion is
     * enabled.
     */
    public InhomogeneousPoint2D getSuggestedPrincipalPointValue() {
        return mSuggestedPrincipalPointValue;
    }
    
    /**
     * Sets suggested principal point value to be reached when suggestion is
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedPrincipalPointValue suggested principal point value to be
     * reached when suggestion is enabled.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedPrincipalPointValue(
            InhomogeneousPoint2D suggestedPrincipalPointValue) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedPrincipalPointValue = suggestedPrincipalPointValue;
    }
    
    /**
     * Indicates whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @return true if camera rotation is suggested, false otherwise.
     */
    public boolean isSuggestRotationEnabled() {
        return mSuggestRotationEnabled;
    }
    
    /**
     * Specifies whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestRotationEnabled true if camera rotation is suggested, false
     * otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestRotationEnabled(boolean suggestRotationEnabled) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestRotationEnabled = suggestRotationEnabled;
        if (suggestRotationEnabled && mSuggestedRotationValue == null) {
            mSuggestedRotationValue = new Quaternion();
        }
        
    }
    
    /**
     * Gets suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value 
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested rotation to be reached when suggestion is enabled.
     */
    public Quaternion getSuggestedRotationValue() {
        return mSuggestedRotationValue;
    }
    
    /**
     * Sets suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedRotationValue suggested rotation to be reached when 
     * suggestion is enabled.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedRotationValue(Quaternion suggestedRotationValue)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedRotationValue = suggestedRotationValue;
    }
    
    /**
     * Indicates whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @return true if camera center is suggested, false otherwise.
     */
    public boolean isSuggestCenterEnabled() {
        return mSuggestCenterEnabled;
    }
    
    /**
     * Specifies whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     * @param suggestCenterEnabled true if camera is suggested, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestCenterEnabled(boolean suggestCenterEnabled) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestCenterEnabled = suggestCenterEnabled;
        if (suggestCenterEnabled && mSuggestedCenterValue == null) {
            mSuggestedCenterValue = new InhomogeneousPoint3D();
        }
    }
    
    /**
     * Gets suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @return suggested center to be reached when suggestion is enabled.
     */
    public InhomogeneousPoint3D getSuggestedCenterValue() {
        return mSuggestedCenterValue;
    }
    
    /**
     * Sets suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     * @param suggestedCenterValue suggested center to be reached when 
     * suggestion is enabled.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedCenterValue(
            InhomogeneousPoint3D suggestedCenterValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSuggestedCenterValue = suggestedCenterValue;
    }    
    
    /**
     * Indicates if this instance is locked because estimation is being 
     * computed.
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change 
     * during estimation.
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }
    
    /**
     * Sets amount of progress variation before notifying a progress change 
     * during estimation.
     * @param progressDelta amount of progress variation before notifying a 
     * progress change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     * greater than 1.
     * @throws LockedException if this estimator is locked because an estimation
     * is being computed.
     */
    public void setProgressDelta(float progressDelta) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || 
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }
    
    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mConfidence;
    }
    
    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the 
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 
     * 1.0.
     * @throws LockedException if this estimator is locked because an estimator 
     * is being computed.
     */
    public void setConfidence(double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
    }
    
    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling 
     * estimate(), a RobustEstimatorException will be raised.
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }
    
    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an 
     * approximate result will be available for retrieval.
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked because an estimation
     * is being computed.
     */
    public void setMaxIterations(int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mMaxIterations = maxIterations;
    }    
    
    /**
     * Gets data related to inliers found after estimation.
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mInliersData;
    }  
    
    /**
     * Indicates whether result must be refined using Levenberg-Marquardt 
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation 
     * regardless of the settings.
     * @return true to refine result, false to simply use result found by
     * robust estimator without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }
    
    /**
     * Specifies whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * @param refineResult true to refine result, false to simply use result 
     * found by robust estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setResultRefined(boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefineResult = refineResult;
    }
    
    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     * @return true if covariance must be kept after refining result, false
     * otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }
    
    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     * @param keepCovariance true if covariance must be kept after refining 
     * result, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(boolean keepCovariance) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }
    
    /**
     * Indicates whether fast refinement must be used or not.
     * When true Levenberg/Marquardt refinement will be used, when false
     * an initial Powell optimization is done and then Levenberg/Marquard is
     * used for covariance estimation if needed.
     * Fast refinement requires less computing time but it is more likely to
     * fail than slow one.
     * @return true to use fast refinement, false to use a slow but more
     * accurate and stable refinement.
     */
    public boolean isFastRefinementUsed() {
        return mUseFastRefinement;
    }
    
    /**
     * Specifies whether fast refinement must be used or not.
     * When true Levenberg/Marquardt refinement will be used, when false
     * an initial Powell optimization is done and then Levenberg/Marquard is
     * used for covariance estimation if needed.
     * Fast refinement requires less computing time but it is more likely to
     * fail than slow one.
     * @param useFastRefinement true to use fast refinement, false to use a slow
     * but more accurate and stable refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setFastRefinementUsed(boolean useFastRefinement) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseFastRefinement = useFastRefinement;
    }    
    
    /**
     * Gets estimated covariance of estimated pinhole camera if available.
     * This is only available when result has been refined and covariance is 
     * kept.
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }
    
    /**
     * Estimates a pinhole camera using a robust estimator and
     * the best set of matched 2D/3D point correspondences or 2D line/3D plane 
     * correspondences found using the robust estimator.
     * @return a pinhole camera.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    public abstract PinholeCamera estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException;
        
    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();    
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust 
     * estimation, since residuals of found inliers are within the range of 
     * such threshold.
     * @return standard deviation used for refinement.
     */
    protected abstract double getRefinementStandardDeviation();    
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided robust estimator method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            List<Point3D> points3D, List<Point2D> points2D, 
            RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, 
                points2D, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D,
            RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, 
                points3D, points2D, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided quality scores and robust estimator
     * method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, 
                points2D, qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                points3D, points2D, qualityScores, method);
    }  
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using default robust estimator method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            List<Point3D> points3D, List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, 
                points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and default robust estimator 
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                points3D, points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided quality scores and default robust 
     * estimator method.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, 
                points2D, qualityScores);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided listener, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                points3D, points2D, qualityScores);
    }  

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, 
                points3D, points2D, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, 
                intrinsic, points3D, points2D, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided quality scores and robust estimator
     * method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D, double[] qualityScores, 
            RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, 
                points3D, points2D, qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D, 
            double[] qualityScores, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                intrinsic, points3D, points2D, qualityScores, method);
    }  
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using default robust estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, 
                points3D, points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and default robust estimator 
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                intrinsic, points3D, points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided quality scores and default robust 
     * estimator method.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D, double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, 
                points3D, points2D, qualityScores);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided listener, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D,
            List<Point2D> points2D, double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                intrinsic, points3D, points2D, qualityScores);
    }  
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided robust estimator method.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size 
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D, RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, 
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided quality scores and robust estimator
     * method.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D, double[] qualityScores, 
            RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * points.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D, double[] qualityScores, 
            RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores, method);
    }  
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using default robust estimator method.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and default robust estimator 
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size
     * (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided quality scores and default robust 
     * estimator method.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D, 
            List<Point2D> points2D, double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint, points3D, 
                points2D, qualityScores);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point 
     * correspondences and using provided listener, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param skewness skewness value of intrinsic parameters of camera to be 
     * estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     * intrinsic parameters of cmera to be estimated.
     * @param verticalPrincipalPoint  vertical principal point value of 
     * intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to 
     * estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size  or their size is smaller than required
     * minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            PinholeCameraRobustEstimatorListener listener,
            double skewness, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, List<Point3D> points3D,
            List<Point2D> points2D, double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener,
                skewness, horizontalPrincipalPoint, verticalPrincipalPoint, 
                points3D, points2D, qualityScores);
    }  
    
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided robust estimator method.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     * have the same size or their size is smaller than required minimum size (4
     * correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            List<Plane> planes, List<Line2D> lines, 
            RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     * have the same size or their size is smaller than required minimum size (4
     * correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines,
            RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, method);
    }
        
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided quality scores and robust estimator 
     * method.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * samples.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            List<Plane> planes, List<Line2D> lines, double[] qualityScores,
            RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener, quality scores and robust 
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * samples.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            PinholeCameraRobustEstimatorListener listener, List<Plane> planes,
            List<Line2D> lines, double[] qualityScores, 
            RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using default robust estimator method.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     * have the same size or their size is smaller than required minimum size (4
     * correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            List<Plane> planes, List<Line2D> lines) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener and default robust estimator 
     * method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     * have the same size or their size is smaller than required minimum size (4
     * correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines);
    }
        
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided quality scores and default robust 
     * estimator method.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * samples.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            List<Plane> planes, List<Line2D> lines, double[] qualityScores) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of mtched
     * samples.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            PinholeCameraRobustEstimatorListener listener, List<Plane> planes,
            List<Line2D> lines, double[] qualityScores) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores);
    }   
        
    /**
     * Residual term for any required suggestions.
     * @param params parameters being optimized. In the following order: 
     * skewness, horizontal focal length, vertical focal length, 
     * horizontal principal point, vertical principal point, quaternion A,
     * quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight weight to apply to obtained residual. This weight increases
     * on each iteration to help into achieving required suggested values.
     * @return term for any required suggestion.
     */
    protected double suggestionResidual(double[] params, double weight) {
        double residual = 0.0;
        
        if (mSuggestSkewnessValueEnabled) {
            residual += Math.pow(params[0] - mSuggestedSkewnessValue, 2.0);
        }
        
        if (mSuggestHorizontalFocalLengthEnabled) {
            residual += Math.pow(
                    params[1] - mSuggestedHorizontalFocalLengthValue, 2.0);
        }
        
        if (mSuggestVerticalFocalLengthEnabled) {
            residual += Math.pow(
                    params[2] - mSuggestedVerticalFocalLengthValue, 2.0);
        }
        
        if (mSuggestAspectRatioEnabled) {
            double aspectRatio = params[2] / params[1];
            residual += Math.pow(aspectRatio - mSuggestedAspectRatioValue, 2.0);
        }
        
        if (mSuggestPrincipalPointEnabled) {
            if (mResidualPrincipalPoint == null) {
                mResidualPrincipalPoint = new InhomogeneousPoint2D(
                        params[3], params[4]);
            } else {
                mResidualPrincipalPoint.setInhomogeneousCoordinates(
                        params[3], params[4]);
            }
            
            residual += Math.pow(mResidualPrincipalPoint.distanceTo(
                    mSuggestedPrincipalPointValue), 2.0);
        }
        
        if (mSuggestRotationEnabled) {
            if (mResidualRotation == null) {
                mResidualRotation = new Quaternion(params[5], params[6], 
                        params[7], params[8]);
            } else {
                mResidualRotation.setA(params[5]);
                mResidualRotation.setB(params[6]);
                mResidualRotation.setC(params[7]);
                mResidualRotation.setD(params[8]);
            }
            mResidualRotation.normalize();
            mSuggestedRotationValue.normalize();
            residual += Math.pow(mResidualRotation.getA() - 
                    mSuggestedRotationValue.getA(), 2.0) + 
                    Math.pow(mResidualRotation.getB() - 
                    mSuggestedRotationValue.getB(), 2.0) +
                    Math.pow(mResidualRotation.getC() - 
                    mSuggestedRotationValue.getC(), 2.0) +
                    Math.pow(mResidualRotation.getD() - 
                    mSuggestedRotationValue.getD(), 2.0);            
        }
        
        if (mSuggestCenterEnabled) {
            if (mResidualCenter == null) {
                mResidualCenter = new InhomogeneousPoint3D(
                        params[9], params[10], params[11]);
            } else {
                mResidualCenter.setInhomogeneousCoordinates(
                        params[9], params[10], params[11]);
            }
            residual += Math.pow(
                    mResidualCenter.distanceTo(mSuggestedCenterValue), 2.0);
        }
        
        return weight * residual;
    }    
    
    /**
     * Sets array of parameters into a pinhole camera.
     * This method is used internally during refinement.
     * @param params parameters to be set. In the following order: 
     * skewness, horizontal focal length, vertical focal length, 
     * horizontal principal point, vertical principal point, quaternion A,
     * quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param result instance where parameters will be set.
     */
    protected void parametersToCamera(double[] params, 
            PinholeCamera result) {
        
        if (mResidualIntrinsic == null) {
            mResidualIntrinsic = new PinholeCameraIntrinsicParameters();
        }
        mResidualIntrinsic.setSkewness(params[0]);
        mResidualIntrinsic.setHorizontalFocalLength(params[1]);
        mResidualIntrinsic.setVerticalFocalLength(params[2]);
        mResidualIntrinsic.setHorizontalPrincipalPoint(params[3]);
        mResidualIntrinsic.setVerticalPrincipalPoint(params[4]);
        
        if (mResidualRotation == null) {
            mResidualRotation = new Quaternion(params[5], params[6], 
                    params[7], params[8]);
        } else {
            mResidualRotation.setA(params[5]);
            mResidualRotation.setB(params[6]);
            mResidualRotation.setC(params[7]);
            mResidualRotation.setD(params[8]);
        }
        mResidualRotation.normalize();
        
        if (mResidualCenter == null) {
            mResidualCenter = new InhomogeneousPoint3D(params[9], params[10], 
                    params[11]);
        } else {
            mResidualCenter.setInhomogeneousCoordinates(params[9], params[10],
                    params[11]);
        }
        mResidualCenter.normalize();
        
        result.setIntrinsicAndExtrinsicParameters(mResidualIntrinsic, 
                mResidualRotation, mResidualCenter);     
        result.normalize();
    }   
    
    /**
     * Sets camera parameters into array of parameters.
     * This method is used internally during refinement.
     * @param camera camera to obtain parameters to be set into array.
     * @param result array where extracted parameters are stored. In the 
     * following order: 
     * skewness, horizontal focal length, vertical focal length, 
     * horizontal principal point, vertical principal point, quaternion A,
     * quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @throws CameraException if camera cannot be decomposed.
     * @throws NotAvailableException if any camera component cannot be 
     * retrieved.
     */
    protected void cameraToParameters(PinholeCamera camera, double[] result) 
            throws CameraException, NotAvailableException {
        
        camera.decompose();
        
        PinholeCameraIntrinsicParameters intrinsic =
                camera.getIntrinsicParameters();
        result[0] = intrinsic.getSkewness();
        result[1] = intrinsic.getHorizontalFocalLength();
        result[2] = intrinsic.getVerticalFocalLength();
        result[3] = intrinsic.getHorizontalPrincipalPoint();
        result[4] = intrinsic.getVerticalPrincipalPoint();
        
        Rotation3D rotation = camera.getCameraRotation();
        if (mResidualRotation == null) {
            mResidualRotation = rotation.toQuaternion();
        } else {
            rotation.toQuaternion(mResidualRotation);
        }
        mResidualRotation.normalize();
        
        result[5] = mResidualRotation.getA();
        result[6] = mResidualRotation.getB();
        result[7] = mResidualRotation.getC();
        result[8] = mResidualRotation.getD();
        
        Point3D center = camera.getCameraCenter();
        
        result[9] = center.getInhomX();
        result[10] = center.getInhomY();
        result[11] = center.getInhomZ();        
    }    
    
    /**
     * Indicates whether obtained solution requires refinement to apply provided
     * suggestions.
     * @return true if solution requires refinement to apply provided 
     * suggestions, false otherwise.
     */
    protected boolean hasSuggestions() {
        return hasIntrinsicSuggestions() || hasExtrinsicSuggestions();
    }
    
    /**
     * Indicates whether suggestions for any intrinsic parameter are required
     * or not.
     * @return true if suggestions for any intrinsic parameters are required, 
     * false otherwise.
     */
    private boolean hasIntrinsicSuggestions() {
        return mSuggestSkewnessValueEnabled || 
                mSuggestHorizontalFocalLengthEnabled || 
                mSuggestVerticalFocalLengthEnabled || 
                mSuggestAspectRatioEnabled;
    }
    
    /**
     * Indicates whether suggestions for any extrinsic parameter are required
     * or not.
     * @return true if suggestions for any extrinsic parameter are required, 
     * false otherwise.
     */
    private boolean hasExtrinsicSuggestions() {
        return mSuggestPrincipalPointEnabled ||
                mSuggestRotationEnabled || mSuggestCenterEnabled;
    }                    
}
