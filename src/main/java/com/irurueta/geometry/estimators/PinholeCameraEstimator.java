/**
 * @file
 * This file contains definition of
 * com.irurueta.geometry.estimators.PinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 17, 2013
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Quaternion;

/**
 * This class defines the interface for an estimator for pinhole cameras.
 */
public abstract class PinholeCameraEstimator {
    
    /**
     * Default estimator type.
     */
    public static final PinholeCameraEstimatorType DEFAULT_ESTIMATOR_TYPE =
            PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR;   
    
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
     * Default value for minimum suggestion weight. This weight is used to 
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested 
     * values can be converged if they differ too much from the original ones.
     */
    public static final double DEFAULT_MIN_SUGGESTION_WEIGHT = 0.1;
    
    /**
     * Default value for maximum suggestion weight. This weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    public static final double DEFAULT_MAX_SUGGESTION_WEIGHT = 2.0;
    
    /**
     * Default value for the step to increase suggestion weight. This weight is 
     * used to slowly draw original camera parameters into desired suggested 
     * values. Suggestion weight slowly increases each time Levenberg-Marquardt 
     * is used to find a solution so that the algorithm can converge into 
     * desired value. The faster the weights are increased the less likely that 
     * suggested values can be converged if they differ too much from the 
     * original ones.
     */
    public static final double DEFAULT_SUGGESTION_WEIGHT_STEP = 0.475;
        
    /**
     * True when an estimator is estimating a camera.
     */
    protected boolean mLocked;
    
    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or estimation progress changes.
     */
    protected PinholeCameraEstimatorListener mListener;
    
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
     * Minimum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested 
     * values can be converged if they differ too much from the original ones.
     */
    protected double mMinSuggestionWeight = DEFAULT_MIN_SUGGESTION_WEIGHT;
    
    /**
     * Maximum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    protected double mMaxSuggestionWeight = DEFAULT_MAX_SUGGESTION_WEIGHT;

    /**
     * Step to increase suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values. Suggestion 
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The 
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     */
    protected double mSuggestionWeightStep = DEFAULT_SUGGESTION_WEIGHT_STEP;    
    
    /**
     * Constructor.
     */
    public PinholeCameraEstimator() {
        mLocked = false;
        mListener = null;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public PinholeCameraEstimator(PinholeCameraEstimatorListener listener) {
        mLocked = false;
        mListener = listener;
    }
    
    /**
     * Returns listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @return listener to be notified of events.
     */
    public PinholeCameraEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(PinholeCameraEstimatorListener listener)
        throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
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
     * Gets minimum suggestion weight. This weight is used to slowly draw 
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested 
     * values can be converged if they differ too much from the original ones.
     * @return minimum suggestion weight.
     */
    public double getMinSuggestionWeight() {
        return mMinSuggestionWeight;
    }
    
    /**
     * Sets minimum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @param minSuggestionWeight minimum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMinSuggestionWeight(double minSuggestionWeight) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mMinSuggestionWeight = minSuggestionWeight;
    }
    
    /**
     * Gets maximum suggestion weight. This weight is used to slowly draw 
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @return maximum suggestion weight.
     */
    public double getMaxSuggestionWeight() {
        return mMaxSuggestionWeight;
    }
    
    /**
     * Sets maximum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMaxSuggestionWeight(double maxSuggestionWeight)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mMaxSuggestionWeight = maxSuggestionWeight;
    }
    
    /**
     * Sets minimum and maximum suggestion weights. Suggestion weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     * @param minSuggestionWeight minimum suggestion weight.
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if minimum suggestion weight is greater 
     * or equal than maximum value.
     */
    public void setMinMaxSuggestionWeight(double minSuggestionWeight, 
            double maxSuggestionWeight) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minSuggestionWeight >= maxSuggestionWeight) {
            throw new IllegalArgumentException();
        }
        
        mMinSuggestionWeight = minSuggestionWeight;
        mMaxSuggestionWeight = maxSuggestionWeight;
    }
    
    /**
     * Gets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The 
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     * @return step to increase suggestion weight.
     */
    public double getSuggestionWeightStep() {
        return mSuggestionWeightStep;
    }
    
    /**
     * Sets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The 
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     * @param suggestionWeightStep step to increase suggestion weight.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided step is negative or zero.
     */
    public void setSuggestionWeightStep(double suggestionWeightStep) 
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (suggestionWeightStep <= 0.0) {
            throw new IllegalArgumentException();
        }
        
        mSuggestionWeightStep = suggestionWeightStep;
    }
    
    /**
     * Indicates whether this instance is locked.
     * @return true if this estimator is busy estimating a camera, false 
     * otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Indicates if this estimator is ready to start the estimation.
     * @return true if estimator is ready, false otherwise.
     */
    public abstract boolean isReady();
    
    /**
     * Estimates a pinhole camera.
     * @return estimated pinhole camera.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws PinholeCameraEstimatorException if an error occurs during 
     * estimation, usually because input data is not valid.
     */
    public abstract PinholeCamera estimate() throws LockedException, 
            NotReadyException, PinholeCameraEstimatorException;
    
    /**
     * Returns type of pinhole camera estimator.
     * @return type of pinhole camera estimator.
     */
    public abstract PinholeCameraEstimatorType getType();
    
    /**
     * Creates an instance of a pinhole camera estimator using default type.
     * @return an instance of a pinhole camera estimator.
     */
    public static PinholeCameraEstimator create() {
        return create(DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of a pinhole camera estimator using provided type.
     * @param type type of pinhole camera estimator.
     * @return an instance of a pinhole camera estimator.
     */
    public static PinholeCameraEstimator create(
            PinholeCameraEstimatorType type) {
        switch (type) {
            case DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR:
                return new DLTLinePlaneCorrespondencePinholeCameraEstimator();
            case WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR:
                return new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
            case WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR:
                return new WeightedPointCorrespondencePinholeCameraEstimator();
            case DLT_POINT_PINHOLE_CAMERA_ESTIMATOR:
            default:
                return new DLTPointCorrespondencePinholeCameraEstimator();
        }
    }   
    
    /**
     * Indicates whether suggested parameters are ready to be used.
     * @return true if suggested parameters are ready, false otherwise.
     */
    protected boolean isSuggestionReady() {
        return !((mSuggestPrincipalPointEnabled && mSuggestedPrincipalPointValue == null) ||
                (mSuggestRotationEnabled && mSuggestedRotationValue == null) ||
                (mSuggestCenterEnabled && mSuggestedCenterValue == null) ||
                (mMinSuggestionWeight >= mMaxSuggestionWeight) || 
                (mSuggestionWeightStep <= 0.0));
    }
                       
    /**
     * Attempts to refine provided camera using requested suggestions.
     * If no suggestions are requested or if refinement fails, provided
     * camera is returned instead.
     * @param pinholeCamera camera to be refined.
     * @return refined camera.
     */
    protected abstract PinholeCamera attemptRefine(PinholeCamera pinholeCamera);
    
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
