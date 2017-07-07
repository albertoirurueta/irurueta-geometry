/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.PinholeCameraRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 29, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.InliersData;
import java.util.BitSet;
import java.util.List;

/**
 * Base class for pinhole camera refiners.
 * Implementations of this class refine a pinhole camera by taking into account 
 * an initial estimation, inlier point or line matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers in
 * LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be 
 * useful in some other situations.
 * @param <S1> type of matched samples in 1st set.
 * @param <S2> type of matched samples in 2nd set.
 */
public abstract class PinholeCameraRefiner<S1, S2> extends 
        PairMatchesAndInliersDataRefiner<PinholeCamera, S1, S2> {
    
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
     * Standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust 
     * estimation, since residuals of found inliers are within the range of 
     * such threshold.
     */
    protected double mRefinementStandardDeviation;    

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
    public PinholeCameraRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     */
    public PinholeCameraRefiner(PinholeCamera initialEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals,
            int numInliers, List<S1> samples1, List<S2> samples2, 
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers,
                samples1, samples2);
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     */
    public PinholeCameraRefiner(PinholeCamera initialEstimation,
            boolean keepCovariance, InliersData inliersData, List<S1> samples1,
            List<S2> samples2, double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2);
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @return standard deviation used for refinement.
     */
    public double getRefinementStandardDeviation() {
        return mRefinementStandardDeviation;
    }
    
    /**
     * Sets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @param refinementStandardDeviation standard deviation used for 
     * refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementStandardDeviation(
            double refinementStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefinementStandardDeviation = refinementStandardDeviation;
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
     * Refines provided initial estimation.
     * @return refines estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */
    @Override
    public PinholeCamera refine() throws NotReadyException, LockedException, 
            RefinerException {
        PinholeCamera result = new PinholeCamera();
        refine(result);
        return result;
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
     * @throws CameraException if camera cannot be decomposed.
     * @throws NotAvailableException if any camera component cannot be 
     * retrieved.
     */
    protected void parametersToCamera(double[] params, 
            PinholeCamera result) throws CameraException, 
            NotAvailableException {
        
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
