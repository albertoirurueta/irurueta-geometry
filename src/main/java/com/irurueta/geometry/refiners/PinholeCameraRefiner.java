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
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Quaternion;
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
 * Typically, a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 *
 * @param <S1> type of matched samples in 1st set.
 * @param <S2> type of matched samples in 2nd set.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class PinholeCameraRefiner<S1, S2> extends PairMatchesAndInliersDataRefiner<PinholeCamera, S1, S2> {

    /**
     * Default value indicating whether skewness value is suggested or not.
     * By default, this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED = false;

    /**
     * Default value of skewness to be suggested when suggestion is enabled.
     * By default suggested skewness is zero.
     */
    public static final double DEFAULT_SUGGESTED_SKEWNESS_VALUE = 0.0;

    /**
     * Default value indicating whether horizontal focal length value is
     * suggested or not. By default, this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED = false;

    /**
     * Default value indicating whether vertical focal length value is suggested
     * or not. By default, this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED = false;

    /**
     * Default value indicating whether aspect ratio is suggested or not. By
     * default, this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED = false;

    /**
     * Default value of aspect ratio to be suggested when suggestion is enabled.
     * By default, suggested aspect ratio is 1.0, although also -1.0 is a typical
     * value when vertical coordinates increase downwards.
     */
    public static final double DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE = 1.0;

    /**
     * Default value indicating whether principal point is suggested or not. By
     * default, this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED = false;

    /**
     * Default value indicating whether rotation is suggested or not. By default,
     * this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_ROTATION_ENABLED = false;

    /**
     * Default value indicating whether center is suggested or not. By default,
     * this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_CENTER_ENABLED = false;

    /**
     * Standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     */
    protected double refinementStandardDeviation;

    /**
     * Indicates whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestSkewnessValueEnabled = DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED;

    /**
     * Suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided
     * value.
     */
    private double suggestedSkewnessValue = DEFAULT_SUGGESTED_SKEWNESS_VALUE;

    /**
     * Indicates whether horizontal focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestHorizontalFocalLengthEnabled = DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;

    /**
     * Suggested horizontal focal length value to be reached when suggestion is
     * enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    private double suggestedHorizontalFocalLengthValue;

    /**
     * Indicates whether vertical focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestVerticalFocalLengthEnabled = DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;

    /**
     * Suggested vertical focal length value to be reached when suggestion is
     * enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    private double suggestedVerticalFocalLengthValue;

    /**
     * Indicates whether aspect ratio is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestAspectRatioEnabled = DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED;

    /**
     * Suggested aspect ratio value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    private double suggestedAspectRatioValue = DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE;

    /**
     * Indicates whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestPrincipalPointEnabled = DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED;

    /**
     * Suggested principal point value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    private InhomogeneousPoint2D suggestedPrincipalPointValue;

    /**
     * Indicates whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestRotationEnabled = DEFAULT_SUGGEST_ROTATION_ENABLED;

    /**
     * Suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    private Quaternion suggestedRotationValue;

    /**
     * Indicates whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    private boolean suggestCenterEnabled;

    /**
     * Suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    private InhomogeneousPoint3D suggestedCenterValue;

    /**
     * Instance to be reused to compute residual for intrinsic parameters.
     */
    private PinholeCameraIntrinsicParameters residualIntrinsic;

    /**
     * Instance to be reused to compute residual on principal point.
     */
    private InhomogeneousPoint2D residualPrincipalPoint;

    /**
     * Instance to be reused to compute residual on rotation.
     */
    private Quaternion residualRotation;

    /**
     * Instance to be reused to compute center.
     */
    private InhomogeneousPoint3D residualCenter;

    /**
     * Constructor.
     */
    protected PinholeCameraRefiner() {
    }

    /**
     * Constructor.
     *
     * @param initialEstimation           initial estimation to be set.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliers                     set indicating which of the provided matches are inliers.
     * @param residuals                   residuals for matched samples.
     * @param numInliers                  number of inliers on initial estimation.
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    protected PinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<S1> samples1, final List<S2> samples2,
            final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers, samples1, samples2);
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param initialEstimation           initial estimation to be set.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliersData                 inlier data, typically obtained from a robust
     *                                    estimator.
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     */
    protected PinholeCameraRefiner(
            final PinholeCamera initialEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<S1> samples1, final List<S2> samples2, final double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, samples2);
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @return standard deviation used for refinement.
     */
    public double getRefinementStandardDeviation() {
        return refinementStandardDeviation;
    }

    /**
     * Sets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @param refinementStandardDeviation standard deviation used for
     *                                    refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementStandardDeviation(final double refinementStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Indicates whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if skewness value is suggested, false otherwise.
     */
    public boolean isSuggestSkewnessValueEnabled() {
        return suggestSkewnessValueEnabled;
    }

    /**
     * Specifies whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestSkewnessValueEnabled true if skewness value is suggested,
     *                                    false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestSkewnessValueEnabled(final boolean suggestSkewnessValueEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestSkewnessValueEnabled = suggestSkewnessValueEnabled;
    }

    /**
     * Gets suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested skewness value.
     */
    public double getSuggestedSkewnessValue() {
        return suggestedSkewnessValue;
    }

    /**
     * Sets suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedSkewnessValue suggested skewness value.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedSkewnessValue(final double suggestedSkewnessValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedSkewnessValue = suggestedSkewnessValue;
    }

    /**
     * Indicates whether horizontal focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if horizontal focal length is suggested, false otherwise.
     */
    public boolean isSuggestHorizontalFocalLengthEnabled() {
        return suggestHorizontalFocalLengthEnabled;
    }

    /**
     * Specifies whether horizontal focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestHorizontalFocalLengthEnabled true if horizontal focal
     *                                            length is suggested, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestHorizontalFocalLengthEnabled(final boolean suggestHorizontalFocalLengthEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestHorizontalFocalLengthEnabled = suggestHorizontalFocalLengthEnabled;
    }

    /**
     * Gets suggested horizontal focal length value to be reached when
     * suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested horizontal focal length value.
     */
    public double getSuggestedHorizontalFocalLengthValue() {
        return suggestedHorizontalFocalLengthValue;
    }

    /**
     * Sets suggested horizontal focal length value to be reached when
     * suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedHorizontalFocalLengthValue suggested horizontal focal
     *                                            length value.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedHorizontalFocalLengthValue(final double suggestedHorizontalFocalLengthValue)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedHorizontalFocalLengthValue = suggestedHorizontalFocalLengthValue;
    }

    /**
     * Indicates whether vertical focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if vertical focal length is suggested, false otherwise.
     */
    public boolean isSuggestVerticalFocalLengthEnabled() {
        return suggestVerticalFocalLengthEnabled;
    }

    /**
     * Specifies whether vertical focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestVerticalFocalLengthEnabled true if vertical focal length is
     *                                          suggested, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestVerticalFocalLengthEnabled(final boolean suggestVerticalFocalLengthEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestVerticalFocalLengthEnabled = suggestVerticalFocalLengthEnabled;
    }

    /**
     * Gets suggested vertical focal length value to be reached when suggestion
     * is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested vertical focal length.
     */
    public double getSuggestedVerticalFocalLengthValue() {
        return suggestedVerticalFocalLengthValue;
    }

    /**
     * Sets suggested vertical focal length value to be reached when suggestion
     * is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedVerticalFocalLengthValue suggested vertical focal length.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedVerticalFocalLengthValue(final double suggestedVerticalFocalLengthValue)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedVerticalFocalLengthValue = suggestedVerticalFocalLengthValue;
    }

    /**
     * Indicates whether aspect ratio is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if aspect ratio is suggested, false otherwise.
     */
    public boolean isSuggestAspectRatioEnabled() {
        return suggestAspectRatioEnabled;
    }

    /**
     * Specifies whether aspect ratio is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestAspectRatioEnabled true if aspect ratio is suggested, false
     *                                  otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestAspectRatioEnabled(final boolean suggestAspectRatioEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestAspectRatioEnabled = suggestAspectRatioEnabled;
    }

    /**
     * Gets suggested aspect ratio value to be reached when suggestion is
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested aspect ratio value.
     */
    public double getSuggestedAspectRatioValue() {
        return suggestedAspectRatioValue;
    }

    /**
     * Sets suggested aspect ratio value to be reached when suggestion is
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedAspectRatioValue suggested aspect ratio value.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedAspectRatioValue(final double suggestedAspectRatioValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedAspectRatioValue = suggestedAspectRatioValue;
    }

    /**
     * Indicates whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if principal point is suggested, false otherwise.
     */
    public boolean isSuggestPrincipalPointEnabled() {
        return suggestPrincipalPointEnabled;
    }

    /**
     * Specifies whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestPrincipalPointEnabled true if principal point is suggested,
     *                                     false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestPrincipalPointEnabled(final boolean suggestPrincipalPointEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestPrincipalPointEnabled = suggestPrincipalPointEnabled;
        if (suggestPrincipalPointEnabled && suggestedPrincipalPointValue == null) {
            suggestedPrincipalPointValue = new InhomogeneousPoint2D();
        }
    }

    /**
     * Gets suggested principal point value to be reached when suggestion is
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested principal point value to be reached when suggestion is
     * enabled.
     */
    public InhomogeneousPoint2D getSuggestedPrincipalPointValue() {
        return suggestedPrincipalPointValue;
    }

    /**
     * Sets suggested principal point value to be reached when suggestion is
     * enabled. Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedPrincipalPointValue suggested principal point value to be
     *                                     reached when suggestion is enabled.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedPrincipalPointValue(final InhomogeneousPoint2D suggestedPrincipalPointValue)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedPrincipalPointValue = suggestedPrincipalPointValue;
    }

    /**
     * Indicates whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if camera rotation is suggested, false otherwise.
     */
    public boolean isSuggestRotationEnabled() {
        return suggestRotationEnabled;
    }

    /**
     * Specifies whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestRotationEnabled true if camera rotation is suggested, false
     *                               otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestRotationEnabled(final boolean suggestRotationEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestRotationEnabled = suggestRotationEnabled;
        if (suggestRotationEnabled && suggestedRotationValue == null) {
            suggestedRotationValue = new Quaternion();
        }

    }

    /**
     * Gets suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested rotation to be reached when suggestion is enabled.
     */
    public Quaternion getSuggestedRotationValue() {
        return suggestedRotationValue;
    }

    /**
     * Sets suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedRotationValue suggested rotation to be reached when
     *                               suggestion is enabled.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedRotationValue(final Quaternion suggestedRotationValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedRotationValue = suggestedRotationValue;
    }

    /**
     * Indicates whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @return true if camera center is suggested, false otherwise.
     */
    public boolean isSuggestCenterEnabled() {
        return suggestCenterEnabled;
    }

    /**
     * Specifies whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     *
     * @param suggestCenterEnabled true if camera is suggested, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestCenterEnabled(final boolean suggestCenterEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestCenterEnabled = suggestCenterEnabled;
        if (suggestCenterEnabled && suggestedCenterValue == null) {
            suggestedCenterValue = new InhomogeneousPoint3D();
        }
    }

    /**
     * Gets suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @return suggested center to be reached when suggestion is enabled.
     */
    public InhomogeneousPoint3D getSuggestedCenterValue() {
        return suggestedCenterValue;
    }

    /**
     * Sets suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     *
     * @param suggestedCenterValue suggested center to be reached when
     *                             suggestion is enabled.
     * @throws LockedException if estimator is locked.
     */
    public void setSuggestedCenterValue(final InhomogeneousPoint3D suggestedCenterValue) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.suggestedCenterValue = suggestedCenterValue;
    }

    /**
     * Refines provided initial estimation.
     *
     * @return refines estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    @Override
    public PinholeCamera refine() throws NotReadyException, LockedException, RefinerException {
        final var result = new PinholeCamera();
        refine(result);
        return result;
    }

    /**
     * Residual term for any required suggestions.
     *
     * @param params parameters being optimized. In the following order:
     *               skewness, horizontal focal length, vertical focal length,
     *               horizontal principal point, vertical principal point, quaternion A,
     *               quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param weight weight to apply to obtained residual. This weight increases
     *               on each iteration to help into achieving required suggested values.
     * @return term for any required suggestion.
     */
    protected double suggestionResidual(final double[] params, final double weight) {
        double residual = 0.0;

        if (suggestSkewnessValueEnabled) {
            residual += Math.pow(params[0] - suggestedSkewnessValue, 2.0);
        }

        if (suggestHorizontalFocalLengthEnabled) {
            residual += Math.pow(params[1] - suggestedHorizontalFocalLengthValue, 2.0);
        }

        if (suggestVerticalFocalLengthEnabled) {
            residual += Math.pow(params[2] - suggestedVerticalFocalLengthValue, 2.0);
        }

        if (suggestAspectRatioEnabled) {
            final var aspectRatio = params[2] / params[1];
            residual += Math.pow(aspectRatio - suggestedAspectRatioValue, 2.0);
        }

        if (suggestPrincipalPointEnabled) {
            if (residualPrincipalPoint == null) {
                residualPrincipalPoint = new InhomogeneousPoint2D(params[3], params[4]);
            } else {
                residualPrincipalPoint.setInhomogeneousCoordinates(params[3], params[4]);
            }

            residual += Math.pow(residualPrincipalPoint.distanceTo(suggestedPrincipalPointValue), 2.0);
        }

        if (suggestRotationEnabled) {
            if (residualRotation == null) {
                residualRotation = new Quaternion(params[5], params[6], params[7], params[8]);
            } else {
                residualRotation.setA(params[5]);
                residualRotation.setB(params[6]);
                residualRotation.setC(params[7]);
                residualRotation.setD(params[8]);
            }
            residualRotation.normalize();
            suggestedRotationValue.normalize();
            residual += Math.pow(residualRotation.getA() - suggestedRotationValue.getA(), 2.0)
                    + Math.pow(residualRotation.getB() - suggestedRotationValue.getB(), 2.0)
                    + Math.pow(residualRotation.getC() - suggestedRotationValue.getC(), 2.0)
                    + Math.pow(residualRotation.getD() - suggestedRotationValue.getD(), 2.0);
        }

        if (suggestCenterEnabled) {
            if (residualCenter == null) {
                residualCenter = new InhomogeneousPoint3D(params[9], params[10], params[11]);
            } else {
                residualCenter.setInhomogeneousCoordinates(params[9], params[10], params[11]);
            }
            residual += Math.pow(residualCenter.distanceTo(suggestedCenterValue), 2.0);
        }

        return weight * residual;
    }

    /**
     * Sets array of parameters into a pinhole camera.
     * This method is used internally during refinement.
     *
     * @param params parameters to be set. In the following order:
     *               skewness, horizontal focal length, vertical focal length,
     *               horizontal principal point, vertical principal point, quaternion A,
     *               quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @param result instance where parameters will be set.
     */
    protected void parametersToCamera(final double[] params, final PinholeCamera result) {

        if (residualIntrinsic == null) {
            residualIntrinsic = new PinholeCameraIntrinsicParameters();
        }
        residualIntrinsic.setSkewness(params[0]);
        residualIntrinsic.setHorizontalFocalLength(params[1]);
        residualIntrinsic.setVerticalFocalLength(params[2]);
        residualIntrinsic.setHorizontalPrincipalPoint(params[3]);
        residualIntrinsic.setVerticalPrincipalPoint(params[4]);

        if (residualRotation == null) {
            residualRotation = new Quaternion(params[5], params[6], params[7], params[8]);
        } else {
            residualRotation.setA(params[5]);
            residualRotation.setB(params[6]);
            residualRotation.setC(params[7]);
            residualRotation.setD(params[8]);
        }
        residualRotation.normalize();

        if (residualCenter == null) {
            residualCenter = new InhomogeneousPoint3D(params[9], params[10], params[11]);
        } else {
            residualCenter.setInhomogeneousCoordinates(params[9], params[10], params[11]);
        }
        residualCenter.normalize();

        result.setIntrinsicAndExtrinsicParameters(residualIntrinsic, residualRotation, residualCenter);
        result.normalize();
    }

    /**
     * Sets camera parameters into array of parameters.
     * This method is used internally during refinement.
     *
     * @param camera camera to obtain parameters to be set into array.
     * @param result array where extracted parameters are stored. In the
     *               following order:
     *               skewness, horizontal focal length, vertical focal length,
     *               horizontal principal point, vertical principal point, quaternion A,
     *               quaternion B, quaternion C, quaternion D, center x, center y, center z.
     * @throws CameraException       if camera cannot be decomposed.
     * @throws NotAvailableException if any camera component cannot be
     *                               retrieved.
     */
    protected void cameraToParameters(final PinholeCamera camera, final double[] result) throws CameraException,
            NotAvailableException {

        camera.decompose();

        final var intrinsic = camera.getIntrinsicParameters();
        result[0] = intrinsic.getSkewness();
        result[1] = intrinsic.getHorizontalFocalLength();
        result[2] = intrinsic.getVerticalFocalLength();
        result[3] = intrinsic.getHorizontalPrincipalPoint();
        result[4] = intrinsic.getVerticalPrincipalPoint();

        final var rotation = camera.getCameraRotation();
        if (residualRotation == null) {
            residualRotation = rotation.toQuaternion();
        } else {
            rotation.toQuaternion(residualRotation);
        }
        residualRotation.normalize();

        result[5] = residualRotation.getA();
        result[6] = residualRotation.getB();
        result[7] = residualRotation.getC();
        result[8] = residualRotation.getD();

        final var center = camera.getCameraCenter();

        result[9] = center.getInhomX();
        result[10] = center.getInhomY();
        result[11] = center.getInhomZ();
    }

    /**
     * Indicates whether obtained solution requires refinement to apply provided
     * suggestions.
     *
     * @return true if solution requires refinement to apply provided
     * suggestions, false otherwise.
     */
    protected boolean hasSuggestions() {
        return hasIntrinsicSuggestions() || hasExtrinsicSuggestions();
    }

    /**
     * Indicates whether suggestions for any intrinsic parameter are required
     * or not.
     *
     * @return true if suggestions for any intrinsic parameters are required,
     * false otherwise.
     */
    private boolean hasIntrinsicSuggestions() {
        return suggestSkewnessValueEnabled || suggestHorizontalFocalLengthEnabled || suggestVerticalFocalLengthEnabled
                || suggestAspectRatioEnabled;
    }

    /**
     * Indicates whether suggestions for any extrinsic parameter are required
     * or not.
     *
     * @return true if suggestions for any extrinsic parameter are required,
     * false otherwise.
     */
    private boolean hasExtrinsicSuggestions() {
        return suggestPrincipalPointEnabled || suggestRotationEnabled || suggestCenterEnabled;
    }
}
