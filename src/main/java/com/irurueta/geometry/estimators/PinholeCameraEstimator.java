/*
 * Copyright (C) 2013 Alberto Irurueta Carro (alberto@irurueta.com)
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
    protected boolean locked;

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or estimation progress changes.
     */
    protected PinholeCameraEstimatorListener listener;

    /**
     * Indicates whether skewness value is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestSkewnessValueEnabled = DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED;

    /**
     * Suggested skewness value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided
     * value.
     */
    protected double suggestedSkewnessValue = DEFAULT_SUGGESTED_SKEWNESS_VALUE;

    /**
     * Indicates whether horizontal focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestHorizontalFocalLengthEnabled = DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;

    /**
     * Suggested horizontal focal length value to be reached when suggestion is
     * enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected double suggestedHorizontalFocalLengthValue;

    /**
     * Indicates whether vertical focal length is suggested or not. When
     * enabled, the estimator will attempt to enforce suggested value in an
     * iterative manner starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestVerticalFocalLengthEnabled = DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;

    /**
     * Suggested vertical focal length value to be reached when suggestion is
     * enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected double suggestedVerticalFocalLengthValue;

    /**
     * Indicates whether aspect ratio is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestAspectRatioEnabled = DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED;

    /**
     * Suggested aspect ratio value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected double suggestedAspectRatioValue = DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE;

    /**
     * Indicates whether principal point is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestPrincipalPointEnabled = DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED;

    /**
     * Suggested principal point value to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected InhomogeneousPoint2D suggestedPrincipalPointValue;

    /**
     * Indicates whether camera rotation is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestRotationEnabled = DEFAULT_SUGGEST_ROTATION_ENABLED;

    /**
     * Suggested rotation to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected Quaternion suggestedRotationValue;

    /**
     * Indicates whether camera center is suggested or not. When enabled, the
     * estimator will attempt to enforce suggested value in an iterative manner
     * starting from an initially estimated camera.
     * Even when suggestion is enabled, the iterative algorithm might not reach
     * suggested value if the initial value largely differs from the suggested
     * value.
     */
    protected boolean suggestCenterEnabled;

    /**
     * Suggested center to be reached when suggestion is enabled.
     * Suggested value should be close to the initially estimated value
     * otherwise the iterative refinement might not converge to provided value.
     */
    protected InhomogeneousPoint3D suggestedCenterValue;

    /**
     * Minimum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    protected double minSuggestionWeight = DEFAULT_MIN_SUGGESTION_WEIGHT;

    /**
     * Maximum suggestion weight. This weight is used to slowly draw original
     * camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     */
    protected double maxSuggestionWeight = DEFAULT_MAX_SUGGESTION_WEIGHT;

    /**
     * Step to increase suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     */
    protected double suggestionWeightStep = DEFAULT_SUGGESTION_WEIGHT_STEP;

    /**
     * Constructor.
     */
    protected PinholeCameraEstimator() {
        locked = false;
        listener = null;
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     */
    protected PinholeCameraEstimator(final PinholeCameraEstimatorListener listener) {
        locked = false;
        this.listener = listener;
    }

    /**
     * Returns listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     *
     * @return listener to be notified of events.
     */
    public PinholeCameraEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final PinholeCameraEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
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
        this.suggestHorizontalFocalLengthEnabled =
                suggestHorizontalFocalLengthEnabled;
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
     * Gets minimum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @return minimum suggestion weight.
     */
    public double getMinSuggestionWeight() {
        return minSuggestionWeight;
    }

    /**
     * Sets minimum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @param minSuggestionWeight minimum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMinSuggestionWeight(final double minSuggestionWeight) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.minSuggestionWeight = minSuggestionWeight;
    }

    /**
     * Gets maximum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @return maximum suggestion weight.
     */
    public double getMaxSuggestionWeight() {
        return maxSuggestionWeight;
    }

    /**
     * Sets maximum suggestion weight. This weight is used to slowly draw
     * original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException if estimator is locked.
     */
    public void setMaxSuggestionWeight(final double maxSuggestionWeight) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.maxSuggestionWeight = maxSuggestionWeight;
    }

    /**
     * Sets minimum and maximum suggestion weights. Suggestion weight is used to
     * slowly draw original camera parameters into desired suggested values.
     * Suggestion weight slowly increases each time Levenberg-Marquardt is used
     * to find a solution so that the algorithm can converge into desired value.
     * The faster the weights are increased the less likely that suggested
     * values can be converged if they differ too much from the original ones.
     *
     * @param minSuggestionWeight minimum suggestion weight.
     * @param maxSuggestionWeight maximum suggestion weight.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if minimum suggestion weight is greater
     *                                  or equal than maximum value.
     */
    public void setMinMaxSuggestionWeight(final double minSuggestionWeight, final double maxSuggestionWeight)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minSuggestionWeight >= maxSuggestionWeight) {
            throw new IllegalArgumentException();
        }

        this.minSuggestionWeight = minSuggestionWeight;
        this.maxSuggestionWeight = maxSuggestionWeight;
    }

    /**
     * Gets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     *
     * @return step to increase suggestion weight.
     */
    public double getSuggestionWeightStep() {
        return suggestionWeightStep;
    }

    /**
     * Sets step to increase suggestion weight. This weight is used to slowly
     * draw original camera parameters into desired suggested values. Suggestion
     * weight slowly increases each time Levenberg-Marquardt is used to find a
     * solution so that the algorithm can converge into desired value. The
     * faster the weights are increased the less likely that suggested values
     * can be converged if they differ too much from the original ones.
     *
     * @param suggestionWeightStep step to increase suggestion weight.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided step is negative or zero.
     */
    public void setSuggestionWeightStep(final double suggestionWeightStep) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (suggestionWeightStep <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.suggestionWeightStep = suggestionWeightStep;
    }

    /**
     * Indicates whether this instance is locked.
     *
     * @return true if this estimator is busy estimating a camera, false
     * otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Indicates if this estimator is ready to start the estimation.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Estimates a pinhole camera.
     *
     * @return estimated pinhole camera.
     * @throws LockedException                 if estimator is locked.
     * @throws NotReadyException               if input has not yet been provided.
     * @throws PinholeCameraEstimatorException if an error occurs during
     *                                         estimation, usually because input data is not valid.
     */
    public abstract PinholeCamera estimate() throws LockedException, NotReadyException, PinholeCameraEstimatorException;

    /**
     * Returns type of pinhole camera estimator.
     *
     * @return type of pinhole camera estimator.
     */
    public abstract PinholeCameraEstimatorType getType();

    /**
     * Creates an instance of a pinhole camera estimator using default type.
     *
     * @return an instance of a pinhole camera estimator.
     */
    public static PinholeCameraEstimator create() {
        return create(DEFAULT_ESTIMATOR_TYPE);
    }

    /**
     * Creates an instance of a pinhole camera estimator using provided type.
     *
     * @param type type of pinhole camera estimator.
     * @return an instance of a pinhole camera estimator.
     */
    public static PinholeCameraEstimator create(final PinholeCameraEstimatorType type) {
        return switch (type) {
            case DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR -> new DLTLinePlaneCorrespondencePinholeCameraEstimator();
            case WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR ->
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
            case WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR -> new WeightedPointCorrespondencePinholeCameraEstimator();
            default -> new DLTPointCorrespondencePinholeCameraEstimator();
        };
    }

    /**
     * Attempts to refine provided camera using requested suggestions.
     * If no suggestions are requested or if refinement fails, provided
     * camera is returned instead.
     *
     * @param pinholeCamera camera to be refined.
     * @return refined camera.
     */
    protected abstract PinholeCamera attemptRefine(final PinholeCamera pinholeCamera);

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
