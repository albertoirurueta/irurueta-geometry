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
 * their corresponding projected 2D lines (depending on point
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
     * estimation progress. By default, this is set to 5%.
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
     * accurate because chosen sub-samples will be inliers.
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
     * Default value indicating whether center is suggested or not. By default
     * this is disabled.
     */
    public static final boolean DEFAULT_SUGGEST_CENTER_ENABLED = false;

    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected PinholeCameraRobustEstimatorListener listener;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected volatile boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float progressDelta;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double confidence;

    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int maxIterations;

    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData inliersData;

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     */
    protected boolean refineResult;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    protected boolean keepCovariance;

    /**
     * Indicates whether fast refinement must be used or not.
     * When true Levenberg/Marquardt refinement will be used, when false
     * an initial Powell optimization is done and then Levenberg/Marquard is
     * used for covariance estimation if needed.
     */
    protected boolean useFastRefinement;

    /**
     * Estimated covariance of estimated fundamental matrix.
     * This is only available when result has been refined and covariance is
     * kept.
     */
    protected Matrix covariance;

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
     * Constructor.
     */
    protected PinholeCameraRobustEstimator() {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected PinholeCameraRobustEstimator(final PinholeCameraRobustEstimatorListener listener) {
        this.listener = listener;
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public PinholeCameraRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final PinholeCameraRobustEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return listener != null;
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
     * Indicates if this instance is locked because estimation is being
     * computed.
     *
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @param progressDelta amount of progress variation before notifying a
     *                      progress change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     *                                  greater than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return confidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     * @throws LockedException          if this estimator is locked because an estimator
     *                                  is being computed.
     */
    public void setConfidence(final double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.confidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling
     * estimate(), a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return maxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.maxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return inliersData;
    }

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     *
     * @return true to refine result, false to simply use result found by
     * robust estimator without further refining.
     */
    public boolean isResultRefined() {
        return refineResult;
    }

    /**
     * Specifies whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result
     *                     found by robust estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false
     * otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining
     *                       result, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }

    /**
     * Indicates whether fast refinement must be used or not.
     * When true Levenberg/Marquardt refinement will be used, when false
     * an initial Powell optimization is done and then Levenberg/Marquard is
     * used for covariance estimation if needed.
     * Fast refinement requires less computing time, but it is more likely to
     * fail than slow one.
     *
     * @return true to use fast refinement, false to use a slow but more
     * accurate and stable refinement.
     */
    public boolean isFastRefinementUsed() {
        return useFastRefinement;
    }

    /**
     * Specifies whether fast refinement must be used or not.
     * When true Levenberg/Marquardt refinement will be used, when false
     * an initial Powell optimization is done and then Levenberg/Marquard is
     * used for covariance estimation if needed.
     * Fast refinement requires less computing time, but it is more likely to
     * fail than slow one.
     *
     * @param useFastRefinement true to use fast refinement, false to use a slow
     *                          but more accurate and stable refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setFastRefinementUsed(final boolean useFastRefinement) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useFastRefinement = useFastRefinement;
    }

    /**
     * Gets estimated covariance of estimated pinhole camera if available.
     * This is only available when result has been refined and covariance is
     * kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Estimates a pinhole camera using a robust estimator and
     * the best set of matched 2D/3D point correspondences or 2D line/3D plane
     * correspondences found using the robust estimator.
     *
     * @return a pinhole camera.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract PinholeCamera estimate() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

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
    protected abstract double getRefinementStandardDeviation();

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided robust estimator method.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided quality scores and robust estimator
     * method.
     *
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final List<Point3D> points3D, List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D, qualityScores,
                method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using default robust estimator method.
     *
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and default robust estimator
     * method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     *                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided quality scores and default robust
     * estimator method.
     *
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(points3D, points2D, qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, points3D, points2D, qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided robust estimator method.
     *
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @param method    method of a robust estimator algorithm to estimate the best
     *                  pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and robust estimator method.
     *
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @param method    method of a robust estimator algorithm to estimate the best
     *                  pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided quality scores and robust estimator
     * method.
     *
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, points3D, points2D, qualityScores,
                method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, points3D, points2D,
                qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using default robust estimator method.
     *
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and default robust estimator
     * method.
     *
     * @param listener  listener to be notified of events such as when estimation
     *                  starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D  list of 3D points used to estimate a pinhole camera.
     * @param points2D  list of corresponding projected 2D points used to
     *                  estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided quality scores and default robust
     * estimator method.
     *
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraIntrinsicParameters intrinsic, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(intrinsic, points3D, points2D, qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param intrinsic     intrinsic parameters of camera to be estimated.
     * @param points3D      list of 3D points used to estimate a pinhole camera.
     * @param points2D      list of corresponding projected 2D points used to
     *                      estimate a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final PinholeCameraIntrinsicParameters intrinsic,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, intrinsic, points3D, points2D,
                qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided robust estimator method.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and robust estimator method.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided quality scores and robust estimator
     * method.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @param method                   method of a robust estimator algorithm to estimate the best
     *                                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores, final RobustEstimatorMethod method) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using default robust estimator method.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener and default robust estimator
     * method.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than required minimum size
     *                                  (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided quality scores and default robust
     * estimator method.
     *
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final double skewness, final double horizontalPrincipalPoint, final double verticalPrincipalPoint,
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D, qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 2D/3D point
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     *
     * @param listener                 listener to be notified of events such as when estimation
     *                                 starts, ends or its progress significantly changes.
     * @param skewness                 skewness value of intrinsic parameters of camera to be
     *                                 estimated.
     * @param horizontalPrincipalPoint horizontal principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param verticalPrincipalPoint   vertical principal point value of
     *                                 intrinsic parameters of camera to be estimated.
     * @param points3D                 list of 3D points used to estimate a pinhole camera.
     * @param points2D                 list of corresponding projected 2D points used to
     *                                 estimate a pinhole camera.
     * @param qualityScores            quality scores corresponding to each pair of matched
     *                                 points.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of points and quality
     *                                  scores don't have the same size  or their size is smaller than required
     *                                  minimum size (6 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPoints(
            final PinholeCameraRobustEstimatorListener listener, final double skewness,
            final double horizontalPrincipalPoint, final double verticalPrincipalPoint, final List<Point3D> points3D,
            final List<Point2D> points2D, final double[] qualityScores) {
        return PointCorrespondencePinholeCameraRobustEstimator.create(listener, skewness, horizontalPrincipalPoint,
                verticalPrincipalPoint, points3D, points2D, qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided robust estimator method.
     *
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines  list of corresponding projected 2D lines used to estimate
     *               a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate the best
     *               pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     *                                  have the same size or their size is smaller than required minimum size (4
     *                                  correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final List<Plane> planes, final List<Line2D> lines, final RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener and robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   list of 3D planes used to estimate a pinhole camera.
     * @param lines    list of corresponding projected 2D lines used to estimate
     *                 a pinhole camera.
     * @param method   method of a robust estimator algorithm to estimate the best
     *                 pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     *                                  have the same size or their size is smaller than required minimum size (4
     *                                  correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes, final List<Line2D> lines,
            final RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided quality scores and robust estimator
     * method.
     *
     * @param planes        list of 3D planes used to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      samples.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final List<Plane> planes, final List<Line2D> lines, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores, method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener, quality scores and robust
     * estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        list of 3D planes used to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      samples.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes, final List<Line2D> lines,
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines, qualityScores,
                method);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using default robust estimator method.
     *
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines  list of corresponding projected 2D lines used to estimate
     *               a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     *                                  have the same size or their size is smaller than required minimum size (4
     *                                  correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final List<Plane> planes, final List<Line2D> lines) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener and default robust estimator
     * method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param planes   list of 3D planes used to estimate a pinhole camera.
     * @param lines    list of corresponding projected 2D lines used to estimate
     *                 a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines don't
     *                                  have the same size or their size is smaller than required minimum size (4
     *                                  correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes, final List<Line2D> lines) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided quality scores and default robust
     * estimator method.
     *
     * @param planes        list of 3D planes used to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      samples.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final List<Plane> planes, final List<Line2D> lines, final double[] qualityScores) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(planes, lines, qualityScores);
    }

    /**
     * Creates a pinhole camera robust estimator based on 3D plane/2D line
     * correspondences and using provided listener, quality scores and default
     * robust estimator method.
     *
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param planes        list of 3D planes used to estimate a pinhole camera.
     * @param lines         list of corresponding projected 2D lines used to estimate
     *                      a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      samples.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes/lines and
     *                                  quality scores don't have the same size or their size is smaller than
     *                                  required minimum size (4 correspondences).
     */
    public static PinholeCameraRobustEstimator createFromPlanesAndLines(
            final PinholeCameraRobustEstimatorListener listener, final List<Plane> planes,
            final List<Line2D> lines, final double[] qualityScores) {
        return LinePlaneCorrespondencePinholeCameraRobustEstimator.create(listener, planes, lines, qualityScores);
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
