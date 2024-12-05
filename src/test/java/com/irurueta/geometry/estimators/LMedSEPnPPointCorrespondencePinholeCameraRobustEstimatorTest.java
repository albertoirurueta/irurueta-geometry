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

import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class LMedSEPnPPointCorrespondencePinholeCameraRobustEstimatorTest implements PinholeCameraRobustEstimatorListener {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final double MIN_RANDOM_VALUE = 50.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 90.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = 10.0;
    private static final double MAX_ANGLE_DEGREES = 15.0;

    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 10;

    private static final double THRESHOLD = 1e-6;

    private static final double OUTLIER_STD_ERROR = 100.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstants() {
        assertEquals(1.0, LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(0.0, LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.MIN_STOP_THRESHOLD,
                0.0);
    }

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertNull(estimator.getIntrinsic());

        // test constructor with listener
        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertNull(estimator.getIntrinsic());

        // test constructor with points
        final var points3D = new ArrayList<Point3D>();
        final var points2D = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(points3D, points2D);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        final var points3DEmpty = new ArrayList<Point3D>();
        final var points2DEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                points3DEmpty, points2DEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                points3D, points2DEmpty));

        // test constructor with listener and points
        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, points3D, points2D);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertNull(estimator.getIntrinsic());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                this, points3DEmpty, points2DEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                this, points3D, points2DEmpty));

        // test constructor with intrinsic and listener
        final var intrinsic = new PinholeCameraIntrinsicParameters();

        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(intrinsic);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertSame(estimator.getIntrinsic(), intrinsic);

        // test constructor with listener and intrinsic
        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertSame(intrinsic, estimator.getIntrinsic());

        // test constructor with points and intrinsic
        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(intrinsic, points3D, points2D);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertSame(intrinsic, estimator.getIntrinsic());

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                intrinsic, points3DEmpty, points2DEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                intrinsic, points3D, points2DEmpty));

        // test constructor with listener, points and intrinsic
        estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic, points3D,
                points2D);

        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertSame(points2D, estimator.getPoints2D());
        assertSame(points3D, estimator.getPoints3D());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertSame(estimator.getIntrinsic(), intrinsic);

        // Force IllegalArgumentException
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                this, intrinsic, points3DEmpty, points2DEmpty));
        //different sizes
        assertThrows(IllegalArgumentException.class, () -> new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(
                this, intrinsic, points3D, points2DEmpty));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(!PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(!PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
    }

    @Test
    void testIsSetFastRefinementUsed() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());

        // set new value
        estimator.setFastRefinementUsed(!PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT, estimator.isFastRefinementUsed());
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());

        // set new value
        final var points3D = new ArrayList<Point3D>();
        final var points2D = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator.setPoints(points3D, points2D);

        // check correctness
        assertSame(points3D, estimator.getPoints3D());
        assertSame(points2D, estimator.getPoints2D());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var points3DEmpty = new ArrayList<Point3D>();
        final var points2DEmpty = new ArrayList<Point2D>();
        // not enough points
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(points3DEmpty, points2DEmpty));
        // different sizes
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(points3D, points2DEmpty));
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());

        // set new value
        estimator.setSuggestSkewnessValueEnabled(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
    }

    @Test
    void testGetSetSuggestedSkewnessValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);

        // set new value
        estimator.setSuggestedSkewnessValue(-1.0);

        // check correctness
        assertEquals(estimator.getSuggestedSkewnessValue(), -1.0, 0.0);
    }

    @Test
    void testIsSetSuggestedHorizontalFocalLengthEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());

        // set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
    }

    @Test
    void testGetSetSuggestedHorizontalFocalLengthValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestVerticalFocalLengthEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());

        // set new value
        estimator.setSuggestVerticalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
    }

    @Test
    void testGetSetSuggestedVerticalFocalLengthValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);

        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());

        // set new value
        estimator.setSuggestAspectRatioEnabled(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
    }

    @Test
    void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);

        // set new value
        estimator.setSuggestedAspectRatioValue(-1.0);

        // check correctness
        assertEquals(-1.0, estimator.getSuggestedAspectRatioValue(), 0.0);
    }

    @Test
    void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());

        // set new value
        estimator.setSuggestPrincipalPointEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
    }

    @Test
    void testGetSetSuggestedPrincipalPointValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getSuggestedPrincipalPointValue());

        // set new value
        final var principalPoint = new InhomogeneousPoint2D();
        estimator.setSuggestedPrincipalPointValue(principalPoint);

        // check correctness
        assertSame(principalPoint, estimator.getSuggestedPrincipalPointValue());
    }

    @Test
    void testIsSetSuggestRotationEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());

        // set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
    }

    @Test
    void testGetSetSuggestedRotationValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getSuggestedRotationValue());

        // set new value
        final var q = new Quaternion();
        estimator.setSuggestedRotationValue(q);

        // check correctness
        assertSame(q, estimator.getSuggestedRotationValue());
    }

    @Test
    void testIsSetSuggestCenterEnabled() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED,
                estimator.isSuggestCenterEnabled());

        // set new value
        estimator.setSuggestCenterEnabled(
                !LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(!LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED,
                estimator.isSuggestCenterEnabled());
    }

    @Test
    void testGetSetSuggestedCenterValue() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getSuggestedCenterValue());

        // set new value
        final var center = new InhomogeneousPoint3D();
        estimator.setSuggestedCenterValue(center);

        // check correctness
        assertSame(center, estimator.getSuggestedCenterValue());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testIsNormalizeSubsetPointCorrespondences() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());

        // set new value
        estimator.setNormalizeSubsetPointCorrespondences(true);

        // check correctness
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
    }

    @Test
    void testGetSetIntrinsic() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getIntrinsic());
        assertFalse(estimator.isReady());

        // set new value
        final var intrinsic = new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertSame(intrinsic, estimator.getIntrinsic());
        assertFalse(estimator.isReady());
    }

    @Test
    void testIsReady() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertNull(estimator.getIntrinsic());

        // set new value
        final var points3D = new ArrayList<Point3D>();
        final var points2D = new ArrayList<Point2D>();
        for (var i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator.setPoints(points3D, points2D);

        final var intrinsic = new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertSame(points3D, estimator.getPoints3D());
        assertSame(points2D, estimator.getPoints2D());
        assertSame(intrinsic, estimator.getIntrinsic());
        assertTrue(estimator.isReady());
    }

    @Test
    void testIsSetPlanarConfigurationAllowed() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());

        // set new value
        estimator.setPlanarConfigurationAllowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED);

        // check correctness
        assertEquals(!EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
    }

    @Test
    void testIsSetNullspaceDimension2Allowed() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());

        // set new value
        estimator.setNullspaceDimension2Allowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);

        // check correctness
        assertEquals(!EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED,
                estimator.isNullspaceDimension2Allowed());
    }

    @Test
    void testIsSetNullspaceDimension3Allowed() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());

        // set new value
        estimator.setNullspaceDimension3Allowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);

        // check correctness
        assertEquals(!EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION3_ALLOWED,
                estimator.isNullspaceDimension3Allowed());
    }

    @Test
    void testGetSetPlanarThreshold() throws LockedException {
        final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);

        // set new value
        estimator.setPlanarThreshold(1e9);

        // check correctness
        assertEquals(1e9, estimator.getPlanarThreshold(), 0.0);
    }

    @Test
    void testEstimateGeneralNoSuggestion() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValidCameras = 0;
        var numValidProjections = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();

            final PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = camera2.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var point3D = points3D.get(i);
                final var originalPoint2D = points2D.get(i);
                final var estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, originalPoint2D.distanceTo(estimatedPoint2D), ABSOLUTE_ERROR);
            }

            if (isValid) {
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
    }

    @Test
    void testEstimateGeneralNoSuggestionWithRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValidCameras = 0;
        var numValidProjections = 0;
        var numCovariances = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();

            final PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = camera2.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var point3D = points3D.get(i);
                final var originalPoint2D = points2D.get(i);
                final var estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, originalPoint2D.distanceTo(estimatedPoint2D), LARGE_ABSOLUTE_ERROR);
            }

            if (isValid) {
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0 && numCovariances > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }

    @Test
    void testEstimateGeneralNoSuggestionWithFastRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValidCameras = 0;
        var numValidProjections = 0;
        var numCovariances = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();

            final PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = camera2.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var point3D = points3D.get(i);
                final var originalPoint2D = points2D.get(i);
                final var estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, originalPoint2D.distanceTo(estimatedPoint2D), LARGE_ABSOLUTE_ERROR);
            }

            if (isValid) {
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0 && numCovariances > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }

    @Test
    void testEstimateGeneralSuggestedRotationEnabled() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedRotationNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedRotationNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotationNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotationNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotationNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralSuggestedRotationWithRefinement() throws LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedRotationNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedRotationNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotationNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotationNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotationNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralSuggestedRotationWithFastRefinement() throws LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < 2 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedRotationNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedRotationNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotationNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotationNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotationNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralSuggestedCenterEnabled() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >= cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralSuggestedCenterWithRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >= cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralSuggestedCenterWithFastRefinement() throws LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >= cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarNoSuggestion() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValidCameras = 0;
        var numValidProjections = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();

            final PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = camera2.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var point3D = points3D.get(i);
                final var originalPoint2D = points2D.get(i);
                final var estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, originalPoint2D.distanceTo(estimatedPoint2D), ABSOLUTE_ERROR);
            }

            if (isValid) {
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
    }

    @Test
    void testEstimatePlanarNoSuggestionWithRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValidCameras = 0;
        var numValidProjections = 0;
        var numCovariances = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();

            final PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = camera2.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var point3D = points3D.get(i);
                final var originalPoint2D = points2D.get(i);
                final var estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, originalPoint2D.distanceTo(estimatedPoint2D), LARGE_ABSOLUTE_ERROR);
            }

            if (isValid) {
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0 && numCovariances > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }

    @Test
    void testEstimatePlanarNoSuggestionWithFastRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValidCameras = 0;
        var numValidProjections = 0;
        var numCovariances = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            reset();

            final PinholeCamera camera2;
            try {
                camera2 = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = camera2.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                    10 * LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(),
                    10 * LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    10 * LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(),
                    10 * LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 10 * LARGE_ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = camera2.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            var isValid = true;
            for (var i = 0; i < nPoints; i++) {
                final var point3D = points3D.get(i);
                final var originalPoint2D = points2D.get(i);
                final var estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) > LARGE_ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
                assertEquals(0.0, originalPoint2D.distanceTo(estimatedPoint2D),
                        LARGE_ABSOLUTE_ERROR);
            }

            if (isValid) {
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0 && numCovariances > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }

    @Test
    void testEstimatePlanarSuggestedRotationEnabled() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            //create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedRotationNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedRotationNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotationNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotationNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotationNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarSuggestedRotationWithRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedRotationNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedRotationNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotationNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotationNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotationNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarSuggestedRotationWithFastRefinement() throws LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedRotationNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedRotationNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotationNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotationNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotationNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

            if (diffEstimatedNoSuggestion >= diffEstimated) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarSuggestedCenterEnabled() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >= cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarSuggestedCenterWithRefinement() throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >= cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numValid > 0 && numCovariances > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarSuggestedCenterWithFastRefinement() throws LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numCovariances = 0;
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final var cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final var principalAxis = camera.getPrincipalAxisArray();
            final var depth = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final var plane = new Plane(centroid, principalAxis);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var a = plane.getA();
            final var b = plane.getB();
            final var c = plane.getC();
            final var d = plane.getD();

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                final var point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // create outliers
            final var points2DWithError = new ArrayList<Point2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, OUTLIER_STD_ERROR);
            for (final var point2D : points2D) {
                Point2D point2DWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var errorW = errorRandomizer.nextDouble();
                    point2DWithError = new HomogeneousPoint2D(
                            point2D.getHomX() + errorX,
                            point2D.getHomY() + errorY,
                            point2D.getHomW() + errorW);
                } else {
                    // inlier point (without error)
                    point2DWithError = point2D;
                }

                points2DWithError.add(point2DWithError);
            }

            final var estimator = new LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator(this, intrinsic,
                    points3D, points2DWithError);

            estimator.setStopThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >= cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final PinholeCameraRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final PinholeCameraRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final PinholeCameraRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator estimator) {
        final var points3D = new ArrayList<Point3D>();
        final var points2D = new ArrayList<Point2D>();
        assertThrows(LockedException.class, () -> estimator.setPoints(points3D, points2D));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.01f));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setNormalizeSubsetPointCorrespondences(true));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.setSuggestSkewnessValueEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedSkewnessValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestHorizontalFocalLengthEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedHorizontalFocalLengthValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestVerticalFocalLengthEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedVerticalFocalLengthValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestAspectRatioEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedAspectRatioValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestPrincipalPointEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedPrincipalPointValue(null));
        assertThrows(LockedException.class, () -> estimator.setSuggestRotationEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedRotationValue(null));
        assertThrows(LockedException.class, () -> estimator.setSuggestCenterEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedCenterValue(null));
        assertThrows(LockedException.class, () -> estimator.setFastRefinementUsed(true));
        assertThrows(LockedException.class, () -> estimator.setPlanarConfigurationAllowed(true));
        assertThrows(LockedException.class, () -> estimator.setNullspaceDimension2Allowed(true));
        assertThrows(LockedException.class, () -> estimator.setNullspaceDimension3Allowed(true));
        assertThrows(LockedException.class, () -> estimator.setPlanarThreshold(1e9));
        assertThrows(LockedException.class, () -> estimator.setIntrinsic(null));
        assertTrue(estimator.isLocked());
    }
}
