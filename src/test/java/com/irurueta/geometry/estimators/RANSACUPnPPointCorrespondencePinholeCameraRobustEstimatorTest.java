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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RANSACUPnPPointCorrespondencePinholeCameraRobustEstimatorTest
        implements PinholeCameraRobustEstimatorListener {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;

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

    private static final double THRESHOLD = 1e-4;

    private static final double OUTLIER_STD_ERROR = 100.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstants() {
        assertTrue(PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(0.05f, PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, PinholeCameraRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, PinholeCameraRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, PinholeCameraRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, PinholeCameraRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, PinholeCameraRobustEstimator.MIN_ITERATIONS);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(0.0, PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(1.0, PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertFalse(PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertEquals(6, PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES);
        assertTrue(PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_NORMALIZE_SUBSET_POINT_CORRESPONDENCES);
        assertEquals(RobustEstimatorMethod.PROMedS,
                PointCorrespondencePinholeCameraRobustEstimator.DEFAULT_ROBUST_METHOD);
        assertEquals(1.0, RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(0.0, RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.MIN_THRESHOLD, 0.0);
        assertFalse(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertFalse(RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testConstructor() {
        // test constructor without arguments
        RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        assertEquals(estimator.getThreshold(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);


        // test constructor with listener
        estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        this);

        assertEquals(estimator.getThreshold(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);


        // test constructor with points
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        points3D, points2D);

        assertEquals(estimator.getThreshold(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        // Force IllegalArgumentException
        final List<Point3D> points3DEmpty = new ArrayList<>();
        final List<Point2D> points2DEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                    points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                    points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener and points
        estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                        this, points3D, points2D);

        assertEquals(estimator.getThreshold(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getPoints2D(), points2D);
        assertSame(estimator.getPoints3D(), points3D);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertNull(estimator.getCovariance());
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(estimator.getPlanarThreshold(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(estimator.getSkewness(), 0.0, 0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        // Force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                    this, points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 10);

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);

        // set new value
        estimator.setResultRefined(!PinholeCameraRobustEstimator.
                DEFAULT_REFINE_RESULT);

        // check correctness
        assertEquals(estimator.isResultRefined(),
                !PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);

        // set new value
        estimator.setCovarianceKept(!PinholeCameraRobustEstimator.
                DEFAULT_KEEP_COVARIANCE);

        // check correctness
        assertEquals(estimator.isCovarianceKept(),
                !PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testIsSetFastRefinementUsed() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);

        // set new value
        estimator.setFastRefinementUsed(!PinholeCameraRobustEstimator.
                DEFAULT_USE_FAST_REFINEMENT);

        // check correctness
        assertEquals(estimator.isFastRefinementUsed(),
                !PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());
        assertFalse(estimator.isReady());

        // set new value
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator.setPoints(points3D, points2D);

        // check correctness
        assertSame(estimator.getPoints3D(), points3D);
        assertSame(estimator.getPoints2D(), points2D);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point3D> points3DEmpty = new ArrayList<>();
        final List<Point2D> points2DEmpty = new ArrayList<>();
        try {
            // not enough points
            estimator.setPoints(points3DEmpty, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setPoints(points3D, points2DEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // set new value
        estimator.setSuggestSkewnessValueEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }

    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                0.0);

        // set new value
        estimator.setSuggestedSkewnessValue(-1.0);

        // check correctness
        assertEquals(estimator.getSuggestedSkewnessValue(), -1.0, 0.0);
    }

    @Test
    public void testIsSetSuggestedHorizontalFocalLengthEnabled()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);

        // set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);

        // check correctness
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 100.0,
                0.0);
    }

    @Test
    public void testIsSetSuggestVerticalFocalLengthEnabled()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // set new value
        estimator.setSuggestVerticalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetSuggestedVerticalFocalLengthValue()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);

        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 100.0,
                0.0);
    }

    @Test
    public void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // set new value
        estimator.setSuggestAspectRatioEnabled(!PinholeCameraRobustEstimator.
                DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
    }

    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);

        // set new value
        estimator.setSuggestedAspectRatioValue(-1.0);

        // check correctness
        assertEquals(estimator.getSuggestedAspectRatioValue(), -1.0, 0.0);
    }

    @Test
    public void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // set new value
        estimator.setSuggestPrincipalPointEnabled(!PinholeCameraRobustEstimator.
                DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
    }

    @Test
    public void testGetSetSuggestedPrincipalPointValue()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getSuggestedPrincipalPointValue());

        // set new value
        final InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        estimator.setSuggestedPrincipalPointValue(principalPoint);

        // check correctness
        assertSame(estimator.getSuggestedPrincipalPointValue(), principalPoint);
    }

    @Test
    public void testIsSetSuggestRotationEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraRobustEstimator.
                DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestRotationEnabled(),
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
    }

    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getSuggestedRotationValue());

        // set new value
        final Quaternion q = new Quaternion();
        estimator.setSuggestedRotationValue(q);

        // check correctness
        assertSame(estimator.getSuggestedRotationValue(), q);
    }

    @Test
    public void testIsSetSuggestCenterEnabled() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestCenterEnabled(),
                RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);

        // set new value
        estimator.setSuggestCenterEnabled(
                !RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestCenterEnabled(),
                !RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
    }

    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertNull(estimator.getSuggestedCenterValue());

        // set new value
        final InhomogeneousPoint3D center = new InhomogeneousPoint3D();
        estimator.setSuggestedCenterValue(center);

        // check correctness
        assertSame(estimator.getSuggestedCenterValue(), center);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                PinholeCameraRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsNormalizeSubsetPointCorrespondences()
            throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());

        // set new value
        estimator.setNormalizeSubsetPointCorrespondences(true);

        // check correctness
        assertFalse(estimator.isNormalizeSubsetPointCorrespondences());
    }

    @Test
    public void testGetSetSkewness() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getSkewness(), 0.0, 0.0);

        // set new value
        estimator.setSkewness(1e-3);

        // check correctness
        assertEquals(estimator.getSkewness(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetHorizontalPrincipalPoint() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getHorizontalPrincipalPoint(), 0.0, 0.0);

        // set new value
        estimator.setHorizontalPrincipalPoint(100.0);

        // check correctness
        assertEquals(estimator.getHorizontalPrincipalPoint(), 100.0, 0.0);
    }

    @Test
    public void testGetSetVerticalPrincipalPoint() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getVerticalPrincipalPoint(), 0.0, 0.0);

        // set new value
        estimator.setVerticalPrincipalPoint(50.0);

        // check correctness
        assertEquals(estimator.getVerticalPrincipalPoint(), 50.0, 0.0);
    }

    @Test
    public void testIsReady() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertNull(estimator.getPoints2D());
        assertNull(estimator.getPoints3D());

        // set new value
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        for (int i = 0; i < PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES; i++) {
            points3D.add(Point3D.create());
            points2D.add(Point2D.create());
        }

        estimator.setPoints(points3D, points2D);

        // check correctness
        assertSame(estimator.getPoints3D(), points3D);
        assertSame(estimator.getPoints2D(), points2D);
        assertTrue(estimator.isReady());
    }

    @Test
    public void testIsSetPlanarConfigurationAllowed() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);

        // set new value
        estimator.setPlanarConfigurationAllowed(
                !UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);

        // check correctness
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                !UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
    }

    @Test
    public void testIsSetNullspaceDimension2Allowed() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);

        // set new value
        estimator.setNullspaceDimension2Allowed(
                !UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);

        // check correctness
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                !UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
    }

    @Test
    public void testGetSetPlanarThreshold() throws LockedException {
        final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getPlanarThreshold(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);

        // set new value
        estimator.setPlanarThreshold(1e9);

        // check correctness
        assertEquals(estimator.getPlanarThreshold(), 1e9, 0.0);
    }

    @Test
    public void testEstimateGeneralNoSuggestion()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            reset();

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNull(estimator.getCovariance());
            reset();

            // check correctness of estimation

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // Comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            assertEquals(focalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter,
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            if (Math.abs(rotation.getA() - estimatedRotation.getA()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getB() - estimatedRotation.getB()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getC() - estimatedRotation.getC()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getD() - estimatedRotation.getD()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    LARGE_ABSOLUTE_ERROR);

            if (!rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotMatrix.equals(estimatedRotMatrix,
                    LARGE_ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            Point2D originalPoint2D;
            Point2D estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D),
                        0.0, VERY_LARGE_ABSOLUTE_ERROR);
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
    public void testEstimateGeneralNoSuggestionWithRefinement()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        int numCovariances = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            if (Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(focalLength - estimatedIntrinsic.getVerticalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(horizontalPrincipalPoint - estimatedIntrinsic.getHorizontalPrincipalPoint()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(verticalPrincipalPoint - estimatedIntrinsic.getVerticalPrincipalPoint()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(skewness - estimatedIntrinsic.getSkewness()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    VERY_LARGE_ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter,
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            if (Math.abs(rotation.getA() - estimatedRotation.getA()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getB() - estimatedRotation.getB()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getC() - estimatedRotation.getC()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getD() - estimatedRotation.getD()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    LARGE_ABSOLUTE_ERROR);

            if (!rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotMatrix.equals(estimatedRotMatrix,
                    LARGE_ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            Point2D originalPoint2D;
            Point2D estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D),
                        0.0, VERY_LARGE_ABSOLUTE_ERROR);
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0 &&
                    numCovariances > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }

    @Test
    public void testEstimateGeneralNoSuggestionWithFastRefinement()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValidCameras = 0;
        int numValidProjections = 0;
        int numCovariances = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            if (Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(focalLength - estimatedIntrinsic.getVerticalFocalLength()) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(horizontalPrincipalPoint - estimatedIntrinsic.getHorizontalPrincipalPoint()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(verticalPrincipalPoint - estimatedIntrinsic.getVerticalPrincipalPoint()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(skewness - estimatedIntrinsic.getSkewness()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    VERY_LARGE_ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = camera2.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter,
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = camera2.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            if (Math.abs(rotation.getA() - estimatedRotation.getA()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getB() - estimatedRotation.getB()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getC() - estimatedRotation.getC()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getD() - estimatedRotation.getD()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    LARGE_ABSOLUTE_ERROR);

            if (!rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotMatrix.equals(estimatedRotMatrix,
                    LARGE_ABSOLUTE_ERROR));

            numValidCameras++;

            // project original 3D points using estimated camera and check
            // distance to 2D points without error
            Point2D originalPoint2D;
            Point2D estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point3D = points3D.get(i);
                originalPoint2D = points2D.get(i);
                estimatedPoint2D = camera2.project(point3D);

                if (originalPoint2D.distanceTo(estimatedPoint2D) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(originalPoint2D.distanceTo(estimatedPoint2D),
                        0.0, VERY_LARGE_ABSOLUTE_ERROR);
                numValidProjections++;
            }

            if (numValidCameras > 0 && numValidProjections > 0 &&
                    numCovariances > 0) {
                break;
            }
        }

        assertTrue(numValidCameras > 0);
        assertTrue(numValidProjections > 0);
        assertTrue(numCovariances > 0);
    }

    @Test
    public void testEstimateGeneralSuggestedRotationEnabled()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
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

            final Quaternion estimatedRotationNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();

            // check that rotation has become closer to suggested value
            final double diffEstimatedNoSuggestion =
                    Math.pow(rotation.getA() -
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                            Math.pow(rotation.getB() -
                                    estimatedRotationNoSuggestion.getB(), 2.0) +
                            Math.pow(rotation.getC() -
                                    estimatedRotationNoSuggestion.getC(), 2.0) +
                            Math.pow(rotation.getD() -
                                    estimatedRotationNoSuggestion.getD(), 2.0);
            final double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                            Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                            Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                            Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

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
    public void testEstimateGeneralSuggestedRotationWithRefinement()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
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

            final Quaternion estimatedRotationNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();

            // check that rotation has become closer to suggested value
            final double diffEstimatedNoSuggestion =
                    Math.pow(rotation.getA() -
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                            Math.pow(rotation.getB() -
                                    estimatedRotationNoSuggestion.getB(), 2.0) +
                            Math.pow(rotation.getC() -
                                    estimatedRotationNoSuggestion.getC(), 2.0) +
                            Math.pow(rotation.getD() -
                                    estimatedRotationNoSuggestion.getD(), 2.0);
            final double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                            Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                            Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                            Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

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
    public void testEstimateGeneralSuggestedRotationWithFastRefinement()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
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

            final Quaternion estimatedRotationNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraRotation().
                            toQuaternion();

            // check that rotation has become closer to suggested value
            final double diffEstimatedNoSuggestion =
                    Math.pow(rotation.getA() -
                            estimatedRotationNoSuggestion.getA(), 2.0) +
                            Math.pow(rotation.getB() -
                                    estimatedRotationNoSuggestion.getB(), 2.0) +
                            Math.pow(rotation.getC() -
                                    estimatedRotationNoSuggestion.getC(), 2.0) +
                            Math.pow(rotation.getD() -
                                    estimatedRotationNoSuggestion.getD(), 2.0);
            final double diffEstimated =
                    Math.pow(rotation.getA() - estimatedRotation.getA(), 2.0) +
                            Math.pow(rotation.getB() - estimatedRotation.getB(), 2.0) +
                            Math.pow(rotation.getC() - estimatedRotation.getC(), 2.0) +
                            Math.pow(rotation.getD() - estimatedRotation.getD(), 2.0);

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
    public void testEstimateGeneralSuggestedCenterEnabled()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final Point3D estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final Point3D estimatedCenterNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateGeneralSuggestedCenterWithRefinement()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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
            final Point3D estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final Point3D estimatedCenterNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
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
    public void testEstimateGeneralSuggestedCenterWithFastRefinement()
            throws LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // create outliers
            Point2D point2DWithError;
            final List<Point2D> points2DWithError = new ArrayList<>();
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, OUTLIER_STD_ERROR);
            for (final Point2D point2D : points2D) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double errorW = errorRandomizer.nextDouble();
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

            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator =
                    new RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator(
                            this, points3D, points2DWithError);

            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

            estimator.setThreshold(THRESHOLD);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
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
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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
            final Point3D estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final Point3D estimatedCenterNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >=
                    cameraCenter.distanceTo(estimatedCenter)) {
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
        checkLocked(
                (RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraRobustEstimator estimator) {
        estimateEnd++;
        checkLocked(
                (RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final PinholeCameraRobustEstimator estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked(
                (RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final PinholeCameraRobustEstimator estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked(
                (RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator estimator) {
        final List<Point3D> points3D = new ArrayList<>();
        final List<Point2D> points2D = new ArrayList<>();
        try {
            estimator.setPoints(points3D, points2D);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setNormalizeSubsetPointCorrespondences(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setSuggestSkewnessValueEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedSkewnessValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedHorizontalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedVerticalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestAspectRatioEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedAspectRatioValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestPrincipalPointEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedPrincipalPointValue(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestRotationEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedRotationValue(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestCenterEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestedCenterValue(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFastRefinementUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPlanarConfigurationAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setNullspaceDimension2Allowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPlanarThreshold(1e9);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSkewness(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setHorizontalPrincipalPoint(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setVerticalPrincipalPoint(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        assertTrue(estimator.isLocked());
    }
}
