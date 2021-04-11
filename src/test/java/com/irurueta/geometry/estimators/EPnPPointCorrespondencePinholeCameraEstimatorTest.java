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
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class EPnPPointCorrespondencePinholeCameraEstimatorTest implements
        PinholeCameraEstimatorListener {

    private static final double ABSOLUTE_ERROR = 1e-6;

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

    private static final int N_POINTS = 6;

    private static final int TIMES = 100;

    private static final double ERROR_STD = 1e-5;

    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;

    @Test
    public void testConstructor() throws WrongListSizesException,
            NotAvailableException {

        // testing constructor without parameters
        EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        assertNull(estimator.getListener());
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertNull(estimator.getIntrinsic());
        assertFalse(estimator.isPlanar());


        // test estimate with listener
        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(this);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertNull(estimator.getIntrinsic());
        assertFalse(estimator.isPlanar());


        // testing constructor with lists
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final List<Point3D> points3D = new ArrayList<>(N_POINTS);
        final List<Point2D> points2D = new ArrayList<>(N_POINTS);
        for (int i = 0; i < N_POINTS; i++) {
            points3D.add(new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            points2D.add(new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }

        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(points3D,
                points2D);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
        assertNull(estimator.getListener());
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertNull(estimator.getIntrinsic());
        assertFalse(estimator.isPlanar());

        // Force WrongListSizesException
        List<Point3D> wrong3D = new ArrayList<>();
        List<Point2D> wrong2D = new ArrayList<>();
        estimator = null;
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(null,
                    points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // testing constructor with lists and listener
        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(points3D,
                points2D, this);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertNull(estimator.getIntrinsic());
        assertFalse(estimator.isPlanar());

        // Force WrongListSizesException
        wrong3D = new ArrayList<>();
        wrong2D = new ArrayList<>();
        estimator = null;
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    wrong3D, points2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, wrong2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(null,
                    points2D, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with intrinsics
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                intrinsic);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        assertNull(estimator.getListener());
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertSame(estimator.getIntrinsic(), intrinsic);
        assertFalse(estimator.isPlanar());


        // test constructor with intrinsics and listener
        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                intrinsic, this);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertSame(estimator.getIntrinsic(), intrinsic);
        assertFalse(estimator.isPlanar());


        // test constructor with intrinsics and points
        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                intrinsic, points3D, points2D);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
        assertNull(estimator.getListener());
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertSame(estimator.getIntrinsic(), intrinsic);
        assertFalse(estimator.isPlanar());

        // Force WrongListSizesException
        estimator = null;
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, null, points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with intrinsics and points
        estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                intrinsic, points3D, points2D, this);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertTrue(estimator.isNullspaceDimension3Allowed());
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                0.0);
        assertSame(estimator.getIntrinsic(), intrinsic);
        assertFalse(estimator.isPlanar());

        // Force WrongListSizesException
        estimator = null;
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, wrong3D, points2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, points3D, wrong2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, null, points2D, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EPnPPointCorrespondencePinholeCameraEstimator(
                    intrinsic, points3D, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testAreSetPointCorrespondencesNormalized()
            throws LockedException {

        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        assertFalse(estimator.arePointCorrespondencesNormalized());

        // check it can't be enabled
        estimator.setPointCorrespondencesNormalized(true);

        assertFalse(estimator.arePointCorrespondencesNormalized());
    }

    @Test
    public void testGetSetListsValidityAndAvailability() throws LockedException,
            IllegalArgumentException, WrongListSizesException,
            NotAvailableException {

        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.isReady());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final List<Point3D> points3D = new ArrayList<>(N_POINTS);
        final List<Point2D> points2D = new ArrayList<>(N_POINTS);
        for (int i = 0; i < N_POINTS; i++) {
            points3D.add(new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            points2D.add(new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }

        // set lists
        assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, points2D));

        estimator.setLists(points3D, points2D);

        // check correctness
        assertEquals(points3D, estimator.getPoints3D());
        assertEquals(points2D, estimator.getPoints2D());
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.isReady());

        // Force WrongListSizesException
        final List<Point3D> wrong3D = new ArrayList<>();
        final List<Point2D> wrong2D = new ArrayList<>();
        assertFalse(EPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                wrong3D, points2D));
        try {
            estimator.setLists(wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        assertFalse(EPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, wrong2D));
        try {
            estimator.setLists(points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        assertFalse(EPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                null, points2D));
        try {
            estimator.setLists(null, points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertFalse(EPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, null));
        try {
            estimator.setLists(points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        assertNull(estimator.getListener());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // set new value
        estimator.setSuggestSkewnessValueEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }

    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);

        // set new value
        estimator.setSuggestedSkewnessValue(1e-3);

        // check correctness
        assertEquals(estimator.getSuggestedSkewnessValue(), 1e-3, 0.0);
    }

    @Test
    public void testIsSetSuggestHorizontalFocalLengthEnabled()
            throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue()
            throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

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
    public void testIsSetSuggestedVerticalFocalLengthEnabled()
            throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // set new value
        estimator.setSuggestVerticalFocalLengthEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetSuggestedVerticalFocalLengthValue()
            throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0,
                0.0);

        // set new value
        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 100.0,
                0.0);
    }

    @Test
    public void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // set new value
        estimator.setSuggestAspectRatioEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
    }

    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);

        // set new value
        estimator.setSuggestedAspectRatioValue(-1.0);

        // check correctness
        assertEquals(estimator.getSuggestedAspectRatioValue(), -1.0, 0.0);
    }

    @Test
    public void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // set new value
        estimator.setSuggestPrincipalPointEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
    }

    @Test
    public void testGetSetSuggestedPrincipalPointValue()
            throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

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
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // set new value
        estimator.setSuggestRotationEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestRotationEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
    }

    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

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
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // set new value
        estimator.setSuggestCenterEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestCenterEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
    }

    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertNull(estimator.getSuggestedCenterValue());

        // set new value
        final InhomogeneousPoint3D center = new InhomogeneousPoint3D();
        estimator.setSuggestedCenterValue(center);

        // check correctness
        assertSame(estimator.getSuggestedCenterValue(), center);
    }

    @Test
    public void testGetSetMinSuggestionWeight() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);

        // set new value
        estimator.setMinSuggestionWeight(1.0);

        // check correctness
        assertEquals(estimator.getMinSuggestionWeight(), 1.0, 0.0);
    }

    @Test
    public void testGetSetMaxSuggestionWeight() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);

        // set new value
        estimator.setMaxSuggestionWeight(1.0);

        // check correctness
        assertEquals(estimator.getMaxSuggestionWeight(), 1.0, 0.0);
    }

    @Test
    public void testSetMinMaxSuggestionWeight() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);

        // set new value
        estimator.setMinMaxSuggestionWeight(10.0, 20.0);

        // check correctness
        assertEquals(estimator.getMinSuggestionWeight(), 10.0, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(), 20.0, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setMinMaxSuggestionWeight(10.0, 10.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetSuggestionWeightStep() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);

        // set new value
        estimator.setSuggestionWeightStep(1.0);

        // check correctness
        assertEquals(estimator.getSuggestionWeightStep(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setSuggestionWeightStep(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetPlanarConfigurationAllowed() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);

        // set new value
        estimator.setPlanarConfigurationAllowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);

        // check correctness
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
    }

    @Test
    public void testIsSetNullspaceDimension2Allowed() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);

        // set new value
        estimator.setNullspaceDimension2Allowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);

        // check correctness
        assertEquals(estimator.isNullspaceDimension2Allowed(),
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
    }

    @Test
    public void testIsSetNullspaceDimension3Allowed() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);

        // set new value
        estimator.setNullspaceDimension3Allowed(
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);

        // check correctness
        assertEquals(estimator.isNullspaceDimension3Allowed(),
                !EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_NULLSPACE_DIMENSION3_ALLOWED);
    }

    @Test
    public void testGetSetPlanarThreshold() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(estimator.getPlanarThreshold(),
                EPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);

        // set new value
        estimator.setPlanarThreshold(1e-3);

        // check correctness
        assertEquals(estimator.getPlanarThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetIntrinsic() throws LockedException {
        final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                new EPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertNull(estimator.getIntrinsic());

        // set new value
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertSame(estimator.getIntrinsic(), intrinsic);
    }

    @Test
    public void testEstimateGeneralNoSuggestion()
            throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
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

            final int nPoints = EPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new EPnPPointCorrespondencePinholeCameraEstimator(intrinsic,
                            points3D, points2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            assertEquals(horizontalFocalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = estimatedCamera.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            // project original points using estimated camera
            final List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, ABSOLUTE_ERROR));
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimatePlanarNoSuggestion()
            throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
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

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final double[] principalAxis = camera.getPrincipalAxisArray();
            final double depth = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final Plane plane = new Plane(centroid, principalAxis);

            final int nPoints = EPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;


            final double a = plane.getA();
            final double b = plane.getB();
            final double c = plane.getC();
            final double d = plane.getD();

            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final double homW = 1.0;
                final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new EPnPPointCorrespondencePinholeCameraEstimator(intrinsic,
                            points3D, points2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            if (!estimator.isPlanar()) {
                continue;
            }
            assertTrue(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            assertEquals(horizontalFocalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = new InhomogeneousPoint3D(
                    estimatedCamera.getCameraCenter());
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            // project original points using estimated camera
            final List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, ABSOLUTE_ERROR));
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateGeneralSuggestedRotationEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
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

            final int nPoints = EPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // add error to projected points
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);

            for (final Point2D point2D : points2D) {
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX,
                        point2D.getInhomY() + errorY);
            }

            assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new EPnPPointCorrespondencePinholeCameraEstimator(intrinsic,
                            points3D, points2D, this);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final Quaternion estimatedQ = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedQ.normalize();

            // estimate without suggestion
            estimator.setSuggestRotationEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final Quaternion estimatedQNoSuggestion = estimatedCameraNoSuggestion.
                    getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final double diffEstimatedNoSuggestion =
                    Math.pow(rotation.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                            Math.pow(rotation.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                            Math.pow(rotation.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                            Math.pow(rotation.getD() - estimatedQNoSuggestion.getD(), 2.0);
            final double diffEstimated =
                    Math.pow(rotation.getA() - estimatedQ.getA(), 2.0) +
                            Math.pow(rotation.getB() - estimatedQ.getB(), 2.0) +
                            Math.pow(rotation.getC() - estimatedQ.getC(), 2.0) +
                            Math.pow(rotation.getD() - estimatedQ.getD(), 2.0);

            if (diffEstimatedNoSuggestion > diffEstimated) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateGeneralSuggestedCenterEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
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

            final int nPoints = EPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            // add error to projected points
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);

            for (final Point2D point2D : points2D) {
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX,
                        point2D.getInhomY() + errorY);
            }

            assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new EPnPPointCorrespondencePinholeCameraEstimator(intrinsic,
                            points3D, points2D, this);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final Point3D estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final Point3D estimatedCenterNoSuggestion =
                    estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >
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
    public void testEstimateGeneralZeroSkewnessZeroPrincipalPointAndEqualsFocalLengthNoSuggestion()
            throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

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

            final int nPoints = EPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new EPnPPointCorrespondencePinholeCameraEstimator(intrinsic,
                            points3D, points2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            assertEquals(focalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = estimatedCamera.getCameraCenter();
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            // project original points using estimated camera
            final List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimatePlanarZeroSkewnessZeroPrincipalPointAndEqualsFocalLengthNoSuggestion()
            throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // intrinsic parameters
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

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

            // generate plane parallel to the camera's principal plane and
            // located at certain distance from it.
            final double[] principalAxis = camera.getPrincipalAxisArray();
            final double depth = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));

            final Plane plane = new Plane(centroid, principalAxis);

            final int nPoints = EPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;


            final double a = plane.getA();
            final double b = plane.getB();
            final double c = plane.getC();
            final double d = plane.getD();

            final List<Point3D> points3D = new ArrayList<>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final double homW = 1.0;
                final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }

                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));
                assertTrue(camera.isPointInFrontOfCamera(point3D));

                points3D.add(point3D);
            }

            final List<Point2D> points2D = camera.project(points3D);

            assertTrue(EPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            final EPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new EPnPPointCorrespondencePinholeCameraEstimator(intrinsic,
                            points3D, points2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertTrue(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            assertEquals(focalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                    ABSOLUTE_ERROR);

            // comparing estimated camera center
            final Point3D estimatedCameraCenter = new InhomogeneousPoint3D(
                    estimatedCamera.getCameraCenter());
            if (!cameraCenter.equals(estimatedCameraCenter, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    ABSOLUTE_ERROR));

            // comparing estimated rotation
            final Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();

            final Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            final Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(),
                    ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(),
                    ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, ABSOLUTE_ERROR));

            // project original points using estimated camera
            final List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }


    @Override
    public void onEstimateStart(final PinholeCameraEstimator estimator) {
        estimateStart++;
        checkIsLocked((EPnPPointCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraEstimator estimator) {
        estimateEnd++;
        checkIsLocked((EPnPPointCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final PinholeCameraEstimator estimator,
                                           final float progress) {
        estimationProgressChange++;
        checkIsLocked((EPnPPointCorrespondencePinholeCameraEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private void checkIsLocked(
            final EPnPPointCorrespondencePinholeCameraEstimator estimator) {
        try {
            estimator.setLists(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
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
            estimator.setNullspaceDimension3Allowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPlanarThreshold(1e9);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setIntrinsic(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPointCorrespondencesNormalized(true);
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
            estimator.setSuggestedAspectRatioValue(1.0);
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
            estimator.setMinSuggestionWeight(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxSuggestionWeight(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMinMaxSuggestionWeight(1.0, 1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestionWeightStep(0.2);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        assertTrue(estimator.isLocked());
    }
}
