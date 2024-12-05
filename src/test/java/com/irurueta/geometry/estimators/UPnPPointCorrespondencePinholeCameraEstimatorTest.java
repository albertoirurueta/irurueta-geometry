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
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class UPnPPointCorrespondencePinholeCameraEstimatorTest implements PinholeCameraEstimatorListener {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 1.0;

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

    private static final double PLANE_SLANT_DEGREES = 45.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;

    @Test
    void testConstants() {
        assertEquals(6, PointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES);
        assertTrue(PointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        assertEquals(1e-8, PointCorrespondencePinholeCameraEstimator.EPS, 0.0);
        assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_NULLSPACE_DIMENSION2_ALLOWED);
        assertEquals(1e13, UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD, 0.0);
        assertEquals(0.0, UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, 0.0);
        assertEquals(0.0, UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(0.0, UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_VERTICAL_PRINCIPAL_POINT, 0.0);
    }

    @Test
    void testConstructor() throws WrongListSizesException, NotAvailableException {
        // testing constructor without parameters
        var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(PinholeCameraEstimatorType.UPNP_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertThrows(NotAvailableException.class, estimator::getPoints2D);
        assertThrows(NotAvailableException.class, estimator::getPoints3D);
        assertNull(estimator.getListener());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, estimator.getSuggestedSkewnessValue(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, estimator.getMinSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, estimator.getMaxSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, estimator.getSuggestionWeightStep(),
                0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, estimator.getSkewness(),
                0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_HORIZONTAL_PRINCIPAL_POINT,
                estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_VERTICAL_PRINCIPAL_POINT,
                estimator.getVerticalPrincipalPoint(), 0.0);
        assertFalse(estimator.isPlanar());

        // testing constructor with listener
        estimator = new UPnPPointCorrespondencePinholeCameraEstimator(this);

        // check correctness
        assertEquals(PinholeCameraEstimatorType.UPNP_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertThrows(NotAvailableException.class, estimator::getPoints2D);
        assertThrows(NotAvailableException.class, estimator::getPoints3D);
        assertSame(this, estimator.getListener());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, estimator.getSuggestedSkewnessValue(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, estimator.getMinSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, estimator.getMaxSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, estimator.getSuggestionWeightStep(),
                0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, estimator.getSkewness(),
                0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_HORIZONTAL_PRINCIPAL_POINT,
                estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_VERTICAL_PRINCIPAL_POINT,
                estimator.getVerticalPrincipalPoint(), 0.0);
        assertFalse(estimator.isPlanar());

        // testing constructor with lists
        final var randomizer = new UniformRandomizer();
        final var points3D = new ArrayList<Point3D>(N_POINTS);
        final var points2D = new ArrayList<Point2D>(N_POINTS);
        for (var i = 0; i < N_POINTS; i++) {
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

        estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D);

        // check correctness
        assertEquals(PinholeCameraEstimatorType.UPNP_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(points2D, estimator.getPoints2D());
        assertEquals(points3D, estimator.getPoints3D());
        assertNull(estimator.getListener());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, estimator.getSuggestedSkewnessValue(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, estimator.getMinSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, estimator.getMaxSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, estimator.getSuggestionWeightStep(),
                0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, estimator.getSkewness(),
                0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_HORIZONTAL_PRINCIPAL_POINT,
                estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_VERTICAL_PRINCIPAL_POINT,
                estimator.getVerticalPrincipalPoint(), 0.0);
        assertFalse(estimator.isPlanar());

        // Force WrongListSizesException
        final var wrong3D = new ArrayList<Point3D>();
        final var wrong2D = new ArrayList<Point2D>();
        assertThrows(WrongListSizesException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(wrong3D,
                points2D));
        assertThrows(WrongListSizesException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(points3D,
                wrong2D));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(
                null, points2D));
        assertThrows(IllegalArgumentException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(
                points3D, null));

        // testing constructor with lists and listener
        estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);

        // check correctness
        assertEquals(PinholeCameraEstimatorType.UPNP_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(points2D, estimator.getPoints2D());
        assertEquals(points3D, estimator.getPoints3D());
        assertSame(this, estimator.getListener());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, estimator.getSuggestedSkewnessValue(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, estimator.getMinSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, estimator.getMaxSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, estimator.getSuggestionWeightStep(),
                0.0);
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, estimator.getSkewness(),
                0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_HORIZONTAL_PRINCIPAL_POINT,
                estimator.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_VERTICAL_PRINCIPAL_POINT,
                estimator.getVerticalPrincipalPoint(), 0.0);
        assertFalse(estimator.isPlanar());

        // Force WrongListSizesException
        assertThrows(WrongListSizesException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(wrong3D,
                points2D, this));
        assertThrows(WrongListSizesException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(points3D,
                wrong2D, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(
                null, points2D, this));
        assertThrows(IllegalArgumentException.class, () -> new UPnPPointCorrespondencePinholeCameraEstimator(points3D,
                null, this));
    }

    @Test
    void testAreSetPointCorrespondencesNormalized() throws LockedException {

        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        assertFalse(estimator.arePointCorrespondencesNormalized());

        // check it can't be enabled
        estimator.setPointCorrespondencesNormalized(true);

        assertFalse(estimator.arePointCorrespondencesNormalized());
    }

    @Test
    void testGetSetListsValidityAndAvailability() throws LockedException, IllegalArgumentException,
            WrongListSizesException, NotAvailableException {

        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.isReady());
        assertThrows(NotAvailableException.class, estimator::getPoints2D);
        assertThrows(NotAvailableException.class, estimator::getPoints3D);

        final var randomizer = new UniformRandomizer();
        final var points3D = new ArrayList<Point3D>(N_POINTS);
        final var points2D = new ArrayList<Point2D>(N_POINTS);
        for (var i = 0; i < N_POINTS; i++) {
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
        assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

        estimator.setLists(points3D, points2D);

        // check correctness
        assertEquals(points3D, estimator.getPoints3D());
        assertEquals(points2D, estimator.getPoints2D());
        assertTrue(estimator.areListsAvailable());
        assertTrue(estimator.isReady());

        // Force WrongListSizesException
        final var wrong3D = new ArrayList<Point3D>();
        final var wrong2D = new ArrayList<Point2D>();
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(wrong3D, points2D));
        assertThrows(WrongListSizesException.class, () -> estimator.setLists(wrong3D, points2D));
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, wrong2D));
        assertThrows(WrongListSizesException.class, () -> estimator.setLists(points3D, wrong2D));

        // Force IllegalArgumentException
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(null, points2D));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLists(null, points2D));
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLists(points3D, null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        assertNull(estimator.getListener());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());

        // set new value
        estimator.setSuggestSkewnessValueEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
    }

    @Test
    void testGetSetSuggestedSkewnessValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, estimator.getSuggestedSkewnessValue(),
                0.0);

        // set new value
        estimator.setSuggestedSkewnessValue(1e-3);

        // check correctness
        assertEquals(1e-3, estimator.getSuggestedSkewnessValue(), 0.0);
    }

    @Test
    void testIsSetSuggestHorizontalFocalLengthEnabled() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());

        // set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
    }

    @Test
    void testGetSetSuggestedHorizontalFocalLengthValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestedVerticalFocalLengthEnabled() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());

        // set new value
        estimator.setSuggestVerticalFocalLengthEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
    }

    @Test
    void testGetSetSuggestedVerticalFocalLengthValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());

        // set new value
        estimator.setSuggestAspectRatioEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
    }

    @Test
    void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);

        // set new value
        estimator.setSuggestedAspectRatioValue(-1.0);

        // check correctness
        assertEquals(-1.0, estimator.getSuggestedAspectRatioValue(), 0.0);
    }

    @Test
    void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());

        // set new value
        estimator.setSuggestPrincipalPointEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
    }

    @Test
    void testGetSetSuggestedPrincipalPointValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());

        // set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
    }

    @Test
    void testGetSetSuggestedRotationValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());

        // set new value
        estimator.setSuggestCenterEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
    }

    @Test
    void testGetSetSuggestedCenterValue() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertNull(estimator.getSuggestedCenterValue());

        // set new value
        final var center = new InhomogeneousPoint3D();
        estimator.setSuggestedCenterValue(center);

        // check correctness
        assertSame(center, estimator.getSuggestedCenterValue());
    }

    @Test
    void testGetSetMinSuggestionWeight() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, estimator.getMinSuggestionWeight(),
                0.0);

        // set new value
        estimator.setMinSuggestionWeight(1.0);

        // check correctness
        assertEquals(1.0, estimator.getMinSuggestionWeight(), 0.0);
    }

    @Test
    void testGetSetMaxSuggestionWeight() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, estimator.getMaxSuggestionWeight(),
                0.0);

        // set new value
        estimator.setMaxSuggestionWeight(1.0);

        // check correctness
        assertEquals(1.0, estimator.getMaxSuggestionWeight(), 0.0);
    }

    @Test
    void testSetMinMaxSuggestionWeight() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, estimator.getMinSuggestionWeight(),
                0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, estimator.getMaxSuggestionWeight(),
                0.0);

        // set new value
        estimator.setMinMaxSuggestionWeight(10.0, 20.0);

        // check correctness
        assertEquals(10.0, estimator.getMinSuggestionWeight(), 0.0);
        assertEquals(20.0, estimator.getMaxSuggestionWeight(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMinMaxSuggestionWeight(10.0, 10.0));
    }

    @Test
    void testGetSetSuggestionWeightStep() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, estimator.getSuggestionWeightStep(),
                0.0);

        // set new value
        estimator.setSuggestionWeightStep(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSuggestionWeightStep(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setSuggestionWeightStep(0.0));
    }

    @Test
    void testIsSetPlanarConfigurationAllowed() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());

        // set new value
        estimator.setPlanarConfigurationAllowed(
                !UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED);

        // check correctness
        assertEquals(!UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_CONFIGURATION_ALLOWED,
                estimator.isPlanarConfigurationAllowed());
    }

    @Test
    void testIsSetNullspaceDimension2Allowed() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertTrue(estimator.isNullspaceDimension2Allowed());

        // set new value
        estimator.setNullspaceDimension2Allowed(false);

        // check correctness
        assertFalse(estimator.isNullspaceDimension2Allowed());
    }

    @Test
    void testGetSetPlanarThreshold() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD,
                estimator.getPlanarThreshold(), 0.0);

        // set new value
        estimator.setPlanarThreshold(1e-3);

        // check correctness
        assertEquals(1e-3, estimator.getPlanarThreshold(), 0.0);
    }

    @Test
    void testGetSetSkewness() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, estimator.getSkewness(),
                0.0);

        // set new value
        estimator.setSkewness(1e-3);

        // check correctness
        assertEquals(1e-3, estimator.getSkewness(), 0.0);
    }

    @Test
    void testGetSetHorizontalPrincipalPoint() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_HORIZONTAL_PRINCIPAL_POINT,
                estimator.getHorizontalPrincipalPoint(), 0.0);

        // set new value
        estimator.setHorizontalPrincipalPoint(100.0);

        // check correctness
        assertEquals(100.0, estimator.getHorizontalPrincipalPoint(), 0.0);
    }

    @Test
    void testGetSetVerticalPrincipalPoint() throws LockedException {
        final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_VERTICAL_PRINCIPAL_POINT,
                estimator.getVerticalPrincipalPoint(), 0.0);

        // set new value
        estimator.setVerticalPrincipalPoint(100.0);

        // check correctness
        assertEquals(100.0, estimator.getVerticalPrincipalPoint(), 0.0);
    }

    @Test
    void testEstimateGeneralNoSuggestion() throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
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

            final var nPoints = UPnPPointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

            final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

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
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertEquals(0, estimationProgressChange);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            if (Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength, estimatedIntrinsic.getHorizontalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength, estimatedIntrinsic.getVerticalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = new InhomogeneousPoint3D(estimatedCamera.getCameraCenter());
            if (!cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            if (Math.abs(rotation.getA() - estimatedRotation.getA()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getA(), estimatedRotation.getA(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getB() - estimatedRotation.getB()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getB(), estimatedRotation.getB(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getC() - estimatedRotation.getC()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getC(), estimatedRotation.getC(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getD() - estimatedRotation.getD()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getD(), estimatedRotation.getD(), LARGE_ABSOLUTE_ERROR);

            if (!rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR));

            // project original points using estimated camera
            final var estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            for (var i = 0; i < nPoints; i++) {
                final var point2D = points2D.get(i);
                final var estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, VERY_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarNoSuggestion() throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
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

            // plane is completely parallel to image plane
            var plane = new Plane(centroid, principalAxis);

            // we rotate plane slightly using a perpendicular axis
            var axis = camera.getHorizontalAxisPlane().getDirectorVector();
            final var angle = com.irurueta.geometry.Utils.convertToRadians(PLANE_SLANT_DEGREES);
            var planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);

            axis = camera.getVerticalAxisPlane().getDirectorVector();
            planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);

            final var nPoints = UPnPPointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;

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

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

            final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

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
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertEquals(0, estimationProgressChange);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            if (Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength, estimatedIntrinsic.getHorizontalFocalLength(), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength, estimatedIntrinsic.getVerticalFocalLength(), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = new InhomogeneousPoint3D(estimatedCamera.getCameraCenter());
            if (!cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            if (Math.abs(rotation.getA() - estimatedRotation.getA()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getA(), estimatedRotation.getA(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getB() - estimatedRotation.getB()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getB(), estimatedRotation.getB(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getC() - estimatedRotation.getC()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getC(), estimatedRotation.getC(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getD() - estimatedRotation.getD()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getD(), estimatedRotation.getD(), LARGE_ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR));

            // project original points using estimated camera
            final var estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            for (var i = 0; i < nPoints; i++) {
                final var point2D = points2D.get(i);
                final var estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, VERY_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralSuggestedRotationEnabled() throws WrongListSizesException, LockedException,
            NotReadyException, CameraException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
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

            final var nPoints = UPnPPointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // add error to projected points
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            for (final var point2D : points2D) {
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX,
                        point2D.getInhomY() + errorY);
            }

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

            final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
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
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertEquals(0, estimationProgressChange);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing rotation
            final var estimatedQ = estimatedCamera.getCameraRotation().toQuaternion();
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

            final var estimatedQNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(rotation.getA() - estimatedQNoSuggestion.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedQNoSuggestion.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedQNoSuggestion.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedQNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(rotation.getA() - estimatedQ.getA(), 2.0)
                    + Math.pow(rotation.getB() - estimatedQ.getB(), 2.0)
                    + Math.pow(rotation.getC() - estimatedQ.getC(), 2.0)
                    + Math.pow(rotation.getD() - estimatedQ.getD(), 2.0);

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
    void testEstimateGeneralSuggestedCenterEnabled() throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
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

            final var nPoints = UPnPPointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            // add error to projected points
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            for (final var point2D : points2D) {
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX,
                        point2D.getInhomY() + errorY);
            }

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

            final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
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
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertEquals(0, estimationProgressChange);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing center
            final var estimatedCenter = estimatedCamera.getCameraCenter();

            // estimate without suggestion
            estimator.setSuggestCenterEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedCenterNoSuggestion = estimatedCameraNoSuggestion.getCameraCenter();

            // check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) > cameraCenter.distanceTo(estimatedCenter)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateGeneralZeroSkewnessZeroPrincipalPointNoSuggestion() throws WrongListSizesException,
            LockedException, NotReadyException, CameraException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
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

            final var nPoints = UPnPPointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;

            final var points3D = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

            final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

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
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertEquals(0, estimationProgressChange);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            assertEquals(focalLength, estimatedIntrinsic.getHorizontalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength, estimatedIntrinsic.getVerticalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = new InhomogeneousPoint3D(estimatedCamera.getCameraCenter());
            if (!cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), LARGE_ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR));

            // project original points using estimated camera
            final var estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            for (var i = 0; i < nPoints; i++) {
                final var point2D = points2D.get(i);
                final var estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, VERY_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanarZeroSkewnessZeroPrincipalPointNoSuggestion() throws WrongListSizesException, LockedException,
            NotReadyException, CameraException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // intrinsic parameters
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
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

            // plane is completely parallel to image plane
            var plane = new Plane(centroid, principalAxis);

            // we rotate plane slightly using a perpendicular axis
            var axis = camera.getHorizontalAxisPlane().getDirectorVector();
            final var angle = com.irurueta.geometry.Utils.convertToRadians(PLANE_SLANT_DEGREES);
            var planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);

            axis = camera.getVerticalAxisPlane().getDirectorVector();
            planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);

            final var nPoints = UPnPPointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;

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

                points3D.add(point3D);
            }

            final var points2D = camera.project(points3D);

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

            final var estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);

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
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertEquals(0, estimationProgressChange);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            if (Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength, estimatedIntrinsic.getHorizontalFocalLength(), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength, estimatedIntrinsic.getVerticalFocalLength(), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), ABSOLUTE_ERROR);

            // comparing estimated camera center
            final var estimatedCameraCenter = new InhomogeneousPoint3D(estimatedCamera.getCameraCenter());
            if (!cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));

            // comparing estimated rotation
            final var estimatedRotation = estimatedCamera.getCameraRotation().toQuaternion();
            estimatedRotation.normalize();

            final var rotMatrix = rotation.asInhomogeneousMatrix();
            final var estimatedRotMatrix = estimatedRotation.asInhomogeneousMatrix();

            assertEquals(rotation.getA(), estimatedRotation.getA(), LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), LARGE_ABSOLUTE_ERROR);

            assertTrue(rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR));

            // project original points using estimated camera
            final var estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            for (var i = 0; i < nPoints; i++) {
                final var point2D = points2D.get(i);
                final var estimatedPoint2D = estimatedPoints2D.get(i);

                assertTrue(point2D.equals(estimatedPoint2D, VERY_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }


    @Override
    public void onEstimateStart(final PinholeCameraEstimator estimator) {
        estimateStart++;
        checkIsLocked((UPnPPointCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraEstimator estimator) {
        estimateEnd++;
        checkIsLocked((UPnPPointCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final PinholeCameraEstimator estimator, final float progress) {
        estimationProgressChange++;
        checkIsLocked((UPnPPointCorrespondencePinholeCameraEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private static void checkIsLocked(final UPnPPointCorrespondencePinholeCameraEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setLists(null, null));
        assertThrows(LockedException.class, () -> estimator.setPlanarConfigurationAllowed(true));
        assertThrows(LockedException.class, () -> estimator.setNullspaceDimension2Allowed(true));
        assertThrows(LockedException.class, () -> estimator.setPlanarThreshold(1e9));
        assertThrows(LockedException.class, () -> estimator.setSkewness(0.0));
        assertThrows(LockedException.class, () -> estimator.setHorizontalPrincipalPoint(0.0));
        assertThrows(LockedException.class, () -> estimator.setVerticalPrincipalPoint(0.0));
        assertThrows(LockedException.class, () -> estimator.setPointCorrespondencesNormalized(true));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.setSuggestSkewnessValueEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedSkewnessValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestHorizontalFocalLengthEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedHorizontalFocalLengthValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestVerticalFocalLengthEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedVerticalFocalLengthValue(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestAspectRatioEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedAspectRatioValue(1.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestPrincipalPointEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedPrincipalPointValue(null));
        assertThrows(LockedException.class, () -> estimator.setSuggestRotationEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedRotationValue(null));
        assertThrows(LockedException.class, () -> estimator.setSuggestCenterEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setSuggestedCenterValue(null));
        assertThrows(LockedException.class, () -> estimator.setMinSuggestionWeight(1.0));
        assertThrows(LockedException.class, () -> estimator.setMaxSuggestionWeight(1.0));
        assertThrows(LockedException.class, () -> estimator.setMinMaxSuggestionWeight(1.0, 1.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestionWeightStep(0.2));
        assertTrue(estimator.isLocked());
    }
}
