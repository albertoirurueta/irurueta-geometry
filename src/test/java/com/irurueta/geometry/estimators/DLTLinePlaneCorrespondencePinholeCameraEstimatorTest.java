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

import com.irurueta.geometry.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class DLTLinePlaneCorrespondencePinholeCameraEstimatorTest implements PinholeCameraEstimatorListener {

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1.0;

    private static final double MIN_RANDOM_VALUE = 0.0;
    private static final double MAX_RANDOM_VALUE = 1.0;

    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 90.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = 10.0;
    private static final double MAX_ANGLE_DEGREES = 15.0;

    private static final int INHOM_3D_COORDS = 3;

    private static final int N_CORRESPONDENCES = 4;
    private static final int MIN_NUMBER_CORRESPONDENCES = 5;
    private static final int MAX_NUMBER_CORRESPONDENCES = 100;

    private static final int TIMES = 100;

    private static final double ERROR_STD = 1e-5;

    private int startCount = 0;
    private int endCount = 0;
    private int progressCount = 0;

    @Test
    void testConstants() {
        assertEquals(11, DLTLinePlaneCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_EQUATIONS);
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(1e-8, DLTLinePlaneCorrespondencePinholeCameraEstimator.EPS, 0.0);
    }

    @Test
    void testConstructor() throws WrongListSizesException, NotAvailableException {

        // testing constructor without parameters
        var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());

        assertThrows(NotAvailableException.class, estimator::getLines2D);
        assertThrows(NotAvailableException.class, estimator::getPlanes);
        assertNull(estimator.getListener());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
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

        // testing constructor with listener
        estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(this);

        // check correctness
        assertEquals(PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());

        assertThrows(NotAvailableException.class, estimator::getLines2D);
        assertThrows(NotAvailableException.class, estimator::getPlanes);
        assertEquals(estimator.getListener(), this);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
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

        // testing constructor with lists
        final var randomizer = new UniformRandomizer();
        final var planes = new ArrayList<Plane>(N_CORRESPONDENCES);
        final var lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        for (var i = 0; i < N_CORRESPONDENCES; i++) {
            planes.add(new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            lines2D.add(new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }

        estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
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

        // Force WrongListSizesException
        final var wrongPlanes = new ArrayList<Plane>();
        final var wrongLines = new ArrayList<Line2D>();
        assertThrows(WrongListSizesException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                wrongPlanes, lines2D));
        assertThrows(WrongListSizesException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                planes, wrongLines));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                null, lines2D));
        assertThrows(IllegalArgumentException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                planes, null));

        // testing constructor with lists and listener
        estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertEquals(this, estimator.getListener());
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

        // Force WrongListSizesException
        assertThrows(WrongListSizesException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                wrongPlanes, lines2D, this));
        assertThrows(WrongListSizesException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                planes, wrongLines, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                null, lines2D, this));
        assertThrows(IllegalArgumentException.class, () -> new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                planes, null, this));
    }

    @Test
    void testIsSetLMSESolutionAllowed() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        assertEquals(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());

        // disable
        estimator.setLMSESolutionAllowed(false);

        // check correctness
        assertFalse(estimator.isLMSESolutionAllowed());

        // enable
        estimator.setLMSESolutionAllowed(true);

        // check correctness
        assertTrue(estimator.isLMSESolutionAllowed());
    }

    @Test
    void testGetSetListsValidityAndAvailability() throws LockedException, WrongListSizesException,
            NotAvailableException {

        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.isReady());
        assertThrows(NotAvailableException.class, estimator::getLines2D);
        assertThrows(NotAvailableException.class, estimator::getPlanes);

        final var randomizer = new UniformRandomizer();
        final var planes = new ArrayList<Plane>(N_CORRESPONDENCES);
        final var lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        for (var i = 0; i < N_CORRESPONDENCES; i++) {
            planes.add(new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            lines2D.add(new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }

        // set lists
        assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

        estimator.setLists(planes, lines2D);

        // check correctness
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertTrue(estimator.areListsAvailable());

        // Force WrongListSizesException
        final var wrongPlanes = new ArrayList<Plane>();
        final var wrongLines = new ArrayList<Line2D>();
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(wrongPlanes, lines2D));
        assertThrows(WrongListSizesException.class, () -> estimator.setLists(wrongPlanes, lines2D));
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, wrongLines));
        assertThrows(WrongListSizesException.class, () -> estimator.setLists(planes, wrongLines));

        // Force IllegalArgumentException
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(null, lines2D));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLists(null, lines2D));
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLists(planes, null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        assertNull(estimator.getListener());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertEquals(this, estimator.getListener());
    }

    @Test
    void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestedVerticalFocalLengthEnabled() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());

        // set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
    }

    @Test
    void testGetSetSuggestedRotationValue() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());

        // set new value
        estimator.setSuggestCenterEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
    }

    @Test
    void testGetSetSuggestedCenterValue() throws LockedException {
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
    void testEstimateNoSuggestion() throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // testing the case where there are more than 4 lines/planes without
            // allowing an LMSE solution
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            Line2D line2D;
            for (var i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }

            var planes = camera.backProjectLines(lines2D);

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // test the case where there are exactly six correspondences
            nCorrespondences = N_CORRESPONDENCES;
            lines2D.clear();
            for (var i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }

            planes = camera.backProjectLines(lines2D);

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            List<Plane> estimatedPlanes;
            Iterator<Plane> planesIt;
            Iterator<Plane> estimatedPlanesIt;
            Plane plane, estimatedPlane;
            PinholeCameraIntrinsicParameters estimatedIntrinsic;
            Rotation3D estimatedRotation;
            MatrixRotation3D estimatedRotation2;
            double estimatedAlphaEuler, estimatedBetaEuler, estimatedGammaEuler;
            boolean validAlphaEuler, validBetaEuler, validGammaEuler;
            Point3D estimatedCameraCenter;
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());
                assertEquals(1, startCount);
                assertEquals(1, endCount);
                assertEquals(0, progressCount);

                // project original points using estimated camera
                estimatedPlanes = estimatedCamera.backProjectLines(lines2D);

                // check that planes and estimated planes are equal
                planesIt = planes.iterator();
                estimatedPlanesIt = estimatedPlanes.iterator();

                while (planesIt.hasNext() && estimatedPlanesIt.hasNext()) {
                    plane = planesIt.next();
                    estimatedPlane = estimatedPlanesIt.next();

                    assertTrue(plane.equals(estimatedPlane, ABSOLUTE_ERROR));
                }

                // decompose estimated camera and check its parameters
                estimatedCamera.decompose();

                // Comparing camera intrinsic parameters
                estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

                assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(skewness, estimatedIntrinsic.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);

                // Comparing estimated rotation
                estimatedRotation = estimatedCamera.getCameraRotation();

                estimatedRotation2 = (MatrixRotation3D) estimatedRotation;
                estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
                estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
                estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();

                if (Math.abs(alphaEuler - estimatedAlphaEuler) <= LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else {
                    validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - Math.PI)
                            <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(betaEuler - estimatedBetaEuler) <= LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else {
                    validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - Math.PI)
                            <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(gammaEuler - estimatedGammaEuler) <= LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else {
                    validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - Math.PI)
                            <= LARGE_ABSOLUTE_ERROR;
                }

                if (!validAlphaEuler || !validBetaEuler || !validGammaEuler) {
                    continue;
                }
                //noinspection all
                assertTrue(validAlphaEuler);
                //noinspection all
                assertTrue(validBetaEuler);
                //noinspection all
                assertTrue(validGammaEuler);

                // comparing estimated camera center
                estimatedCameraCenter = estimatedCamera.getCameraCenter();
                assertTrue(cameraCenter.equals(estimatedCameraCenter,
                        VERY_LARGE_ABSOLUTE_ERROR));
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            estimator.setLMSESolutionAllowed(true);
            nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            lines2D.clear();
            for (var i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }

            planes = camera.backProjectLines(lines2D);

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());
                assertEquals(1, startCount);
                assertEquals(1, endCount);
                assertEquals(0, progressCount);

                // project original points using estimated camera
                estimatedPlanes = estimatedCamera.backProjectLines(lines2D);

                // check that points2D and estimated points2D are equal
                planesIt = planes.iterator();
                estimatedPlanesIt = estimatedPlanes.iterator();

                while (planesIt.hasNext() && estimatedPlanesIt.hasNext()) {
                    plane = planesIt.next();
                    estimatedPlane = estimatedPlanesIt.next();

                    assertTrue(plane.equals(estimatedPlane, LARGE_ABSOLUTE_ERROR));
                }

                // decompose estimated camera and check its parameters
                estimatedCamera.decompose();

                // Comparing camera intrinsic parameters
                estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

                assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                // Comparing estimated rotation
                estimatedRotation = estimatedCamera.getCameraRotation();

                estimatedRotation2 = (MatrixRotation3D) estimatedRotation;
                estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
                estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
                estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();

                if (Math.abs(alphaEuler - estimatedAlphaEuler) > LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - Math.PI)
                            <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(betaEuler - estimatedBetaEuler) > LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - Math.PI)
                            <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(gammaEuler - estimatedGammaEuler) > LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - Math.PI)
                            <= LARGE_ABSOLUTE_ERROR;
                }

                if (!validAlphaEuler) {
                    continue;
                }
                //noinspection all
                assertTrue(validAlphaEuler);
                if (!validBetaEuler) {
                    continue;
                }
                //noinspection all
                assertTrue(validBetaEuler);
                if (!validGammaEuler) {
                    continue;
                }
                //noinspection all
                assertTrue(validGammaEuler);

                // comparing estimated camera center
                estimatedCameraCenter = estimatedCamera.getCameraCenter();
                assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            // Force PinholeCameraEstimatorException
            nCorrespondences = N_CORRESPONDENCES;
            lines2D.clear();
            for (int i = 0; i < nCorrespondences - 2; i++) {
                line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }
            line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            lines2D.add(line2D);
            lines2D.add(line2D);

            planes = camera.backProjectLines(lines2D);

            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);

            reset();
            assertThrows(PinholeCameraEstimatorException.class, estimator::estimate);

            // Force NotReadyException
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();
            assertThrows(NotReadyException.class, estimator::estimate);

            passedAtLeastOnce = true;
            break;
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedSkewness() throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (int i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);

            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();
            final double estimatedSkewness = estimatedIntrinsic.getSkewness();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);

            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedSkewnessNoSuggestion = estimatedIntrinsicNoSuggestion.getSkewness();

            // check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) > Math.abs(skewness - estimatedSkewness)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedHorizontalFocalLengthEnabled() throws WrongListSizesException, LockedException,
            NotReadyException, NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            Line2D line2D;
            for (var i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(horizontalFocalLength);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();
            final var estimatedHorizontalFocalLength = estimatedIntrinsic.getHorizontalFocalLength();

            // estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);

            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();

            // check that horizontal focal length has become closer to suggested
            // value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedVerticalFocalLengthEnabled() throws WrongListSizesException, LockedException,
            NotReadyException, NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(verticalFocalLength);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();
            final var estimatedVerticalFocalLength = estimatedIntrinsic.getVerticalFocalLength();

            // estimate without suggestion
            estimator.setSuggestVerticalFocalLengthEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();

            // check that horizontal focal length has become closer to suggested
            // value
            if (Math.abs(horizontalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >
                    Math.abs(horizontalFocalLength - estimatedVerticalFocalLength)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedAspectRatioEnabled() throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var aspectRatio = intrinsic.getAspectRatio();

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();
            final var estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);

            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedAspectRatioNoSuggestion = estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that aspect ratio has become closer to suggested value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion)
                    > Math.abs(aspectRatio - estimatedAspectRatio)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedPrincipalPointEnabled() throws WrongListSizesException, LockedException,
            NotReadyException, NotAvailableException, CameraException {

        //to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPoint = new InhomogeneousPoint2D(horizontalPrincipalPoint, verticalPrincipalPoint);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            //final normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(horizontalPrincipalPoint,
                    verticalPrincipalPoint));

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();
            final var estimatedPrincipalPoint = new InhomogeneousPoint2D(
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), estimatedIntrinsic.getVerticalPrincipalPoint());

            // estimate without suggestion
            estimator.setSuggestPrincipalPointEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedPrincipalPointNoSuggestion = new InhomogeneousPoint2D(
                    estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                    estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());

            // check that principal point has become closer to suggested value
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion)
                    > principalPoint.distanceTo(estimatedPrincipalPoint)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedRotationEnabled() throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final var gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
            final var q = rotation.toQuaternion();
            q.normalize();

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (int i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

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
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedQNoSuggestion = estimatedCameraNoSuggestion.getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final var diffEstimatedNoSuggestion = Math.pow(q.getA() - estimatedQNoSuggestion.getA(), 2.0)
                    + Math.pow(q.getB() - estimatedQNoSuggestion.getB(), 2.0)
                    + Math.pow(q.getC() - estimatedQNoSuggestion.getC(), 2.0)
                    + Math.pow(q.getD() - estimatedQNoSuggestion.getD(), 2.0);
            final var diffEstimated = Math.pow(q.getA() - estimatedQ.getA(), 2.0)
                    + Math.pow(q.getB() - estimatedQ.getB(), 2.0)
                    + Math.pow(q.getC() - estimatedQ.getC(), 2.0)
                    + Math.pow(q.getD() - estimatedQ.getD(), 2.0);

            if (diffEstimatedNoSuggestion > diffEstimated) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedCenterEnabled() throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

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
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnesssZeroPrincipalPointAndEqualFocalLength() throws WrongListSizesException,
            LockedException, NotReadyException, NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;
            final var principalPoint = new InhomogeneousPoint2D();

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var aspectRatio = intrinsic.getAspectRatio();

            // create rotation parameters
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // instantiate camera
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.areValidLists(planes, lines2D));

            final var estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());
            assertEquals(1, startCount);
            assertEquals(1, endCount);
            assertEquals(0, progressCount);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();
            final var estimatedSkewness = estimatedIntrinsic.getSkewness();
            final var estimatedPrincipalPoint = new InhomogeneousPoint2D(
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), estimatedIntrinsic.getVerticalPrincipalPoint());
            final var estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedSkewnessNoSuggestion = estimatedIntrinsicNoSuggestion.getSkewness();
            final var estimatedPrincipalPointNoSuggestion = new InhomogeneousPoint2D(
                    estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                    estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            final var estimatedAspectRatioNoSuggestion = estimatedIntrinsicNoSuggestion.getAspectRatio();

            // final check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) > Math.abs(skewness - estimatedSkewness))
                    && (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion)
                    > principalPoint.distanceTo(estimatedPrincipalPoint))
                    && (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion)
                    > Math.abs(aspectRatio - estimatedAspectRatio))) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }


    @Override
    public void onEstimateStart(final PinholeCameraEstimator estimator) {
        startCount++;
        checkIsLocked((DLTLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraEstimator estimator) {
        endCount++;
        checkIsLocked((DLTLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final PinholeCameraEstimator estimator, final float progress) {
        progressCount++;
        checkIsLocked((DLTLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    private void reset() {
        startCount = endCount = progressCount = 0;
    }

    private static void checkIsLocked(final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator) {
        assertTrue(estimator.isLocked());
        assertThrows(LockedException.class, () -> estimator.setLMSESolutionAllowed(true));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setLists(null, null));
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
        assertThrows(LockedException.class, () -> estimator.setMinSuggestionWeight(0.0));
        assertThrows(LockedException.class, () -> estimator.setMaxSuggestionWeight(0.0));
        assertThrows(LockedException.class, () -> estimator.setSuggestionWeightStep(0.0));
    }
}
