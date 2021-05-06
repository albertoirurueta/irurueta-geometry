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
import org.junit.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class DLTLinePlaneCorrespondencePinholeCameraEstimatorTest implements
        PinholeCameraEstimatorListener {

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
    public void testConstants() {
        assertEquals(11, DLTLinePlaneCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_EQUATIONS);
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(1e-8, DLTLinePlaneCorrespondencePinholeCameraEstimator.EPS, 0.0);
    }

    @Test
    public void testConstructor() throws WrongListSizesException,
            NotAvailableException {

        // testing constructor without parameters
        DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                        DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(),
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);

        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPlanes();
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


        // testing constructor with listener
        estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(this);

        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                        DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(),
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);

        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPlanes();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        assertEquals(estimator.getListener(), this);
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


        // testing constructor with lists
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final List<Plane> planes = new ArrayList<>(N_CORRESPONDENCES);
        final List<Line2D> lines2D = new ArrayList<>(N_CORRESPONDENCES);
        for (int i = 0; i < N_CORRESPONDENCES; i++) {
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

        estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                lines2D);
        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                        DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(),
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.getPlanes(), planes);
        assertEquals(estimator.getLines2D(), lines2D);
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

        // Force WrongListSizesException
        final List<Plane> wrongPlanes = new ArrayList<>();
        final List<Line2D> wrongLines = new ArrayList<>();
        estimator = null;
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    wrongPlanes, lines2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, wrongLines);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    null, lines2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // testing constructor with lists and listener
        estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                lines2D, this);
        // check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                        DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(),
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.getPlanes(), planes);
        assertEquals(estimator.getLines2D(), lines2D);
        assertEquals(estimator.getListener(), this);
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

        // Force WrongListSizesException
        estimator = null;
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    wrongPlanes, lines2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, wrongLines, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    null, lines2D, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException {
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        assertEquals(estimator.isLMSESolutionAllowed(),
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);

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
    public void testGetSetListsValidityAndAvailability() throws LockedException,
            WrongListSizesException, NotAvailableException {

        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        // check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.isReady());
        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
        try {
            estimator.getPlanes();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final List<Plane> planes = new ArrayList<>(N_CORRESPONDENCES);
        final List<Line2D> lines2D = new ArrayList<>(N_CORRESPONDENCES);
        for (int i = 0; i < N_CORRESPONDENCES; i++) {
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
        assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(planes, lines2D));

        estimator.setLists(planes, lines2D);

        // check correctness
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertTrue(estimator.areListsAvailable());

        // Force WrongListSizesException
        final List<Plane> wrongPlanes = new ArrayList<>();
        final List<Line2D> wrongLines = new ArrayList<>();
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(wrongPlanes, lines2D));
        try {
            estimator.setLists(wrongPlanes, lines2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(planes, wrongLines));
        try {
            estimator.setLists(planes, wrongLines);
            fail("WrongListSizesException expected but not thrown");
        } catch (final WrongListSizesException ignore) {
        }

        // Force IllegalArgumentException
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(null, lines2D));
        try {
            estimator.setLists(null, lines2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(planes, null));
        try {
            estimator.setLists(planes, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

        assertNull(estimator.getListener());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertEquals(estimator.getListener(), this);
    }

    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetSuggestionWeightStep() throws LockedException {
        final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();

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
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testEstimateNoSuggestion() throws WrongListSizesException,
            LockedException, NotReadyException, PinholeCameraEstimatorException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // testing the case where there are more than 4 lines/planes without
            // allowing an LMSE solution
            int nCorrespondences = randomizer.nextInt(
                    MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }

            List<Plane> planes = camera.backProjectLines(lines2D);

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);

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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);


            // test the case where there are exactly six correspondences
            nCorrespondences = N_CORRESPONDENCES;
            lines2D.clear();
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }

            planes = camera.backProjectLines(lines2D);

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, this);

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
                assertEquals(startCount, 1);
                assertEquals(endCount, 1);
                assertEquals(progressCount, 0);

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

                assertEquals(horizontalFocalLength,
                        estimatedIntrinsic.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength,
                        estimatedIntrinsic.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint,
                        estimatedIntrinsic.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint,
                        estimatedIntrinsic.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                        VERY_LARGE_ABSOLUTE_ERROR);


                // Comparing estimated rotation
                estimatedRotation = estimatedCamera.getCameraRotation();

                estimatedRotation2 = (MatrixRotation3D) estimatedRotation;
                estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
                estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
                estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();

                if (Math.abs(alphaEuler - estimatedAlphaEuler) <=
                        LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else {
                    validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) -
                            Math.PI) <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(betaEuler - estimatedBetaEuler) <=
                        LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else {
                    validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) -
                            Math.PI) <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(gammaEuler - estimatedGammaEuler) <=
                        LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else {
                    validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) -
                            Math.PI) <= LARGE_ABSOLUTE_ERROR;
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
            nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            lines2D.clear();
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }

            planes = camera.backProjectLines(lines2D);

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());
                assertEquals(startCount, 1);
                assertEquals(endCount, 1);
                assertEquals(progressCount, 0);

                // project original points using estimated camera
                estimatedPlanes = estimatedCamera.backProjectLines(lines2D);

                // check that points2D and estimated points2D are equal
                planesIt = planes.iterator();
                estimatedPlanesIt = estimatedPlanes.iterator();

                while (planesIt.hasNext() && estimatedPlanesIt.hasNext()) {
                    plane = planesIt.next();
                    estimatedPlane = estimatedPlanesIt.next();

                    assertTrue(plane.equals(estimatedPlane,
                            LARGE_ABSOLUTE_ERROR));
                }

                // decompose estimated camera and check its parameters
                estimatedCamera.decompose();

                // Comparing camera intrinsic parameters
                estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

                assertEquals(horizontalFocalLength,
                        estimatedIntrinsic.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength,
                        estimatedIntrinsic.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint,
                        estimatedIntrinsic.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint,
                        estimatedIntrinsic.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                        VERY_LARGE_ABSOLUTE_ERROR);


                // Comparing estimated rotation
                estimatedRotation = estimatedCamera.getCameraRotation();

                estimatedRotation2 = (MatrixRotation3D) estimatedRotation;
                estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
                estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
                estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();

                if (Math.abs(alphaEuler - estimatedAlphaEuler) <=
                        LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else {
                    validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) -
                            Math.PI) <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(betaEuler - estimatedBetaEuler) <=
                        LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else {
                    validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) -
                            Math.PI) <= LARGE_ABSOLUTE_ERROR;
                }

                if (Math.abs(gammaEuler - estimatedGammaEuler) <=
                        LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else {
                    validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) -
                            Math.PI) <= LARGE_ABSOLUTE_ERROR;
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
                assertTrue(cameraCenter.equals(estimatedCameraCenter,
                        VERY_LARGE_ABSOLUTE_ERROR));
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            // Force PinholeCameraEstimatorException
            nCorrespondences = N_CORRESPONDENCES;
            lines2D.clear();
            for (int i = 0; i < nCorrespondences - 2; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                lines2D.add(line2D);
            }
            line2D = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE));
            lines2D.add(line2D);
            lines2D.add(line2D);

            planes = camera.backProjectLines(lines2D);

            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, this);

            reset();
            estimatedCamera = null;
            try {
                estimatedCamera = estimator.estimate();
                fail("PinholeCameraEstimatorException expected but not thrown");
            } catch (final PinholeCameraEstimatorException ignore) {
            }

            // Force NotReadyException
            estimator = new DLTLinePlaneCorrespondencePinholeCameraEstimator();
            try {
                estimatedCamera = estimator.estimate();
                fail("NotReadyException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }
            assertNull(estimatedCamera);

            passedAtLeastOnce = true;
            break;
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    public void testEstimateSuggestedSkewness() throws WrongListSizesException,
            LockedException, NotReadyException, NotAvailableException,
            CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);

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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
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

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedSkewnessNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getSkewness();

            // check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >
                    Math.abs(skewness - estimatedSkewness)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    public void testEstimateSuggestedHorizontalFocalLengthEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(
                    MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedHorizontalFocalLength =
                    estimatedIntrinsic.getHorizontalFocalLength();

            // estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);

            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedHorizontalFocalLengthNoSuggestion =
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
    public void testEstimateSuggestedVerticalFocalLengthEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedVerticalFocalLength =
                    estimatedIntrinsic.getVerticalFocalLength();

            // estimate without suggestion
            estimator.setSuggestVerticalFocalLengthEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedVerticalFocalLengthNoSuggestion =
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
    public void testEstimateSuggestedAspectRatioEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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

            final double aspectRatio = intrinsic.getAspectRatio();

            // create rotation parameters
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);

            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that aspect ratio has become closer to suggested
            // value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    public void testEstimateSuggestedPrincipalPointEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        //to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            //final normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final InhomogeneousPoint2D estimatedPrincipalPoint =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(),
                            estimatedIntrinsic.getVerticalPrincipalPoint());

            // estimate without suggestion
            estimator.setSuggestPrincipalPointEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());

            // check that principal point has become closer to suggested value
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    public void testEstimateSuggestedRotationEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);
            final Quaternion q = rotation.toQuaternion();
            q.normalize();

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

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
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final Quaternion estimatedQNoSuggestion = estimatedCameraNoSuggestion.
                    getCameraRotation().toQuaternion();

            // check that rotation has become closer to suggested value
            final double diffEstimatedNoSuggestion =
                    Math.pow(q.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                            Math.pow(q.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                            Math.pow(q.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                            Math.pow(q.getD() - estimatedQNoSuggestion.getD(), 2.0);
            final double diffEstimated =
                    Math.pow(q.getA() - estimatedQ.getA(), 2.0) +
                            Math.pow(q.getB() - estimatedQ.getB(), 2.0) +
                            Math.pow(q.getC() - estimatedQ.getC(), 2.0) +
                            Math.pow(q.getD() - estimatedQ.getD(), 2.0);

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
    public void testEstimateSuggestedCenterEnabled()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
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
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

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
                passedAtLeastOnce = true;
            }

            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnesssZeroPrincipalPointAndEqualFocalLength()
            throws WrongListSizesException, LockedException, NotReadyException,
            NotAvailableException, CameraException {

        // to account for random degeneracies
        boolean passedAtLeastOnce = false;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;
            final InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final double aspectRatio = intrinsic.getAspectRatio();

            // create rotation parameters
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            // instantiate camera
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                    cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES,
                    MAX_NUMBER_CORRESPONDENCES);
            final List<Line2D> lines2D = new ArrayList<>(nCorrespondences);
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);
            }

            final List<Plane> planes = camera.backProjectLines(lines2D);

            // add error to lines
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (final Line2D line : lines2D) {
                final double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(),
                        line.getC() + errorC);
            }

            assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidLists(planes, lines2D));

            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D, this);
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
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            assertNotNull(estimatedCamera);

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedSkewness = estimatedIntrinsic.getSkewness();
            final InhomogeneousPoint2D estimatedPrincipalPoint =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(),
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            final double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

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

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedSkewnessNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getSkewness();
            final InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            final double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();

            // final check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >
                            principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >
                            Math.abs(aspectRatio - estimatedAspectRatio))) {
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
        checkIsLocked(
                (DLTLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraEstimator estimator) {
        endCount++;
        checkIsLocked(
                (DLTLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final PinholeCameraEstimator estimator,
                                           final float progress) {
        progressCount++;
        checkIsLocked(
                (DLTLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    private void reset() {
        startCount = endCount = progressCount = 0;
    }

    private void checkIsLocked(
            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator) {
        assertTrue(estimator.isLocked());
        try {
            estimator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }

        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }

        try {
            estimator.setLists(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Throwable t) {
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
            estimator.setMinSuggestionWeight(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxSuggestionWeight(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSuggestionWeightStep(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
