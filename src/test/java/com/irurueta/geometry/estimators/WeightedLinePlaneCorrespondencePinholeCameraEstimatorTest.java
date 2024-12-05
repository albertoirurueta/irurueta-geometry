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

class WeightedLinePlaneCorrespondencePinholeCameraEstimatorTest implements PinholeCameraEstimatorListener {

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 2.0;

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

    private static final int TIMES = 10;

    private static final double MIN_WEIGHT_VALUE = 0.75;
    private static final double MAX_WEIGHT_VALUE = 1.0;

    private static final double ERROR_STD = 1e-5;

    private int startCount = 0;
    private int endCount = 0;
    private int progressCount = 0;

    @Test
    void testConstants() {
        assertEquals(PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR,
                PinholeCameraEstimator.DEFAULT_ESTIMATOR_TYPE);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(0.0, PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(1.0, PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertFalse(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertEquals(0.1, PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(2.0, PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(0.475, PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertEquals(50, WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES);
        assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS);
        assertEquals(1e-8, WeightedLinePlaneCorrespondencePinholeCameraEstimator.EPS, 0.0);
    }

    @Test
    void testConstructor() throws WrongListSizesException, NotAvailableException {

        // testing constructor without parameters
        var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertThrows(NotAvailableException.class, estimator::getLines2D);
        assertThrows(NotAvailableException.class, estimator::getPlanes);
        assertThrows(NotAvailableException.class, estimator::getWeights);
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

        // testing constructor with listener
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(this);

        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertThrows(NotAvailableException.class, estimator::getLines2D);
        assertThrows(NotAvailableException.class, estimator::getPlanes);
        assertThrows(NotAvailableException.class, estimator::getWeights);
        assertEquals(this, estimator.getListener());
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

        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(lines2D, estimator.getLines2D());
        assertEquals(planes, estimator.getPlanes());
        assertThrows(NotAvailableException.class, estimator::getWeights);
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
        final var  wrongPlanes = new ArrayList<Plane>();
        final var wrongLines2D = new ArrayList<Line2D>();
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                wrongPlanes, lines2D));
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, wrongLines2D));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                null, lines2D));
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, null));

        // testing constructor with lists and listener
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, this);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(lines2D, estimator.getLines2D());
        assertEquals(planes, estimator.getPlanes());
        assertThrows(NotAvailableException.class, estimator::getWeights);
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
        final var wrongPlanes2 = new ArrayList<Plane>();
        final var wrongLines2D2 = new ArrayList<Line2D>();
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                wrongPlanes2, lines2D, this));
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, wrongLines2D2, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                null, lines2D, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, null, this));

        // testing constructor with lists and weights
        final var weights = new double[N_CORRESPONDENCES];
        randomizer.fill(weights, MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);

        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertArrayEquals(weights, estimator.getWeights(), 0.0);
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
        final var wrongWeights = new double[1];
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                wrongPlanes, lines2D, weights));
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, wrongLines2D, weights));
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, wrongWeights));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                null, lines2D, weights));
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, null, weights));
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, (double[]) null));

        // testing constructor with lists, weights and listener
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights, this);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());
        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(estimator.getLines2D(), lines2D);
        assertEquals(planes, estimator.getPlanes());
        assertArrayEquals(weights, estimator.getWeights(), 0.0);
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
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                wrongPlanes, lines2D, weights, this));
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, wrongLines2D, weights, this));
        assertThrows(WrongListSizesException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, wrongWeights, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                null, lines2D, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, null, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, null, this));
    }

    @Test
    void testGetSetListsValidityAndAvailability() throws LockedException, WrongListSizesException,
            NotAvailableException {

        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
    void testGetSetListsAndWeightsValidityAndAvailability() throws LockedException, IllegalArgumentException,
            WrongListSizesException, NotAvailableException {

        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        // check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertFalse(estimator.isReady());
        assertThrows(NotAvailableException.class, estimator::getLines2D);
        assertThrows(NotAvailableException.class, estimator::getPlanes);
        assertThrows(NotAvailableException.class, estimator::getWeights);

        final var randomizer = new UniformRandomizer();
        final var planes = new ArrayList<Plane>(N_CORRESPONDENCES);
        final var lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        final var weights = new double[N_CORRESPONDENCES];
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
        randomizer.fill(weights, MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);

        // set lists
        assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                weights));

        estimator.setListsAndWeights(planes, lines2D, weights);

        // check correctness
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertArrayEquals(weights, estimator.getWeights(), 0.0);
        assertTrue(estimator.areListsAvailable());
        assertTrue(estimator.areWeightsAvailable());

        // Force WrongListSizesException
        final var wrongPlanes = new ArrayList<Plane>();
        final var wrongLines2D = new ArrayList<Line2D>();
        final var wrongWeights = new double[1];
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(wrongPlanes, lines2D,
                weights));
        assertThrows(WrongListSizesException.class, () -> estimator.setListsAndWeights(wrongPlanes, lines2D, weights));
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, wrongLines2D,
                weights));
        assertThrows(WrongListSizesException.class, () -> estimator.setListsAndWeights(planes, wrongLines2D, weights));
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                wrongWeights));
        assertThrows(WrongListSizesException.class, () -> estimator.setListsAndWeights(planes, lines2D, wrongWeights));

        // Force IllegalArgumentException
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(null, lines2D,
                weights));
        assertThrows(IllegalArgumentException.class, () -> estimator.setListsAndWeights(null, lines2D, weights));
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, null,
                weights));
        assertThrows(IllegalArgumentException.class, () -> estimator.setListsAndWeights(planes, null, weights));
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setListsAndWeights(planes, lines2D, null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        assertNull(estimator.getListener());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertEquals(this, estimator.getListener());
    }

    @Test
    void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestedVerticalFocalLengthEnabled() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());

        // set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
    }

    @Test
    void testGetSetSuggestedRotationValue() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());

        // set new value
        estimator.setSuggestCenterEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
    }

    @Test
    void testGetSetSuggestedCenterValue() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

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
    void testGetSetMaxCorrespondences() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_MAX_CORRESPONDENCES,
                estimator.getMaxCorrespondences());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var maxCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
        estimator.setMaxCorrespondences(maxCorrespondences);

        // check correctness
        assertEquals(maxCorrespondences, estimator.getMaxCorrespondences());
    }

    @Test
    void testIsSetSortWeightsEnabled() throws LockedException {
        final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        assertEquals(WeightedLinePlaneCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());

        // disable
        estimator.setSortWeightsEnabled(false);

        // check correctness
        assertFalse(estimator.isSortWeightsEnabled());

        // enable
        estimator.setSortWeightsEnabled(true);

        // check correctness
        assertTrue(estimator.isSortWeightsEnabled());
    }

    @Test
    void testEstimateNoSuggestion() throws WrongListSizesException, LockedException, NotReadyException,
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

            // test the case where there are exactly 4 correspondences
            var nCorrespondences = N_CORRESPONDENCES;
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            var planes = camera.backProjectLines(lines2D);

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            List<Plane> estimatedPlanes;
            Iterator<Plane> planesIt;
            Iterator<Plane> estimatedPlanesIt;
            Plane plane;
            Plane estimatedPlane;
            PinholeCameraIntrinsicParameters estimatedIntrinsic;
            Rotation3D estimatedRotation;
            MatrixRotation3D estimatedRotation2;
            double estimatedAlphaEuler;
            double estimatedBetaEuler;
            double estimatedGammaEuler;
            boolean validAlphaEuler;
            boolean validBetaEuler;
            boolean validGammaEuler;
            Point3D estimatedCameraCenter;
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());
                assertEquals(1, startCount);
                assertEquals(1, endCount);
                assertEquals(0, progressCount);

                assertNotNull(estimatedCamera);

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

                if (Math.abs(horizontalFocalLength - estimatedIntrinsic.getHorizontalFocalLength())
                        > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(verticalFocalLength - estimatedIntrinsic.getVerticalFocalLength())
                        > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(horizontalPrincipalPoint - estimatedIntrinsic.getHorizontalPrincipalPoint())
                        > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(verticalPrincipalPoint - estimatedIntrinsic.getVerticalPrincipalPoint())
                        > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(skewness - estimatedIntrinsic.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
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

                if (!validAlphaEuler) {
                    continue;
                }
                if (!validBetaEuler) {
                    continue;
                }
                if (!validGammaEuler) {
                    continue;
                }

                // comparing estimated camera center
                estimatedCameraCenter = estimatedCamera.getCameraCenter();
                assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            // Testing the case where there are more than four correspondences
            nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            lines2D.clear();
            weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            planes = camera.backProjectLines(lines2D);

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());
                assertEquals(1, startCount);
                assertEquals(1, endCount);
                assertEquals(0, progressCount);

                assertNotNull(estimatedCamera);

                // project original points using estimated camera
                estimatedPlanes = estimatedCamera.backProjectLines(lines2D);

                // check that points2D and estimated points2D are equal
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

                assertTrue(validAlphaEuler);
                assertTrue(validBetaEuler);
                assertTrue(validGammaEuler);

                // comparing estimated camera center
                estimatedCameraCenter = estimatedCamera.getCameraCenter();
                assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            // Force PinholeCameraEstimatorException
            nCorrespondences = N_CORRESPONDENCES;
            lines2D.clear();
            weights = new double[nCorrespondences];
            for (int i = 0; i < nCorrespondences - 2; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }
            final var line2D = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            lines2D.add(line2D);
            lines2D.add(line2D);

            planes = camera.backProjectLines(lines2D);

            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            reset();
            assertThrows(PinholeCameraEstimatorException.class, estimator::estimate);

            // Force NotReadyException
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
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
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize the camera to improve accuracy
            camera.normalize();

            // Testing the case where there are more than six correspondences and
            // the LMSE solution is allowed
            final var nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, MAX_NUMBER_CORRESPONDENCES);
            final var lines2D = new ArrayList<Line2D>(nCorrespondences);
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);

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
            final var estimatedSkewness = estimatedIntrinsic.getSkewness();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
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
        for (int t = 0; t < TIMES; t++) {
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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(horizontalFocalLength);

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
            final var estimatedHorizontalFocalLength = estimatedIntrinsic.getHorizontalFocalLength();

            // estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
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
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion)
                    > Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
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
        for (var t = 0; t < 2 * TIMES; t++) {
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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(verticalFocalLength);

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
            if (Math.abs(horizontalFocalLength - estimatedVerticalFocalLengthNoSuggestion)
                    > Math.abs(horizontalFocalLength - estimatedVerticalFocalLength)) {
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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

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

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedAspectRatioNoSuggestion = estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that aspect ratio has become closer to suggested
            // value
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

        // to account for random degeneracies
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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(horizontalPrincipalPoint,
                    verticalPrincipalPoint));

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
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

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
    void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLength() throws WrongListSizesException,
            LockedException, NotReadyException, NotAvailableException, CameraException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
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
            final var weights = new double[nCorrespondences];
            for (var i = 0; i < nCorrespondences; i++) {
                final var line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                line2D.normalize();
                lines2D.add(line2D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            final var planes = camera.backProjectLines(lines2D);

            // add error to lines
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            for (final var line : lines2D) {
                final var errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), line.getC() + errorC);
            }

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.areValidListsAndWeights(planes, lines2D,
                    weights));

            final var estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(planes, lines2D, weights,
                    this);

            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

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
            final var estimatedSkewness = estimatedIntrinsic.getSkewness();
            final var estimatedPrincipalPoint = new InhomogeneousPoint2D(
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), estimatedIntrinsic.getVerticalPrincipalPoint());
            final double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final PinholeCameraEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final var estimatedIntrinsicNoSuggestion = estimatedCameraNoSuggestion.getIntrinsicParameters();
            final var estimatedSkewnessNoSuggestion = estimatedIntrinsicNoSuggestion.getSkewness();
            final var estimatedPrincipalPointNoSuggestion = new InhomogeneousPoint2D(
                    estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                    estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            final var estimatedAspectRatioNoSuggestion = estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that intrinsic values have become closer to suggested ones
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
        checkIsLocked((WeightedLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraEstimator estimator) {
        endCount++;
        checkIsLocked((WeightedLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final PinholeCameraEstimator estimator, final float progress) {
        progressCount++;
        checkIsLocked((WeightedLinePlaneCorrespondencePinholeCameraEstimator) estimator);
    }

    private void reset() {
        startCount = endCount = progressCount = 0;
    }

    private static void checkIsLocked(final WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator) {
        assertTrue(estimator.isLocked());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setLists(null, null));
        assertThrows(LockedException.class,
                () -> estimator.setListsAndWeights(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setMaxCorrespondences(0));
        assertThrows(LockedException.class, () -> estimator.setSortWeightsEnabled(true));
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