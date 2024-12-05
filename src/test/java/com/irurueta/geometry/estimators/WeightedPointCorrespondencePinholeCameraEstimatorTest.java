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

import static org.junit.jupiter.api.Assertions.*;

class WeightedPointCorrespondencePinholeCameraEstimatorTest implements PinholeCameraEstimatorListener {

    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;

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

    private static final int INHOM_3D_COORDS = 3;

    private static final int N_POINTS = 6;
    private static final int MIN_POINTS = 7;
    private static final int MAX_POINTS = 100;

    private static final int TIMES = 100;

    private static final double MIN_WEIGHT_VALUE = 0.5;
    private static final double MAX_WEIGHT_VALUE = 1.0;

    private static final double ERROR_STD = 1e-5;

    private int startCount = 0;
    private int endCount = 0;
    private int progressCount = 0;

    @Test
    void testConstants() {
        assertEquals(6, PointCorrespondencePinholeCameraEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES);
        assertTrue(PointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        assertEquals(1e-8, PointCorrespondencePinholeCameraEstimator.EPS, 0.0);
        assertEquals(50, WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS);
        assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS);
    }

    @Test
    void testConstructor() throws IllegalArgumentException, WrongListSizesException, NotAvailableException {

        // testing constructor without parameters
        var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertThrows(NotAvailableException.class, estimator::getPoints2D);
        assertThrows(NotAvailableException.class, estimator::getPoints3D);
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
        estimator = new WeightedPointCorrespondencePinholeCameraEstimator(this);

        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertThrows(NotAvailableException.class, estimator::getPoints2D);
        assertThrows(NotAvailableException.class, estimator::getPoints3D);
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

        estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(points2D, estimator.getPoints2D());
        assertEquals(points3D, estimator.getPoints3D());
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
        final var wrong3D = new ArrayList<Point3D>();
        final var wrong2D = new ArrayList<Point2D>();
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(wrong3D,
                points2D));
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, wrong2D));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                null, points2D));
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, null));

        // testing constructor with lists and listener
        estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, this);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(points2D, estimator.getPoints2D());
        assertEquals(points3D, estimator.getPoints3D());
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
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                wrong3D, points2D, this));
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, wrong2D, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                null, points2D, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, null, this));

        // testing constructor with lists and weights
        final var weights = new double[N_POINTS];
        randomizer.fill(weights, MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);

        estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());
        assertEquals(points2D, estimator.getPoints2D());
        assertEquals(points3D, estimator.getPoints3D());
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
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                wrong3D, points2D, weights));
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, wrong2D, weights));
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, points2D, wrongWeights));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                null, points2D));
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, null));
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, points2D, (double[]) null));

        // testing constructor with lists, weights and listener
        estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights, this);
        // check correctness
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR, estimator.getType());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());
        assertEquals(points2D, estimator.getPoints2D());
        assertEquals(points3D, estimator.getPoints3D());
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
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                wrong3D, points2D, weights, this));
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, wrong2D, weights, this));
        assertThrows(WrongListSizesException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, points2D, wrongWeights, this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                null, points2D, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, null, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedPointCorrespondencePinholeCameraEstimator(
                points3D, points2D, null, this));
    }

    @Test
    void testAreSetPointCorrespondencesNormalized() throws LockedException {

        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        assertEquals(DLTPointCorrespondencePinholeCameraEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointCorrespondencesNormalized());

        // disable
        estimator.setPointCorrespondencesNormalized(false);

        // check correctness
        assertFalse(estimator.arePointCorrespondencesNormalized());

        // enable
        estimator.setPointCorrespondencesNormalized(true);

        // check correctness
        assertTrue(estimator.arePointCorrespondencesNormalized());
    }

    @Test
    void testGetSetListsValidityAndAvailability() throws LockedException, IllegalArgumentException,
            WrongListSizesException, NotAvailableException {

        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        assertTrue(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, points2D));

        estimator.setLists(points3D, points2D);

        // check correctness
        assertEquals(points3D, estimator.getPoints3D());
        assertEquals(points2D, estimator.getPoints2D());
        assertTrue(estimator.areListsAvailable());

        // Force WrongListSizesException
        final var wrong3D = new ArrayList<Point3D>();
        final var wrong2D = new ArrayList<Point2D>();
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidLists(wrong3D, points2D));
        assertThrows(WrongListSizesException.class, () -> estimator.setLists(wrong3D, points2D));
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, wrong2D));
        assertThrows(WrongListSizesException.class, () -> estimator.setLists(points3D, wrong2D));

        // Force IllegalArgumentException
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(null, points2D));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLists(null, points2D));
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(points3D, null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLists(points3D, null));
    }

    @Test
    void testGetSetListsAndWeightsValidityAndAvailability() throws LockedException, IllegalArgumentException,
            WrongListSizesException, NotAvailableException {

        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        // check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertFalse(estimator.isReady());
        assertThrows(NotAvailableException.class, estimator::getPoints2D);
        assertThrows(NotAvailableException.class, estimator::getPoints3D);
        assertThrows(NotAvailableException.class, estimator::getWeights);

        final var randomizer = new UniformRandomizer();
        final var points3D = new ArrayList<Point3D>(N_POINTS);
        final var points2D = new ArrayList<Point2D>(N_POINTS);
        final var weights = new double[N_POINTS];
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
        randomizer.fill(weights, MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);

        // set lists
        assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                weights));

        estimator.setListsAndWeights(points3D, points2D, weights);

        // check correctness
        assertEquals(points3D, estimator.getPoints3D());
        assertEquals(points2D, estimator.getPoints2D());
        assertArrayEquals(weights, estimator.getWeights(), 0.0);
        assertTrue(estimator.areListsAvailable());
        assertTrue(estimator.areWeightsAvailable());

        // Force WrongListSizesException
        final var wrong3D = new ArrayList<Point3D>();
        final var wrong2D = new ArrayList<Point2D>();
        final var wrongWeights = new double[1];
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(wrong3D, points2D,
                weights));
        assertThrows(WrongListSizesException.class, () -> estimator.setListsAndWeights(wrong3D, points2D, weights));
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, wrong2D,
                weights));
        assertThrows(WrongListSizesException.class, () -> estimator.setListsAndWeights(points3D, wrong2D, weights));
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                wrongWeights));
        assertThrows(WrongListSizesException.class, () -> estimator.setListsAndWeights(points3D, points2D,
                wrongWeights));

        // Force IllegalArgumentException
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(null, points2D,
                weights));
        assertThrows(IllegalArgumentException.class, () -> estimator.setListsAndWeights(null, points2D,
                weights));
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, null,
                weights));
        assertThrows(IllegalArgumentException.class, () -> estimator.setListsAndWeights(points3D, null,
                weights));
        assertFalse(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setListsAndWeights(points3D, points2D,
                null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        assertNull(estimator.getListener());

        // set listener
        estimator.setListener(this);

        // check correctness
        assertEquals(this, estimator.getListener());
    }

    @Test
    void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestedVerticalFocalLengthEnabled() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);

        // set new value
        estimator.setSuggestedVerticalFocalLengthValue(100.0);

        // check correctness
        assertEquals(100.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
    }

    @Test
    void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());

        // set new value
        estimator.setSuggestRotationEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED, estimator.isSuggestRotationEnabled());
    }

    @Test
    void testGetSetSuggestedRotationValue() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        // check default value
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());

        // set new value
        estimator.setSuggestCenterEnabled(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(!PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED, estimator.isSuggestCenterEnabled());
    }

    @Test
    void testGetSetSuggestedCenterValue() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

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
    void testGetSetMaxPoints() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var maxPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        estimator.setMaxPoints(maxPoints);

        // check correctness
        assertEquals(maxPoints, estimator.getMaxPoints());
    }

    @Test
    void testIsSetSortWeightsEnabled() throws LockedException {
        final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator();

        assertEquals(WeightedPointCorrespondencePinholeCameraEstimator.DEFAULT_SORT_WEIGHTS,
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
    void testEstimateNoSuggestion() throws WrongListSizesException, LockedException, NotReadyException, CameraException,
            NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing case where there are exactly six points
            var nPoints = N_POINTS;
            final var points3D = new ArrayList<Point3D>(nPoints);
            var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            var points2D = camera.project(points3D);

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            var  estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
                    this);

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

            // project original points using estimated camera
            var estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            var points2DIt = points2D.iterator();
            var estimatedPoints2DIt = estimatedPoints2D.iterator();

            var validPoints = true;
            while (points2DIt.hasNext() && estimatedPoints2DIt.hasNext()) {
                final var point2D = points2DIt.next();
                final var estimatedPoint2D = estimatedPoints2DIt.next();

                if (!point2D.equals(estimatedPoint2D, LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(point2D.equals(estimatedPoint2D, LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            var estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);

            // Comparing estimated rotation
            var estimatedRotation = estimatedCamera.getCameraRotation();

            var estimatedRotation2 = (MatrixRotation3D) estimatedRotation;
            var estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
            var estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
            var  estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();
            boolean validAlphaEuler;
            boolean validBetaEuler;
            boolean validGammaEuler;

            if (Math.abs(alphaEuler - estimatedAlphaEuler) <= LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else {
                validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - Math.PI)
                        <= LARGE_ABSOLUTE_ERROR;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) <= LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else {
                validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - Math.PI) <= LARGE_ABSOLUTE_ERROR;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) <= LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else {
                validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - Math.PI)
                        <= LARGE_ABSOLUTE_ERROR;
            }

            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);

            // comparing estimated camera center
            var estimatedCameraCenter = estimatedCamera.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));

            // testing the case where there are more than six points
            nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            points3D.clear();
            weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
            }

            points2D = camera.project(points3D);

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
                    this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
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

            // project original points using estimated camera
            estimatedPoints2D = estimatedCamera.project(points3D);

            // check that points2D and estimated points2D are equal
            points2DIt = points2D.iterator();
            estimatedPoints2DIt = estimatedPoints2D.iterator();

            while (points2DIt.hasNext() && estimatedPoints2DIt.hasNext()) {
                final var point2D = points2DIt.next();
                final var estimatedPoint2D = estimatedPoints2DIt.next();

                assertTrue(point2D.equals(estimatedPoint2D, VERY_LARGE_ABSOLUTE_ERROR));
            }

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // Comparing camera intrinsic parameters
            estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
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

            if (Math.abs(alphaEuler - estimatedAlphaEuler) > VERY_LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - Math.PI)
                        <= VERY_LARGE_ABSOLUTE_ERROR;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) > VERY_LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - Math.PI)
                        <= VERY_LARGE_ABSOLUTE_ERROR;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) > VERY_LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - Math.PI)
                        <= VERY_LARGE_ABSOLUTE_ERROR;
            }

            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);

            // comparing estimated camera center
            estimatedCameraCenter = estimatedCamera.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter, VERY_LARGE_ABSOLUTE_ERROR));
            passedAtLeastOnce = true;
            break;
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    void testEstimateSuggestedSkewnessEnabled() throws WrongListSizesException, LockedException, NotReadyException,
            CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
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
            NotReadyException, CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
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
            NotReadyException, CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < 2 * TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing case where there are exactly six points
            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
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

            // check that horizontal focal length has become closer to suggested value
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
            CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var aspectRatio = intrinsic.getAspectRatio();

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
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
            NotReadyException, CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPoint = new InhomogeneousPoint2D(horizontalPrincipalPoint, verticalPrincipalPoint);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
                    this);
            estimator.setSuggestAspectRatioEnabled(true);
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
            CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
                    this);
            estimator.setSuggestAspectRatioEnabled(true);
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
            CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
                    this);
            estimator.setSuggestAspectRatioEnabled(true);
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
            LockedException, NotReadyException, CameraException, NotAvailableException {

        // to account for random degeneracies
        var passedAtLeastOnce = false;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;
            final var principalPoint = new InhomogeneousPoint2D();

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var aspectRatio = intrinsic.getAspectRatio();

            // create rotation
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

            // normalize camera to improve precision
            camera.normalize();

            // testing the case where there are more than six points
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var points3D = new ArrayList<Point3D>(nPoints);
            final var weights = new double[nPoints];
            for (var i = 0; i < nPoints; i++) {
                final var point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                points3D.add(point3D);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
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

            assertTrue(WeightedPointCorrespondencePinholeCameraEstimator.areValidListsAndWeights(points3D, points2D,
                    weights));

            final var estimator = new WeightedPointCorrespondencePinholeCameraEstimator(points3D, points2D, weights,
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
            final var estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestions
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
        checkIsLocked((WeightedPointCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraEstimator estimator) {
        endCount++;
        checkIsLocked((WeightedPointCorrespondencePinholeCameraEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final PinholeCameraEstimator estimator, final float progress) {
        progressCount++;
        checkIsLocked((WeightedPointCorrespondencePinholeCameraEstimator) estimator);
    }

    private void reset() {
        startCount = endCount = progressCount = 0;
    }

    private static void checkIsLocked(final WeightedPointCorrespondencePinholeCameraEstimator estimator) {
        assertTrue(estimator.isLocked());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setLists(null, null));
        assertThrows(LockedException.class, () -> estimator.setListsAndWeights(null, null,
                null));
        assertThrows(LockedException.class, () -> estimator.setMaxPoints(0));
        assertThrows(LockedException.class, () -> estimator.setPointCorrespondencesNormalized(true));
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