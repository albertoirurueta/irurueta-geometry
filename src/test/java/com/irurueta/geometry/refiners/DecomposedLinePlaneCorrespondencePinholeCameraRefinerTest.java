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
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class DecomposedLinePlaneCorrespondencePinholeCameraRefinerTest implements RefinerListener<PinholeCamera> {

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

    private static final int MIN_SAMPLES = 500;
    private static final int MAX_SAMPLES = 1000;

    private static final double THRESHOLD = 1e-5;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 50;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int refineStart;
    private int refineEnd;

    private double skewness;
    private double horizontalFocalLength;
    private double verticalFocalLength;
    private double aspectRatio;
    private InhomogeneousPoint2D principalPoint;
    private Quaternion rotation;
    private InhomogeneousPoint3D cameraCenter;

    @Test
    void testConstants() {
        assertFalse(Refiner.DEFAULT_KEEP_COVARIANCE);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(0.0, PinholeCameraRefiner.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(1.0, PinholeCameraRefiner.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertEquals(0.1, DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MIN_SUGGESTION_WEIGHT,
                0.0);
        assertEquals(2.0, DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MAX_SUGGESTION_WEIGHT,
                0.0);
        assertEquals(0.475,
                DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
    }

    @Test
    void testConstructor() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();
        final var camera = estimator.estimate();
        final var inliersData = estimator.getInliersData();
        final var inliers = inliersData.getInliers();
        final var residuals = inliersData.getResiduals();
        final var numInliers = inliersData.getNumInliers();
        final var refinementStandardDeviation = estimator.getThreshold();
        final var samples1 = estimator.getPlanes();
        final var samples2 = estimator.getLines();

        assertNotNull(camera);
        assertNotNull(inliersData);

        // test empty constructor
        var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());
        assertEquals(0, refiner.getTotalSamples());
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        assertFalse(refiner.isSuggestSkewnessValueEnabled());
        assertEquals(0.0, refiner.getSuggestedSkewnessValue(), 0.0);
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, refiner.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, refiner.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertFalse(refiner.isSuggestAspectRatioEnabled());
        assertEquals(1.0, refiner.getSuggestedAspectRatioValue(), 0.0);
        assertFalse(refiner.isSuggestPrincipalPointEnabled());
        assertNull(refiner.getSuggestedPrincipalPointValue());
        assertFalse(refiner.isSuggestRotationEnabled());
        assertNull(refiner.getSuggestedRotationValue());
        assertFalse(refiner.isSuggestCenterEnabled());
        assertNull(refiner.getSuggestedCenterValue());

        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MIN_SUGGESTION_WEIGHT,
                refiner.getMinSuggestionWeight(), 0.0);
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MAX_SUGGESTION_WEIGHT,
                refiner.getMaxSuggestionWeight(), 0.0);
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_SUGGESTION_WEIGHT_STEP,
                refiner.getSuggestionWeightStep(), 0.0);

        // test non-empty constructor
        refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true, inliers,
                residuals, numInliers, samples1, samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples1.size(), refiner.getTotalSamples());
        assertSame(camera, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        assertFalse(refiner.isSuggestSkewnessValueEnabled());
        assertEquals(0.0, refiner.getSuggestedSkewnessValue(), 0.0);
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, refiner.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, refiner.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertFalse(refiner.isSuggestAspectRatioEnabled());
        assertEquals(1.0, refiner.getSuggestedAspectRatioValue(), 0.0);
        assertFalse(refiner.isSuggestPrincipalPointEnabled());
        assertNull(refiner.getSuggestedPrincipalPointValue());
        assertFalse(refiner.isSuggestRotationEnabled());
        assertNull(refiner.getSuggestedRotationValue());
        assertFalse(refiner.isSuggestCenterEnabled());
        assertNull(refiner.getSuggestedCenterValue());

        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MIN_SUGGESTION_WEIGHT,
                refiner.getMinSuggestionWeight(), 0.0);
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MAX_SUGGESTION_WEIGHT,
                refiner.getMaxSuggestionWeight(), 0.0);
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_SUGGESTION_WEIGHT_STEP,
                refiner.getSuggestionWeightStep(), 0.0);
    }

    @Test
    void testGetSetMinSuggestionWeight() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // default value
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MIN_SUGGESTION_WEIGHT,
                refiner.getMinSuggestionWeight(), 0.0);

        // new value
        final var randomizer = new UniformRandomizer();
        final var weight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setMinSuggestionWeight(weight);

        // check correctness
        assertEquals(weight, refiner.getMinSuggestionWeight(), 0.0);
    }

    @Test
    void testGetSetMaxSuggestionWeight() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // default value
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MAX_SUGGESTION_WEIGHT,
                refiner.getMaxSuggestionWeight(), 0.0);

        // new value
        final var randomizer = new UniformRandomizer();
        final var weight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setMaxSuggestionWeight(weight);

        // check correctness
        assertEquals(weight, refiner.getMaxSuggestionWeight(), 0.0);
    }

    @Test
    void testSetMinMaxSuggestionWeight() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // default values
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MIN_SUGGESTION_WEIGHT,
                refiner.getMinSuggestionWeight(), 0.0);
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_MAX_SUGGESTION_WEIGHT,
                refiner.getMaxSuggestionWeight(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var minWeight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var maxWeight = randomizer.nextDouble(minWeight, MAX_RANDOM_VALUE);
        refiner.setMinMaxSuggestionWeight(minWeight, maxWeight);

        // check correctness
        assertEquals(minWeight, refiner.getMinSuggestionWeight(), 0.0);
        assertEquals(maxWeight, refiner.getMaxSuggestionWeight(), 0.0);
    }

    @Test
    void testGetSetSuggestionWeightStep() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // default value
        assertEquals(DecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_SUGGESTION_WEIGHT_STEP,
                refiner.getSuggestionWeightStep(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var step = randomizer.nextDouble(1e-6, MAX_RANDOM_VALUE);
        refiner.setSuggestionWeightStep(step);

        // check correctness
        assertEquals(step, refiner.getSuggestionWeightStep(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> refiner.setSuggestionWeightStep(0.0));
    }

    @Test
    void testGetSetRefinementStandardDeviation() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var refinementStandardDeviation = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
    }

    @Test
    void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestSkewnessValueEnabled());

        // set new value
        refiner.setSuggestSkewnessValueEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestSkewnessValueEnabled());
    }

    @Test
    void testGetSetSuggestedSkewnessValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(0.0, refiner.getSuggestedSkewnessValue(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var s = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        refiner.setSuggestedSkewnessValue(s);

        // check correctness
        assertEquals(s, refiner.getSuggestedSkewnessValue(), 0.0);
    }

    @Test
    void testIsSetSuggestHorizontalFocalLengthEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());

        // set new value
        refiner.setSuggestHorizontalFocalLengthEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestHorizontalFocalLengthEnabled());
    }

    @Test
    void testGetSetSuggestedHorizontalFocalLengthValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(0.0, refiner.getSuggestedHorizontalFocalLengthValue(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var focalLength = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setSuggestedHorizontalFocalLengthValue(focalLength);

        // check correctness
        assertEquals(refiner.getSuggestedHorizontalFocalLengthValue(), focalLength, 0.0);
    }

    @Test
    void testIsSetSuggestVerticalFocalLengthEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());

        // set new value
        refiner.setSuggestVerticalFocalLengthEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestVerticalFocalLengthEnabled());
    }

    @Test
    void testGetSetSuggestedVerticalFocalLengthValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(0.0, refiner.getSuggestedVerticalFocalLengthValue(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var focalLength = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setSuggestedVerticalFocalLengthValue(focalLength);

        // check correctness
        assertEquals(refiner.getSuggestedVerticalFocalLengthValue(), focalLength, 0.0);
    }

    @Test
    void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestAspectRatioEnabled());

        // set new value
        refiner.setSuggestAspectRatioEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestAspectRatioEnabled());
    }

    @Test
    void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(1.0, refiner.getSuggestedAspectRatioValue(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var aRatio = randomizer.nextDouble();
        refiner.setSuggestedAspectRatioValue(aRatio);

        // check correctness
        assertEquals(aRatio, refiner.getSuggestedAspectRatioValue(), 0.0);
    }

    @Test
    void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestPrincipalPointEnabled());

        // set new value
        refiner.setSuggestPrincipalPointEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestPrincipalPointEnabled());
    }

    @Test
    void testGetSetSuggestedPrincipalPointValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSuggestedPrincipalPointValue());

        // set new value
        final var point = new InhomogeneousPoint2D();
        refiner.setSuggestedPrincipalPointValue(point);

        // check correctness
        assertSame(point, refiner.getSuggestedPrincipalPointValue());
    }

    @Test
    void testIsSetSuggestRotationEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestRotationEnabled());

        // set new value
        refiner.setSuggestRotationEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestRotationEnabled());
    }

    @Test
    void testGetSetSuggestedRotationValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSuggestedRotationValue());

        // set new value
        final var q = new Quaternion();
        refiner.setSuggestedRotationValue(q);

        // check correctness
        assertSame(q, refiner.getSuggestedRotationValue());
    }

    @Test
    void testIsSetSuggestCenterEnabled() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestCenterEnabled());

        // set new value
        refiner.setSuggestCenterEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestCenterEnabled());
    }

    @Test
    void testGetSetSuggestedCenterValue() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSuggestedCenterValue());

        // set new value
        final var value = new InhomogeneousPoint3D();
        refiner.setSuggestedCenterValue(value);

        // check correctness
        assertSame(value, refiner.getSuggestedCenterValue());
    }

    @Test
    void testGetSetSamples1() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSamples1());

        // new value
        final var samples1 = new ArrayList<Plane>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(samples1, refiner.getSamples1());
    }

    @Test
    void testGetSetSamples2() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSamples2());

        // new value
        final var samples2 = new ArrayList<Line2D>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(refiner.getSamples2(), samples2);
    }

    @Test
    void testGetSetInliers() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var inliers = inliersData.getInliers();

        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(inliers, refiner.getInliers());
    }

    @Test
    void testGetSetResiduals() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var residuals = inliersData.getResiduals();

        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(residuals, refiner.getResiduals());
    }

    @Test
    void testGetSetNumInliers() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var numInliers = inliersData.getNumInliers();

        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(numInliers, refiner.getNumInliers());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> refiner.setNumInliers(0));
    }

    @Test
    void testSetInliersData() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();

        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(inliersData.getInliers(), refiner.getInliers());
        assertSame(inliersData.getResiduals(), refiner.getResiduals());
        assertEquals(inliersData.getNumInliers(), refiner.getNumInliers());
    }

    @Test
    void testGetSetListener() {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(this, refiner.getListener());
    }

    @Test
    void testGetSetInitialEstimation() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final var camera = new PinholeCamera();
        refiner.setInitialEstimation(camera);

        // check correctness
        assertSame(camera, refiner.getInitialEstimation());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    void testRefineNoSuggestions() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            assertFalse(refiner.refine(result1));
            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            if (refiner.getCovariance() == null) {
                continue;
            }
            assertNotNull(refiner.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedSkewness() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < 10 * TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestSkewnessValueEnabled(true);
            refiner.setSuggestedSkewnessValue(skewness);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertTrue(result1.getInternalMatrix().equals(result2.getInternalMatrix(), ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedHorizontalFocalLength() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < 10 * TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestHorizontalFocalLengthEnabled(true);
            refiner.setSuggestedHorizontalFocalLengthValue(horizontalFocalLength);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedVerticalFocalLength() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestVerticalFocalLengthEnabled(true);
            refiner.setSuggestedVerticalFocalLengthValue(verticalFocalLength);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            if (refiner.getCovariance() == null) {
                continue;
            }
            assertNotNull(refiner.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedAspectRatio() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < 5 * TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestAspectRatioEnabled(true);
            refiner.setSuggestedAspectRatioValue(aspectRatio);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            if (refiner.getCovariance() == null) {
                continue;
            }
            assertNotNull(refiner.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedPrincipalPoint() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < 2 * TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestPrincipalPointEnabled(true);
            refiner.setSuggestedPrincipalPointValue(principalPoint);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedRotation() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestRotationEnabled(true);
            refiner.setSuggestedRotationValue(rotation);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            if (!result1.getInternalMatrix().equals(result2.getInternalMatrix())) {
                continue;
            }
            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedCenter() throws LockedException, NotReadyException, RobustEstimatorException,
            RefinerException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestCenterEnabled(true);
            refiner.setSuggestedCenterValue(cameraCenter);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            if (refiner.getCovariance() == null) {
                continue;
            }
            assertNotNull(refiner.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testRefineSuggestedSkewnessZeroPrincipalPointAndEqualFocalLength() throws LockedException, NotReadyException,
            RobustEstimatorException, RefinerException {
        var numValid = 0;
        for (var t = 0; t < 10 * TIMES; t++) {
            final var estimator = createRobustEstimator2();

            final var camera = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getPlanes();
            final var samples2 = estimator.getLines();

            final var refiner = new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(camera, true,
                    inliersData, samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);
            refiner.setSuggestSkewnessValueEnabled(true);
            refiner.setSuggestedSkewnessValue(skewness);
            refiner.setSuggestPrincipalPointEnabled(true);
            refiner.setSuggestedPrincipalPointValue(principalPoint);
            refiner.setSuggestAspectRatioEnabled(true);
            refiner.setSuggestedAspectRatioValue(aspectRatio);

            final var result1 = new PinholeCamera();

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator createRobustEstimator()
            throws LockedException {

        final var randomizer = new UniformRandomizer();
        horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        principalPoint = new InhomogeneousPoint2D(horizontalPrincipalPoint, verticalPrincipalPoint);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);
        aspectRatio = intrinsic.getAspectRatio();

        // create rotation parameters
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rot = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        rotation = rot.toQuaternion();

        // create camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera = new PinholeCamera(intrinsic, rot, cameraCenter);

        // normalize the camera to improve accuracy
        camera.normalize();

        final var nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
        final var lines = new ArrayList<Line2D>(nSamples);
        for (var i = 0; i < nSamples; i++) {
            final var line = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            lines.add(line);
        }

        final var planes = camera.backProjectLines(lines);

        // create outliers
        final var planesWithError = new ArrayList<Plane>();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (final var plane : planes) {
            Plane planeWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // plane is outlier
                final var errorA = errorRandomizer.nextDouble();
                final var errorB = errorRandomizer.nextDouble();
                final var errorC = errorRandomizer.nextDouble();
                final var errorD = errorRandomizer.nextDouble();
                planeWithError = new Plane(plane.getA() + errorA, plane.getB() + errorB,
                        plane.getC() + errorC, plane.getD() + errorD);
            } else {
                // inlier plane (without error)
                planeWithError = plane;
            }

            planesWithError.add(planeWithError);
        }

        final var estimator = new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(planesWithError, lines);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    private RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator createRobustEstimator2()
            throws LockedException {

        final var randomizer = new UniformRandomizer();
        horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        verticalFocalLength = horizontalFocalLength;
        skewness = 0.0;
        final var horizontalPrincipalPoint = 0.0;
        final var verticalPrincipalPoint = 0.0;
        principalPoint = new InhomogeneousPoint2D(horizontalPrincipalPoint, verticalPrincipalPoint);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);
        aspectRatio = intrinsic.getAspectRatio();

        // create rotation parameters
        final var alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final var betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final var gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final var rot = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        rotation = rot.toQuaternion();

        // create camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        // instantiate camera
        final var camera = new PinholeCamera(intrinsic, rot, cameraCenter);

        // normalize the camera to improve accuracy
        camera.normalize();

        final var nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
        final var lines = new ArrayList<Line2D>(nSamples);
        for (var i = 0; i < nSamples; i++) {
            final var line = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            lines.add(line);
        }

        final var planes = camera.backProjectLines(lines);

        // create outliers
        final var planesWithError = new ArrayList<Plane>();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
        for (final var plane : planes) {
            Plane planeWithError;
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // plane is outlier
                final var errorA = errorRandomizer.nextDouble();
                final var errorB = errorRandomizer.nextDouble();
                final var errorC = errorRandomizer.nextDouble();
                final var errorD = errorRandomizer.nextDouble();
                planeWithError = new Plane(plane.getA() + errorA, plane.getB() + errorB,
                        plane.getC() + errorC, plane.getD() + errorD);
            } else {
                // inlier plane (without error)
                planeWithError = plane;
            }

            planesWithError.add(planeWithError);
        }

        final var estimator = new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(planesWithError, lines);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    @Override
    public void onRefineStart(final Refiner<PinholeCamera> refiner, final PinholeCamera initialEstimation) {
        refineStart++;
        checkLocked((DecomposedLinePlaneCorrespondencePinholeCameraRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<PinholeCamera> refiner, final PinholeCamera initialEstimation,
                            final PinholeCamera result, final boolean errorDecreased) {
        refineEnd++;
        checkLocked((DecomposedLinePlaneCorrespondencePinholeCameraRefiner) refiner);
    }

    private void reset() {
        refineStart = refineEnd = 0;
    }

    private static void checkLocked(final DecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner) {
        assertTrue(refiner.isLocked());
        assertThrows(LockedException.class, () -> refiner.setInitialEstimation(null));
        assertThrows(LockedException.class, () -> refiner.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> refiner.refine(null));
        assertThrows(LockedException.class, refiner::refine);
        assertThrows(LockedException.class, () -> refiner.setInliers(null));
        assertThrows(LockedException.class, () -> refiner.setResiduals(null));
        assertThrows(LockedException.class, () -> refiner.setNumInliers(0));
        assertThrows(LockedException.class, () -> refiner.setInliersData(null));
        assertThrows(LockedException.class, () -> refiner.setSamples1(null));
        assertThrows(LockedException.class, () -> refiner.setSamples2(null));
        assertThrows(LockedException.class, () -> refiner.setRefinementStandardDeviation(0.0));
        assertThrows(LockedException.class, () -> refiner.setMinSuggestionWeight(0.0));
        assertThrows(LockedException.class, () -> refiner.setMaxSuggestionWeight(0.0));
        assertThrows(LockedException.class, () -> refiner.setMinMaxSuggestionWeight(0.0, 0.0));
        assertThrows(LockedException.class, () -> refiner.setSuggestionWeightStep(0.0));
        assertThrows(LockedException.class, () -> refiner.setSuggestSkewnessValueEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedSkewnessValue(0.0));
        assertThrows(LockedException.class, () -> refiner.setSuggestHorizontalFocalLengthEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedHorizontalFocalLengthValue(0.0));
        assertThrows(LockedException.class, () -> refiner.setSuggestVerticalFocalLengthEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedVerticalFocalLengthValue(0.0));
        assertThrows(LockedException.class, () -> refiner.setSuggestAspectRatioEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedAspectRatioValue(0.0));
        assertThrows(LockedException.class, () -> refiner.setSuggestPrincipalPointEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedPrincipalPointValue(null));
        assertThrows(LockedException.class, () -> refiner.setSuggestRotationEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedRotationValue(null));
        assertThrows(LockedException.class, () -> refiner.setSuggestCenterEnabled(true));
        assertThrows(LockedException.class, () -> refiner.setSuggestedCenterValue(null));
    }
}
