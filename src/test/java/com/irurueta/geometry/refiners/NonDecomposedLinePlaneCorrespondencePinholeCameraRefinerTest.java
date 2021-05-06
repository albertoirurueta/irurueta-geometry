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
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class NonDecomposedLinePlaneCorrespondencePinholeCameraRefinerTest
        implements RefinerListener<PinholeCamera> {

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

    private static final int TIMES = 100;

    private int mRefineStart;
    private int mRefineEnd;

    @Test
    public void testConstants() {
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(0.0, PinholeCameraRefiner.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                0.0);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(1.0, PinholeCameraRefiner.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                0.0);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertFalse(PinholeCameraRefiner.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertEquals(2.0,
                NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner.DEFAULT_SUGGESTION_ERROR_WEIGHT, 0.0);
    }

    @Test
    public void testConstructor() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                createRobustEstimator();
        final PinholeCamera camera = estimator.estimate();
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();
        final double[] residuals = inliersData.getResiduals();
        final int numInliers = inliersData.getNumInliers();
        final double refinementStandardDeviation = estimator.getThreshold();
        final List<Plane> samples1 = estimator.getPlanes();
        final List<Line2D> samples2 = estimator.getLines();

        assertNotNull(camera);
        assertNotNull(inliersData);

        // test empty constructor
        NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);
        assertEquals(refiner.getTotalSamples(), 0);
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        assertFalse(refiner.isSuggestSkewnessValueEnabled());
        assertEquals(refiner.getSuggestedSkewnessValue(), 0.0, 0.0);
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(refiner.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());
        assertEquals(refiner.getSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertFalse(refiner.isSuggestAspectRatioEnabled());
        assertEquals(refiner.getSuggestedAspectRatioValue(), 1.0, 0.0);
        assertFalse(refiner.isSuggestPrincipalPointEnabled());
        assertNull(refiner.getSuggestedPrincipalPointValue());
        assertFalse(refiner.isSuggestRotationEnabled());
        assertNull(refiner.getSuggestedRotationValue());
        assertFalse(refiner.isSuggestCenterEnabled());
        assertNull(refiner.getSuggestedCenterValue());

        assertEquals(refiner.getSuggestionErrorWeight(),
                NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner.
                        DEFAULT_SUGGESTION_ERROR_WEIGHT, 0.0);

        // test non-empty constructor
        refiner = new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
                camera, true, inliers, residuals, numInliers, samples1,
                samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), camera);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        assertFalse(refiner.isSuggestSkewnessValueEnabled());
        assertEquals(refiner.getSuggestedSkewnessValue(), 0.0, 0.0);
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(refiner.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());
        assertEquals(refiner.getSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertFalse(refiner.isSuggestAspectRatioEnabled());
        assertEquals(refiner.getSuggestedAspectRatioValue(), 1.0, 0.0);
        assertFalse(refiner.isSuggestPrincipalPointEnabled());
        assertNull(refiner.getSuggestedPrincipalPointValue());
        assertFalse(refiner.isSuggestRotationEnabled());
        assertNull(refiner.getSuggestedRotationValue());
        assertFalse(refiner.isSuggestCenterEnabled());
        assertNull(refiner.getSuggestedCenterValue());

        assertEquals(refiner.getSuggestionErrorWeight(),
                NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner.
                        DEFAULT_SUGGESTION_ERROR_WEIGHT, 0.0);

        // test another non-empty constructor
        refiner = new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
                camera, true, inliersData, samples1, samples2,
                refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), camera);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        assertFalse(refiner.isSuggestSkewnessValueEnabled());
        assertEquals(refiner.getSuggestedSkewnessValue(), 0.0, 0.0);
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(refiner.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());
        assertEquals(refiner.getSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertFalse(refiner.isSuggestAspectRatioEnabled());
        assertEquals(refiner.getSuggestedAspectRatioValue(), 1.0, 0.0);
        assertFalse(refiner.isSuggestPrincipalPointEnabled());
        assertNull(refiner.getSuggestedPrincipalPointValue());
        assertFalse(refiner.isSuggestRotationEnabled());
        assertNull(refiner.getSuggestedRotationValue());
        assertFalse(refiner.isSuggestCenterEnabled());
        assertNull(refiner.getSuggestedCenterValue());

        assertEquals(refiner.getSuggestionErrorWeight(),
                NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner.
                        DEFAULT_SUGGESTION_ERROR_WEIGHT, 0.0);
    }

    @Test
    public void testGetSetSuggestionErrorWeight() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(refiner.getSuggestionErrorWeight(),
                NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner.
                        DEFAULT_SUGGESTION_ERROR_WEIGHT, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double suggestionErrorWeight = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        refiner.setSuggestionErrorWeight(suggestionErrorWeight);

        // check correctness
        assertEquals(refiner.getSuggestionErrorWeight(), suggestionErrorWeight,
                0.0);
    }

    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double refinementStandardDeviation = randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
    }

    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestSkewnessValueEnabled());

        // set new value
        refiner.setSuggestSkewnessValueEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestSkewnessValueEnabled());
    }

    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(refiner.getSuggestedSkewnessValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        refiner.setSuggestedSkewnessValue(skewness);

        // check correctness
        assertEquals(refiner.getSuggestedSkewnessValue(), skewness, 0.0);
    }

    @Test
    public void testIsSetSuggestHorizontalFocalLengthEnabled()
            throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestHorizontalFocalLengthEnabled());

        // set new value
        refiner.setSuggestHorizontalFocalLengthEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestHorizontalFocalLengthEnabled());
    }

    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue()
            throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(refiner.getSuggestedHorizontalFocalLengthValue(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double focalLength = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        refiner.setSuggestedHorizontalFocalLengthValue(focalLength);

        // check correctness
        assertEquals(refiner.getSuggestedHorizontalFocalLengthValue(),
                focalLength, 0.0);
    }

    @Test
    public void testIsSetSuggestVerticalFocalLengthEnabled()
            throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestVerticalFocalLengthEnabled());

        // set new value
        refiner.setSuggestVerticalFocalLengthEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestVerticalFocalLengthEnabled());
    }

    @Test
    public void testGetSetSuggestedVerticalFocalLengthValue()
            throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(refiner.getSuggestedVerticalFocalLengthValue(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double focalLength = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        refiner.setSuggestedVerticalFocalLengthValue(focalLength);

        // check correctness
        assertEquals(refiner.getSuggestedVerticalFocalLengthValue(),
                focalLength, 0.0);
    }

    @Test
    public void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestAspectRatioEnabled());

        // set new value
        refiner.setSuggestAspectRatioEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestAspectRatioEnabled());
    }

    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertEquals(refiner.getSuggestedAspectRatioValue(), 1.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double aspectRatio = randomizer.nextDouble();
        refiner.setSuggestedAspectRatioValue(aspectRatio);

        // check correctness
        assertEquals(refiner.getSuggestedAspectRatioValue(), aspectRatio, 0.0);
    }

    @Test
    public void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestPrincipalPointEnabled());

        // set new value
        refiner.setSuggestPrincipalPointEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestPrincipalPointEnabled());
    }

    @Test
    public void testGetSetSuggestedPrincipalPointValue()
            throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSuggestedPrincipalPointValue());

        // set new value
        final InhomogeneousPoint2D point = new InhomogeneousPoint2D();
        refiner.setSuggestedPrincipalPointValue(point);

        // check correctness
        assertSame(refiner.getSuggestedPrincipalPointValue(), point);
    }

    @Test
    public void testIsSetSuggestRotationEnabled() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestRotationEnabled());

        // set new value
        refiner.setSuggestRotationEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestRotationEnabled());
    }

    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSuggestedRotationValue());

        // set new value
        final Quaternion q = new Quaternion();
        refiner.setSuggestedRotationValue(q);

        // check correctness
        assertSame(refiner.getSuggestedRotationValue(), q);
    }

    @Test
    public void testIsSetSuggestCenterEnabled() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertFalse(refiner.isSuggestCenterEnabled());

        // set new value
        refiner.setSuggestCenterEnabled(true);

        // check correctness
        assertTrue(refiner.isSuggestCenterEnabled());
    }

    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSuggestedCenterValue());

        // set new value
        final InhomogeneousPoint3D value = new InhomogeneousPoint3D();
        refiner.setSuggestedCenterValue(value);

        // check correctness
        assertSame(refiner.getSuggestedCenterValue(), value);
    }

    @Test
    public void testGetSetSamples1() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSamples1());

        // new value
        final List<Plane> samples1 = new ArrayList<>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(refiner.getSamples1(), samples1);
    }

    @Test
    public void testGetSetSamples2() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // initial value
        assertNull(refiner.getSamples2());

        // new value
        final List<Line2D> samples2 = new ArrayList<>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(refiner.getSamples2(), samples2);
    }

    @Test
    public void testGetSetInliers() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();

        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(refiner.getInliers(), inliers);
    }

    @Test
    public void testGetSetResiduals() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final double[] residuals = inliersData.getResiduals();

        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(refiner.getResiduals(), residuals);
    }

    @Test
    public void testGetSetNumInliers() throws LockedException,
            NotReadyException, RobustEstimatorException {
        final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final int numInliers = inliersData.getNumInliers();

        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertEquals(refiner.getNumInliers(), 0);

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(refiner.getNumInliers(), numInliers);

        // Force IllegalArgumentException
        try {
            refiner.setNumInliers(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetInliersData() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                createRobustEstimator();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();

        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(refiner.getInliers(), inliersData.getInliers());
        assertSame(refiner.getResiduals(), inliersData.getResiduals());
        assertEquals(refiner.getNumInliers(), inliersData.getNumInliers());
    }

    @Test
    public void testGetSetListener() {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(refiner.getListener(), this);
    }

    @Test
    public void testGetSetInitialEstimation() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final PinholeCamera camera = new PinholeCamera();
        refiner.setInitialEstimation(camera);

        // check correctness
        assertSame(refiner.getInitialEstimation(), camera);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    public void testRefine() throws LockedException, NotReadyException,
            RobustEstimatorException, RefinerException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    createRobustEstimator();

            final PinholeCamera camera = estimator.estimate();
            final InliersData inliersData = estimator.getInliersData();
            final double refinementStandardDeviation = estimator.getThreshold();
            final List<Plane> samples1 = estimator.getPlanes();
            final List<Line2D> samples2 = estimator.getLines();

            final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                    new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
                            camera, true, inliersData, samples1, samples2,
                            refinementStandardDeviation);
            refiner.setListener(this);

            final PinholeCamera result1 = new PinholeCamera();

            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);

            try {
                if (!refiner.refine(result1)) {
                    continue;
                }
            } catch (final Exception e) {
                continue;
            }

            final PinholeCamera result2 = refiner.refine();

            assertEquals(mRefineStart, 2);
            assertEquals(mRefineEnd, 2);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(),
                    result2.getInternalMatrix());

            assertNotNull(refiner.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator createRobustEstimator()
            throws LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
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
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                cameraCenterArray);

        // instantiate camera
        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                cameraCenter);

        // normalize the camera to improve accuracy
        camera.normalize();

        final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
        final List<Line2D> lines = new ArrayList<>(nSamples);
        Line2D line;
        for (int i = 0; i < nSamples; i++) {
            line = new Line2D(randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE));
            lines.add(line);
        }

        final List<Plane> planes = camera.backProjectLines(lines);

        // create outliers
        Plane planeWithError;
        final List<Plane> planesWithError = new ArrayList<>();
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (final Plane plane : planes) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                // plane is outlier
                final double errorA = errorRandomizer.nextDouble();
                final double errorB = errorRandomizer.nextDouble();
                final double errorC = errorRandomizer.nextDouble();
                final double errorD = errorRandomizer.nextDouble();
                planeWithError = new Plane(plane.getA() + errorA,
                        plane.getB() + errorB, plane.getC() + errorC,
                        plane.getD() + errorD);
            } else {
                // inlier plane (without error)
                planeWithError = plane;
            }

            planesWithError.add(planeWithError);
        }

        final RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new RANSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                        planesWithError, lines);

        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    @Override
    public void onRefineStart(final Refiner<PinholeCamera> refiner,
                              final PinholeCamera initialEstimation) {
        mRefineStart++;
        checkLocked(
                (NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<PinholeCamera> refiner,
                            final PinholeCamera initialEstimation,
                            final PinholeCamera result,
                            final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked(
                (NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner) refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(
            final NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner) {
        assertTrue(refiner.isLocked());
        try {
            refiner.setInitialEstimation(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.refine(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.refine();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.setInliers(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setResiduals(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setNumInliers(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setInliersData(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSamples1(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSamples2(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestionErrorWeight(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestSkewnessValueEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedSkewnessValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestHorizontalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedHorizontalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestVerticalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedVerticalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestAspectRatioEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedAspectRatioValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestPrincipalPointEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedPrincipalPointValue(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestRotationEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedRotationValue(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestCenterEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSuggestedCenterValue(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
