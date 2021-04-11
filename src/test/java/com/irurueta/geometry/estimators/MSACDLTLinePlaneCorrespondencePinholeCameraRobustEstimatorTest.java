/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimatorTest
        implements PinholeCameraRobustEstimatorListener {

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

    private static final int MIN_SAMPLES = 500;
    private static final int MAX_SAMPLES = 1000;

    private static final double THRESHOLD = 1e-5;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 10;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        assertEquals(estimator.getThreshold(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getLines());
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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


        // test constructor with lines and planes
        final List<Line2D> lines = new ArrayList<>();
        final List<Plane> planes = new ArrayList<>();
        for (int i = 0;
             i < LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES; i++) {
            lines.add(new Line2D());
            planes.add(new Plane());
        }

        estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                planes, lines);

        assertEquals(estimator.getThreshold(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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


        // force IllegalArgumentException
        final List<Plane> planesEmpty = new ArrayList<>();
        final List<Line2D> linesEmpty = new ArrayList<>();
        estimator = null;
        try {
            // not enough points
            estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    planes, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                this);

        assertEquals(estimator.getThreshold(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getPlanes());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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


        // test constructor with listener and samples
        estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                this, planes, lines);

        assertEquals(estimator.getThreshold(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                LinePlaneCorrespondencePinholeCameraRobustEstimator.
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

        // force IllegalArgumentException
        estimator = null;
        try {
            // not enough points
            estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator = new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                    this, planes, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
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
    public void testGetSetConfidence() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 10);

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isResultRefined(),
                PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);

        // set new value
        estimator.setResultRefined(
                !PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);

        // check correctness
        assertEquals(estimator.isResultRefined(),
                !PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isCovarianceKept(),
                PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);

        // set new value
        estimator.setCovarianceKept(
                !PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);

        // check correctness
        assertEquals(estimator.isCovarianceKept(),
                !PinholeCameraRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testIsSetFastRefinementUsed() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isFastRefinementUsed(),
                PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);

        // set new value
        estimator.setFastRefinementUsed(
                !PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);

        // check correctness
        assertEquals(estimator.isFastRefinementUsed(),
                !PinholeCameraRobustEstimator.DEFAULT_USE_FAST_REFINEMENT);
    }

    @Test
    public void testGetSetLinesAndPlanesAndIsReady() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default values
        assertNull(estimator.getPlanes());
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());

        // set new value
        final List<Plane> planes = new ArrayList<>();
        final List<Line2D> lines = new ArrayList<>();
        for (int i = 0;
             i < LinePlaneCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES; i++) {
            planes.add(new Plane());
            lines.add(new Line2D());
        }

        estimator.setLinesAndPlanes(planes, lines);

        // check correctness
        assertSame(estimator.getPlanes(), planes);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Plane> planesEmpty = new ArrayList<>();
        final List<Line2D> linesEmpty = new ArrayList<>();
        try {
            // not enough samples
            estimator.setLinesAndPlanes(planesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        try {
            // different sizes
            estimator.setLinesAndPlanes(planes, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // set new value
        estimator.setSuggestSkewnessValueEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }

    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
    public void testIsSetSuggestHorizontalFocalLengthEnabled()
            throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                !PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue()
            throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // set new value
        estimator.setSuggestAspectRatioEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
    }

    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // set new value
        estimator.setSuggestPrincipalPointEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
    }

    @Test
    public void testGetSetSuggestedPrincipalPointValue()
            throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ROTATION_ENABLED);

        // set new value
        estimator.setSuggestRotationEnabled(
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ROTATION_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestRotationEnabled(),
                !PinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_ROTATION_ENABLED);
    }

    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

        // check default value
        assertEquals(estimator.isSuggestCenterEnabled(),
                LMedSDLTPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);

        // set new value
        estimator.setSuggestCenterEnabled(
                !LMedSDLTPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);

        // check correctness
        assertEquals(estimator.isSuggestCenterEnabled(),
                !LMedSDLTPointCorrespondencePinholeCameraRobustEstimator.
                        DEFAULT_SUGGEST_CENTER_ENABLED);
    }

    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
        final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator();

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
    public void testEstimateWithoutRefinement() throws IllegalArgumentException,
            LockedException, NotReadyException, RobustEstimatorException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera camera2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());
            reset();

            // check correctness of estimation

            // backproject original 2D lines using estimated camera and check
            // distance to 3D planes without error
            Plane originalPlane, estimatedPlane;
            for (int i = 0; i < nSamples; i++) {
                line = lines.get(i);
                originalPlane = planes.get(i);
                estimatedPlane = camera2.backProject(line);

                assertTrue(originalPlane.equals(estimatedPlane,
                        ABSOLUTE_ERROR));
            }

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // compare intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

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
            final Rotation3D estimatedRotation = camera2.getCameraRotation();

            final MatrixRotation3D estimatedRotation2 =
                    (MatrixRotation3D) estimatedRotation;
            final double estimatedAlphaEuler =
                    estimatedRotation2.getAlphaEulerAngle();
            final double estimatedBetaEuler =
                    estimatedRotation2.getBetaEulerAngle();
            final double estimatedGammaEuler =
                    estimatedRotation2.getGammaEulerAngle();
            final boolean validAlphaEuler;
            final boolean validBetaEuler;
            final boolean validGammaEuler;

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
            //noinspection ConstantConditions
            assertTrue(validAlphaEuler);
            //noinspection ConstantConditions
            assertTrue(validBetaEuler);
            //noinspection ConstantConditions
            assertTrue(validGammaEuler);


            // comparing estimated camera center
            final Point3D estimatedCameraCenter = camera2.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws IllegalArgumentException,
            LockedException, NotReadyException, RobustEstimatorException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera camera2 = estimator.estimate();

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

            // backproject original 2D lines using estimated camera and check
            // distance to 3D planes without error
            Plane originalPlane, estimatedPlane;
            boolean failed = false;
            for (int i = 0; i < nSamples; i++) {
                line = lines.get(i);
                originalPlane = planes.get(i);
                estimatedPlane = camera2.backProject(line);

                if (!originalPlane.equals(estimatedPlane, ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(originalPlane.equals(estimatedPlane,
                        ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // compare intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            if (Math.abs(horizontalFocalLength -
                    estimatedIntrinsic.getHorizontalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalFocalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(verticalFocalLength -
                    estimatedIntrinsic.getVerticalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
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
            final Rotation3D estimatedRotation = camera2.getCameraRotation();

            final MatrixRotation3D estimatedRotation2 =
                    (MatrixRotation3D) estimatedRotation;
            final double estimatedAlphaEuler =
                    estimatedRotation2.getAlphaEulerAngle();
            final double estimatedBetaEuler =
                    estimatedRotation2.getBetaEulerAngle();
            final double estimatedGammaEuler =
                    estimatedRotation2.getGammaEulerAngle();
            final boolean validAlphaEuler;
            final boolean validBetaEuler;
            final boolean validGammaEuler;

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


            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);


            // comparing estimated camera center
            final Point3D estimatedCameraCenter = camera2.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    LARGE_ABSOLUTE_ERROR));

            numValid++;

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            RobustEstimatorException, CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera camera2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // check correctness of estimation

            // backproject original 2D lines using estimated camera and check
            // distance to 3D planes without error
            Plane originalPlane, estimatedPlane;
            boolean failed = false;
            for (int i = 0; i < nSamples; i++) {
                line = lines.get(i);
                originalPlane = planes.get(i);
                estimatedPlane = camera2.backProject(line);

                if (!originalPlane.equals(estimatedPlane, ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(originalPlane.equals(estimatedPlane,
                        ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            // decompose estimated camera and check its parameters
            camera2.decompose();

            // compare intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    camera2.getIntrinsicParameters();

            if (Math.abs(horizontalFocalLength -
                    estimatedIntrinsic.getHorizontalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalFocalLength,
                    estimatedIntrinsic.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(verticalFocalLength -
                    estimatedIntrinsic.getVerticalFocalLength()) >
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
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
            final Rotation3D estimatedRotation = camera2.getCameraRotation();

            final MatrixRotation3D estimatedRotation2 =
                    (MatrixRotation3D) estimatedRotation;
            final double estimatedAlphaEuler =
                    estimatedRotation2.getAlphaEulerAngle();
            final double estimatedBetaEuler =
                    estimatedRotation2.getBetaEulerAngle();
            final double estimatedGammaEuler =
                    estimatedRotation2.getGammaEulerAngle();
            final boolean validAlphaEuler;
            final boolean validBetaEuler;
            final boolean validGammaEuler;

            if (Math.abs(alphaEuler - estimatedAlphaEuler) <=
                    10.0 * LARGE_ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else {
                validAlphaEuler = (Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) -
                        Math.PI) <= 10.0 * LARGE_ABSOLUTE_ERROR;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) <=
                    10.0 * LARGE_ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else {
                validBetaEuler = (Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) -
                        Math.PI) <= 10.0 * LARGE_ABSOLUTE_ERROR;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) <=
                    10.0 * LARGE_ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else {
                validGammaEuler = (Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) -
                        Math.PI) <= 10.0 * LARGE_ABSOLUTE_ERROR;
            }


            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);


            // comparing estimated camera center
            final Point3D estimatedCameraCenter = camera2.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter,
                    LARGE_ABSOLUTE_ERROR));

            numValid++;

            if (numCovariances > 0 && numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedSkewness() throws IllegalArgumentException,
            LockedException, NotReadyException, CameraException,
            NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedSkewness = estimatedIntrinsic.getSkewness();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedSkewnessNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getSkewness();

            // check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedSkewnessWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedSkewness = estimatedIntrinsic.getSkewness();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedSkewnessNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getSkewness();

            // check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) {
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
    public void testEstimateSuggestedSkewnessWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNotNull(estimator.getCovariance());
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedSkewness = estimatedIntrinsic.getSkewness();

            // estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedSkewnessNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getSkewness();

            // check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) {
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
    public void testEstimateSuggestedHorizontalFocalLengthEnabled()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedHorizontalFocalLength =
                    estimatedIntrinsic.getHorizontalFocalLength();

            // estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();

            // check that horizontal focal length has become closer to suggested
            // value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedHorizontalFocalLengthWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedHorizontalFocalLength =
                    estimatedIntrinsic.getHorizontalFocalLength();

            // estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();

            // check that horizontal focal length has become closer to suggested
            // value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
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
    public void testEstimateSuggestedHorizontalFocalLengthWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNotNull(estimator.getCovariance());
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedHorizontalFocalLength =
                    estimatedIntrinsic.getHorizontalFocalLength();


            // estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();

            // check that horizontal focal length has become closer to suggested
            // value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
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
    public void testEstimateSuggestedVerticalFocalLengthEnabled()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

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
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();

            // check that vertical focal length has become closer to suggested
            // value
            if (Math.abs(verticalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >=
                    Math.abs(verticalFocalLength - estimatedVerticalFocalLength)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedVerticalFocalLengthWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();

            // check that horizontal focal length has become closer to suggested
            // value
            if (Math.abs(verticalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >=
                    Math.abs(verticalFocalLength - estimatedVerticalFocalLength)) {
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
    public void testEstimateSuggestedVerticalFocalLengthWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
        for (int t = 0; t < 5 * TIMES; t++) {
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            if (estimator.getCovariance() == null) {
                continue;
            }
            assertNotNull(estimator.getCovariance());
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }
            reset();

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
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();

            // check that vertical focal length has become closer to suggested
            // value
            if (Math.abs(horizontalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >=
                    Math.abs(horizontalFocalLength - estimatedVerticalFocalLength)) {
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
    public void testEstimateSuggestedAspectRatioEnabled()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that aspect ratio has become closer to suggested
            // value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedAspectRatioWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that aspect ratio has become closer to suggested
            // value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
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
    public void testEstimateSuggestedAspectRatioWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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

            // decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            // comparing camera intrinsic parameters
            final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            final double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();

            // estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            estimatedCameraNoSuggestion.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            final double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();

            // check that aspect ratio has become closer to suggested
            // value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
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
    public void testEstimateSuggestedPrincipalPointEnabled()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

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
            } catch (final RobustEstimatorException e) {
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
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedPrincipalPointWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            } catch (final RobustEstimatorException e) {
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
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
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
    public void testEstimateSuggestedPrincipalPointWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            // normalize the camera to improve accuracy
            camera.normalize();

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            reset();

            if (estimator.getCovariance() == null) {
                continue;
            }
            assertNotNull(estimator.getCovariance());
            if (estimator.getCovariance() != null) {
                numCovariances++;
            }

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
            } catch (final RobustEstimatorException e) {
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
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numCovariances > 0);
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateSuggestedRotationEnabled()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

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
            } catch (final RobustEstimatorException e) {
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
    public void testEstimateSuggestedRotationWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            } catch (final RobustEstimatorException e) {
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
    public void testEstimateSuggestedRotationWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            } catch (final RobustEstimatorException e) {
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
    public void testEstimateSuggestedCenterEnabled()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
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
    public void testEstimateSuggestedCenterWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
    public void testEstimateSuggestedCenterWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
    public void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLength()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            assertNull(estimator.getCovariance());
            reset();

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

            // estimate without suggestions
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
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

            // check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                            principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                            Math.abs(aspectRatio - estimatedAspectRatio))) {
                numValid++;
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLengthWithRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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

            // estimate without suggestions
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);

            final PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (final RobustEstimatorException e) {
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

            // check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                            principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                            Math.abs(aspectRatio - estimatedAspectRatio))) {
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
    public void testEstimateZeroSkewnessZeroPrincipalPointAndEqualFocalLengthWithFastRefinement()
            throws IllegalArgumentException, LockedException, NotReadyException,
            CameraException, NotAvailableException {
        int numCovariances = 0;
        int numValid = 0;
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

            final int nSamples = randomizer.nextInt(MIN_SAMPLES, MAX_SAMPLES);
            final List<Line2D> lines = new ArrayList<>(nSamples);
            Line2D line;
            for (int i = 0; i < nSamples; i++) {
                line = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
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

            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator =
                    new MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
                            this, planesWithError, lines);

            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setFastRefinementUsed(true);
            estimator.setCovarianceKept(true);
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            final PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
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
            } catch (final RobustEstimatorException e) {
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

            // check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >=
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >=
                            principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >=
                            Math.abs(aspectRatio - estimatedAspectRatio))) {
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
        checkLocked((MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final PinholeCameraRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final PinholeCameraRobustEstimator estimator,
                                        int iteration) {
        estimateNextIteration++;
        checkLocked((MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final PinholeCameraRobustEstimator estimator,
                                         float progress) {
        estimateProgressChange++;
        checkLocked((MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            final MSACDLTLinePlaneCorrespondencePinholeCameraRobustEstimator estimator) {
        final List<Plane> planes = new ArrayList<>();
        final List<Line2D> lines = new ArrayList<>();
        try {
            estimator.setLinesAndPlanes(planes, lines);
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
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
