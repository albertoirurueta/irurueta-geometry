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

import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class ProjectiveTransformation3DRobustEstimatorTest {

    @Test
    public void testConstants() {
        assertEquals(5, ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, ProjectiveTransformation3DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0, ProjectiveTransformation3DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, ProjectiveTransformation3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, ProjectiveTransformation3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, ProjectiveTransformation3DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, ProjectiveTransformation3DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, ProjectiveTransformation3DRobustEstimator.MIN_ITERATIONS);
        assertTrue(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(ProjectiveTransformation3DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testCreateFromPoints() {
        // create with points and method
        final List<Point3D> inputPoints = new ArrayList<>();
        final List<Point3D> outputPoints = new ArrayList<>();
        for (int i = 0; i < ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        ProjectiveTransformation3DRobustEstimator estimator =
                ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point3D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        final ProjectiveTransformation3DRobustEstimatorListener listener =
                new ProjectiveTransformation3DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(final ProjectiveTransformation3DRobustEstimator estimator,
                                                         final float progress) {
                    }
                };

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final double[] qualityScores = new double[
                ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, points, quality scores and method
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, outputPoints, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                    listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }

    @Test
    public void testFromPlanes() {
        // create with planes and method
        final List<Plane> inputPlanes = new ArrayList<>();
        final List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        ProjectiveTransformation3DRobustEstimator estimator =
                ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Plane> emptyPlanes = new ArrayList<>();

        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes, RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        final ProjectiveTransformation3DRobustEstimatorListener listener =
                new ProjectiveTransformation3DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
                    }
                };

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, outputPlanes, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, planes, quality scores and method
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertTrue(estimator instanceof
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, emptyPlanes, outputPlanes, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, emptyPlanes, qualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, outputPlanes, wrongQualityScores,
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener and points
        estimator = ProjectiveTransformation3DRobustEstimator.
                createFromPlanes(listener, inputPlanes, outputPlanes);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, emptyPlanes, outputPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                    listener, inputPlanes, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
