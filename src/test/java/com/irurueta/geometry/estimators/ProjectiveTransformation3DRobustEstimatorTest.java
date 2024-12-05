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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class ProjectiveTransformation3DRobustEstimatorTest {

    @Test
    void testConstants() {
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
    void testCreateFromPoints() {
        // create with points and method
        final var inputPoints = new ArrayList<Point3D>();
        final var outputPoints = new ArrayList<Point3D>();
        for (var i = 0; i < ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }

        var estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                emptyPoints, outputPoints, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, emptyPoints, RobustEstimatorMethod.LMEDS));

        // test with listener and points
        final var listener = new ProjectiveTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final var qualityScores = new double[ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, points, quality scores and method
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, emptyPoints, outputPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, emptyPoints, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, outputPoints, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                inputPoints, emptyPoints));

        // test with listener and points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, emptyPoints, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPoints(
                listener, inputPoints, emptyPoints));

        // test with points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(inputPoints, outputPoints,
                qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPoints(listener, inputPoints, outputPoints,
                qualityScores);
        assertInstanceOf(PROMedSPointCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }

    @Test
    void testFromPlanes() {
        // create with planes and method
        final var inputPlanes = new ArrayList<Plane>();
        final var outputPlanes = new ArrayList<Plane>();
        for (var i = 0; i < ProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }

        var estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final var emptyPlanes = new ArrayList<Plane>();
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                emptyPlanes, outputPlanes, RobustEstimatorMethod.LMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, emptyPlanes, RobustEstimatorMethod.LMEDS));

        // test with listener and points
        final var listener = new ProjectiveTransformation3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final ProjectiveTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final ProjectiveTransformation3DRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(
                    final ProjectiveTransformation3DRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final ProjectiveTransformation3DRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        final var qualityScores = new double[PointCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        final var wrongQualityScores = new double[1];

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                emptyPlanes, outputPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, emptyPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, outputPlanes, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with listener, planes, quality scores and method
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.RANSAC);
        assertInstanceOf(RANSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.LMEDS);
        assertInstanceOf(LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.MSAC);
        assertInstanceOf(MSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROSAC);
        assertInstanceOf(PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                qualityScores, RobustEstimatorMethod.PROMEDS);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, emptyPlanes, outputPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, emptyPlanes, qualityScores, RobustEstimatorMethod.PROMEDS));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, outputPlanes, wrongQualityScores, RobustEstimatorMethod.PROMEDS));

        // test with points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                emptyPlanes, outputPlanes));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                inputPlanes, emptyPlanes));

        // test with listener and points
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, emptyPlanes, outputPlanes));
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3DRobustEstimator.createFromPlanes(
                listener, inputPlanes, emptyPlanes));

        // test with points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(inputPlanes, outputPlanes,
                qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = ProjectiveTransformation3DRobustEstimator.createFromPlanes(listener, inputPlanes, outputPlanes,
                qualityScores);
        assertInstanceOf(PROMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.class, estimator);
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
