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

import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class PointCorrespondenceProjectiveTransformation2DRobustEstimatorTest {

    @Test
    public void testConstants() {
        assertEquals(4, ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE);
        assertEquals(0.05f, ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0f);
        assertEquals(0.0f, ProjectiveTransformation2DRobustEstimator.MIN_PROGRESS_DELTA, 0.0f);
        assertEquals(1.0f, ProjectiveTransformation2DRobustEstimator.MAX_PROGRESS_DELTA, 0.0f);
        assertEquals(0.99, ProjectiveTransformation2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(5000, ProjectiveTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(0.0, ProjectiveTransformation2DRobustEstimator.MIN_CONFIDENCE, 0.0);
        assertEquals(1.0, ProjectiveTransformation2DRobustEstimator.MAX_CONFIDENCE, 0.0);
        assertEquals(1, ProjectiveTransformation2DRobustEstimator.MIN_ITERATIONS);
        assertTrue(ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(ProjectiveTransformation2DRobustEstimator.DEFAULT_KEEP_COVARIANCE);
        assertEquals(RobustEstimatorMethod.PROMedS,
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Test
    public void testCreate() {
        // create with robust estimator method
        PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        create(RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with points and method
        final List<Point2D> inputPoints = new ArrayList<>();
        final List<Point2D> outputPoints = new ArrayList<>();
        for (int i = 0; i < PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();

        estimator = null;
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(emptyPoints, outputPoints,
                            RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputPoints, emptyPoints,
                            RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with listener and method
        final ProjectiveTransformation2DRobustEstimatorListener listener =
                new ProjectiveTransformation2DRobustEstimatorListener() {

                    @Override
                    public void onEstimateStart(final ProjectiveTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateEnd(final ProjectiveTransformation2DRobustEstimator estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final ProjectiveTransformation2DRobustEstimator estimator, final int iteration) {
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final ProjectiveTransformation2DRobustEstimator estimator, final float progress) {
                    }
                };


        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints,
                        RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with quality scores and method
        final double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        final double[] wrongQualityScores = new double[1];

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points, quality scores and method
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(emptyPoints, outputPoints, qualityScores,
                            RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputPoints, emptyPoints, qualityScores,
                            RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputPoints, outputPoints, wrongQualityScores,
                            RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener, quality scores and method
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores, RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points, quality scores and method
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof
                LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof
                MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores,
                        RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, emptyPoints, outputPoints, qualityScores,
                            RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, inputPoints, emptyPoints, qualityScores,
                            RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, inputPoints, outputPoints, wrongQualityScores,
                            RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test no arguments
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create();
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with listener
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and points
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, emptyPoints, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create(listener, inputPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with points and quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener and quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test with listener, points and quality scores
        estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                create(listener, inputPoints, outputPoints, qualityScores);
        assertTrue(estimator instanceof
                PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
