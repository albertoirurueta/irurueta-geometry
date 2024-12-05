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

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MetricTransformation2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Rotation2D;
import com.irurueta.geometry.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MetricTransformation2DEstimatorTest implements MetricTransformation2DEstimatorListener {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_TRANSLATION = -100.0;
    private static final double MAX_TRANSLATION = 100.0;

    private static final double MIN_RANDOM_VALUE = 50.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_SCALE = 0.5;
    private static final double MAX_SCALE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstructor() {
        // empty constructor
        var estimator = new MetricTransformation2DEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with points
        final var inputPoints = new ArrayList<Point2D>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());

        final var outputPoints = new ArrayList<Point2D>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());

        // constructor with points
        estimator = new MetricTransformation2DEstimator(inputPoints, outputPoints);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        final var wrong = new ArrayList<Point2D>();
        wrong.add(Point2D.create());

        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(wrong, wrong));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(wrong, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(inputPoints, wrong));

        // constructor with listener
        estimator = new MetricTransformation2DEstimator(this);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with listener and points
        estimator = new MetricTransformation2DEstimator(this, inputPoints, outputPoints);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(this, wrong,
                wrong));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(this, wrong,
                outputPoints));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(this,
                inputPoints, wrong));

        // empty constructor with weak minimum points allowed
        estimator = new MetricTransformation2DEstimator(true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with points
        final var inputPoints2 = new ArrayList<Point2D>();
        inputPoints2.add(Point2D.create());
        inputPoints2.add(Point2D.create());

        final var outputPoints2 = new ArrayList<Point2D>();
        outputPoints2.add(Point2D.create());
        outputPoints2.add(Point2D.create());


        // constructor with points
        estimator = new MetricTransformation2DEstimator(inputPoints2, outputPoints2, true);

        // check default values
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MetricTransformation2DEstimator(wrong, wrong, true));
        assertThrows(IllegalArgumentException.class,
                () -> new MetricTransformation2DEstimator(wrong, outputPoints, true));
        assertThrows(IllegalArgumentException.class,
                () -> new MetricTransformation2DEstimator(inputPoints, wrong, true));

        // constructor with listener
        estimator = new MetricTransformation2DEstimator(this, true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with listener and points
        estimator = new MetricTransformation2DEstimator(this, inputPoints, outputPoints, true);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(this, wrong,
                wrong, true));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(this, wrong,
                outputPoints, true));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation2DEstimator(this,
                inputPoints, wrong, true));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var estimator = new MetricTransformation2DEstimator();

        // initial values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());

        // set values
        final var inputPoints = new ArrayList<Point2D>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());

        final var outputPoints = new ArrayList<Point2D>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());

        // Force IllegalArgumentException
        final var wrong = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(wrong, wrong));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(wrong, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(inputPoints, wrong));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new MetricTransformation2DEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final var estimator = new MetricTransformation2DEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
    }

    @Test
    void testEstimateNoLMSE() throws NotReadyException, LockedException, CoincidentPointsException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var transformation = new MetricTransformation2D(rotation, translation, scale);

            // generate random list of input points and transform them
            final var inputPoints = new ArrayList<Point2D>();
            for (var i = 0; i < MetricTransformation2DEstimator.MINIMUM_SIZE; i++) {
                final var x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var inputPoint = new InhomogeneousPoint2D(x, y);
                inputPoints.add(inputPoint);
            }

            // transform points
            final var outputPoints = transformation.transformPointsAndReturnNew(inputPoints);

            final var estimator = new MetricTransformation2DEstimator(this, inputPoints, outputPoints);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation3 = new MetricTransformation2D();
            estimator.estimate(transformation3);

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            // check correctness of estimated transformations

            // transform points using transformation2
            final var outputPoints2 = transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (var i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR));
            }

            final var rotation2 = transformation2.getRotation();

            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

            assertEquals(rotation2.getTheta(), rotation.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final var outputPoints3 = transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            var isValid = true;
            for (var i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR)) {
                    isValid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR));
            }

            if (!isValid) {
                continue;
            }

            final var rotation3 = transformation3.getRotation();

            final var translation3 = transformation3.getTranslation();
            final var scale3 = transformation3.getScale();

            assertEquals(rotation.getTheta(), rotation3.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            assertEquals(scale, scale3, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateLMSE() throws NotReadyException, LockedException, CoincidentPointsException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {

            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var transformation = new MetricTransformation2D(rotation, translation, scale);

            // generate random list of input points and transform them
            final var inputPoints = new ArrayList<Point2D>();
            InhomogeneousPoint2D inputPoint;
            for (var i = 0; i < MetricTransformation2DEstimator.MINIMUM_SIZE + 1; i++) {
                final var x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint2D(x, y);
                inputPoints.add(inputPoint);
            }

            // transform points
            final var outputPoints = transformation.transformPointsAndReturnNew(inputPoints);

            final var estimator = new MetricTransformation2DEstimator(this, inputPoints, outputPoints);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation3 = new MetricTransformation2D();
            estimator.estimate(transformation3);

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            // check correctness of estimated transformations

            // transform points using transformation2
            final var outputPoints2 = transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (var i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR));
            }

            final var rotation2 = transformation2.getRotation();

            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

            assertEquals(rotation.getTheta(), rotation2.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final var outputPoints3 = transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            var valid = true;
            for (var i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR)) {
                    valid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR));
            }

            if (!valid) {
                continue;
            }

            final var rotation3 = transformation3.getRotation();

            final var translation3 = transformation3.getTranslation();
            final var scale3 = transformation3.getScale();

            assertEquals(rotation.getTheta(), rotation3.getTheta(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            assertEquals(scale, scale3, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateColinear() throws NotReadyException, LockedException, CoincidentPointsException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {

            final var randomizer = new UniformRandomizer();

            final var theta = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new Rotation2D(theta);

            final var translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var transformation = new MetricTransformation2D(rotation, translation, scale);

            // generate random list of input points and transform them
            // generate random line
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var line = new Line2D(a, b, c);

            final var inputPoints = new ArrayList<Point2D>();
            HomogeneousPoint2D inputPoint;
            for (var i = 0; i < MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE; i++) {
                final double homX;
                final double homY;
                final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homW) / a;
                }

                inputPoint = new HomogeneousPoint2D(homX, homY, homW);

                assertTrue(line.isLocus(inputPoint));

                inputPoints.add(inputPoint);
            }

            // transform points
            final var outputPoints = transformation.transformPointsAndReturnNew(inputPoints);

            final var estimator = new MetricTransformation2DEstimator(this, inputPoints, outputPoints,
                    true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation3 = new MetricTransformation2D();
            estimator.estimate(transformation3);

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            // check correctness of estimated transformations

            // transform points using transformation2
            final var outputPoints2 = transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            var isValid = true;
            for (var i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR)) {
                    isValid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR));
            }

            if (!isValid) {
                continue;
            }

            final var rotation2 = transformation2.getRotation();

            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

            if (Math.abs(rotation2.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getTheta(), rotation2.getTheta(), ABSOLUTE_ERROR);
            var failed = false;
            for (var i = 0; i < translation.length; i++) {
                if (Math.abs(translation[i] - translation2[i]) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
            }
            if (failed) {
                continue;
            }
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            if (Math.abs(scale - scale2) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final var outputPoints3 = transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            final var rotation3 = transformation3.getRotation();

            final var translation3 = transformation3.getTranslation();
            final var scale3 = transformation3.getScale();

            if (Math.abs(rotation3.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getTheta(), rotation3.getTheta(), ABSOLUTE_ERROR);
            for (var i = 0; i < translation.length; i++) {
                if (Math.abs(translation[i] - translation3[i]) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
            }
            if (failed) {
                continue;
            }
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            if (Math.abs(scale - scale3) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scale, scale3, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final MetricTransformation2DEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final MetricTransformation2DEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(final MetricTransformation2DEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.estimate(null));
        assertThrows(LockedException.class, () -> estimator.setWeakMinimumSizeAllowed(true));
    }
}
