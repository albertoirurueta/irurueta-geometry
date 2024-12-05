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
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class MetricTransformation3DEstimatorTest implements MetricTransformation3DEstimatorListener {

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
        var estimator = new MetricTransformation3DEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with points
        final var inputPoints = new ArrayList<Point3D>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());

        final var outputPoints = new ArrayList<Point3D>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());

        estimator = new MetricTransformation3DEstimator(inputPoints, outputPoints);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        final var wrong = new ArrayList<Point3D>();
        wrong.add(Point3D.create());
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(wrong, wrong));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(wrong, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(inputPoints, wrong));

        // constructor with listener
        estimator = new MetricTransformation3DEstimator(this);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with listener and points
        estimator = new MetricTransformation3DEstimator(this, inputPoints, outputPoints);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(this, wrong,
                wrong));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(this, wrong,
                outputPoints));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(this,
                inputPoints, wrong));

        // constructor with weak minimum size allowed
        estimator = new MetricTransformation3DEstimator(true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with points and weak minimum size allowed
        final var inputPoints2 = new ArrayList<Point3D>();
        inputPoints2.add(Point3D.create());
        inputPoints2.add(Point3D.create());
        inputPoints2.add(Point3D.create());

        final var outputPoints2 = new ArrayList<Point3D>();
        outputPoints2.add(Point3D.create());
        outputPoints2.add(Point3D.create());
        outputPoints2.add(Point3D.create());

        estimator = new MetricTransformation3DEstimator(inputPoints2, outputPoints2, true);

        // check default values
        assertSame(inputPoints2, estimator.getInputPoints());
        assertSame(outputPoints2, estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(wrong, wrong,
                true));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(wrong, outputPoints,
                true));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(inputPoints, wrong,
                true));

        // constructor with listener and weak minimum points
        estimator = new MetricTransformation3DEstimator(this, true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // constructor with listener and points
        estimator = new MetricTransformation3DEstimator(this, inputPoints, outputPoints,
                true);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(this, wrong,
                wrong, true));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(this, wrong,
                outputPoints, true));
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3DEstimator(this,
                inputPoints, wrong, true));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var estimator = new MetricTransformation3DEstimator();

        // initial values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());

        // set values
        final var inputPoints = new ArrayList<Point3D>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());

        final var outputPoints = new ArrayList<Point3D>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());

        // Force IllegalArgumentException
        final var wrong = new ArrayList<Point3D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(wrong, wrong));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(wrong, outputPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(inputPoints, wrong));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new MetricTransformation3DEstimator();

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
        final var estimator = new MetricTransformation3DEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.MINIMUM_SIZE, estimator.getMinimumPoints());

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE, estimator.getMinimumPoints());
    }

    @Test
    void testEstimateNoLMSE() throws NotReadyException, LockedException, CoincidentPointsException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {

            final var randomizer = new UniformRandomizer();

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final var translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var transformation = new MetricTransformation3D(q, translation, scale);

            // generate random list of input points and transform them
            final var inputPoints = new ArrayList<Point3D>();
            for (var i = 0; i < MetricTransformation3DEstimator.MINIMUM_SIZE; i++) {
                final var x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var z = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var inputPoint = new InhomogeneousPoint3D(x, y, z);
                inputPoints.add(inputPoint);
            }

            // transform points
            final var outputPoints = transformation.transformPointsAndReturnNew(inputPoints);

            final var estimator = new MetricTransformation3DEstimator(this, inputPoints, outputPoints);

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

            final var transformation3 = new MetricTransformation3D();
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

            final var q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final var outputPoints3 = transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (var i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR));
            }

            final var q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();

            final var translation3 = transformation3.getTranslation();
            final var scale3 = transformation3.getScale();

            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
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

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final var translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var transformation = new MetricTransformation3D(q, translation, scale);

            // generate random list of input points and transform them
            final var inputPoints = new ArrayList<Point3D>();
            for (var i = 0; i < MetricTransformation3DEstimator.MINIMUM_SIZE + 1; i++) {
                final var x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var z = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final var inputPoint = new InhomogeneousPoint3D(x, y, z);
                inputPoints.add(inputPoint);
            }

            // transform points
            final var outputPoints = transformation.transformPointsAndReturnNew(inputPoints);

            final var estimator = new MetricTransformation3DEstimator(this, inputPoints, outputPoints);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final var transformation2 = estimator.estimate();
            final var transformation3 = new MetricTransformation3D();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            estimator.estimate(transformation3);

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            // check correctness of estimated transformations

            // transform points using transformation2
            final var outputPoints2 = transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            var valid = true;
            for (var i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR)) {
                    valid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR));
            }

            if (!valid) {
                continue;
            }

            final var q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final var outputPoints3 = transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (var i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR));
            }

            final var q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();

            final var translation3 = transformation3.getTranslation();
            final var scale3 = transformation3.getScale();

            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            assertEquals(scale, scale3, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimatePlanar() throws NotReadyException, LockedException, CoincidentPointsException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final var translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

            final var transformation = new MetricTransformation3D(q, translation, scale);

            // generate random list of input points and transform them
            // generate random plane
            final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var plane = new Plane(a, b, c, d);

            final var inputPoints = new ArrayList<Point3D>();
            for (var i = 0; i < MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE; i++) {

                final double homX;
                final double homY;
                final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                final var inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(inputPoint));

                inputPoints.add(inputPoint);
            }

            // transform points
            final var outputPoints = transformation.transformPointsAndReturnNew(inputPoints);

            final var estimator = new MetricTransformation3DEstimator(this, inputPoints, outputPoints,
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

            final var transformation3 = new MetricTransformation3D();
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

            final var q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            final var translation2 = transformation2.getTranslation();
            final var scale2 = transformation2.getScale();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            for (var i = 0; i < translation.length; i++) {
                if (Math.abs(translation[i] - translation2[i]) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }
            }

            if (!isValid) {
                continue;
            }

            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final var outputPoints3 = transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (var i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR));
            }

            final var q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();

            final var translation3 = transformation3.getTranslation();
            final var scale3 = transformation3.getScale();

            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            assertEquals(scale, scale3, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    @Override
    public void onEstimateStart(final MetricTransformation3DEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final MetricTransformation3DEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void checkLocked(final MetricTransformation3DEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.estimate(null));
        assertThrows(LockedException.class, () -> estimator.setWeakMinimumSizeAllowed(true));
    }
}
