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
import com.irurueta.geometry.EuclideanTransformation3D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class EuclideanTransformation3DEstimatorTest implements
        EuclideanTransformation3DEstimatorListener {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_TRANSLATION = -100.0;
    private static final double MAX_TRANSLATION = 100.0;

    private static final double MIN_RANDOM_VALUE = 50.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    public void testConstructor() {
        // empty constructor
        EuclideanTransformation3DEstimator estimator =
                new EuclideanTransformation3DEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.MINIMUM_SIZE,
                estimator.getMinimumPoints());


        // constructor with points
        List<Point3D> inputPoints = new ArrayList<>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());

        List<Point3D> outputPoints = new ArrayList<>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());

        estimator = new EuclideanTransformation3DEstimator(inputPoints,
                outputPoints);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.MINIMUM_SIZE,
                estimator.getMinimumPoints());

        // Force IllegalArgumentException
        final List<Point3D> wrong = new ArrayList<>();
        wrong.add(Point3D.create());

        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong,
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(inputPoints,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with listener
        estimator = new EuclideanTransformation3DEstimator(this);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.MINIMUM_SIZE,
                estimator.getMinimumPoints());


        // constructor with listener and points
        estimator = new EuclideanTransformation3DEstimator(this, inputPoints,
                outputPoints);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.MINIMUM_SIZE,
                estimator.getMinimumPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong,
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(this,
                    inputPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // empty constructor with weak minimum points allowed
        estimator = new EuclideanTransformation3DEstimator(true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE,
                estimator.getMinimumPoints());


        // constructor with points
        inputPoints = new ArrayList<>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());

        outputPoints = new ArrayList<>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());

        estimator = new EuclideanTransformation3DEstimator(inputPoints,
                outputPoints, true);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE,
                estimator.getMinimumPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong, wrong,
                    true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong,
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(inputPoints,
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with listener
        estimator = new EuclideanTransformation3DEstimator(this, true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE,
                estimator.getMinimumPoints());


        // constructor with listener and points
        estimator = new EuclideanTransformation3DEstimator(this, inputPoints,
                outputPoints, true);

        // check default values
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE,
                estimator.getMinimumPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong,
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong,
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation3DEstimator(this,
                    inputPoints, wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final EuclideanTransformation3DEstimator estimator =
                new EuclideanTransformation3DEstimator();

        // initial values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());

        // set values
        final List<Point3D> inputPoints = new ArrayList<>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());

        final List<Point3D> outputPoints = new ArrayList<>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(inputPoints, estimator.getInputPoints());
        assertSame(outputPoints, estimator.getOutputPoints());

        // Force IllegalArgumentException
        final List<Point3D> wrong = new ArrayList<>();
        try {
            estimator.setPoints(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(wrong, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(inputPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final EuclideanTransformation3DEstimator estimator =
                new EuclideanTransformation3DEstimator();

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
    public void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final EuclideanTransformation3DEstimator estimator =
                new EuclideanTransformation3DEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.MINIMUM_SIZE,
                estimator.getMinimumPoints());

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE,
                estimator.getMinimumPoints());
    }

    @Test
    public void testEstimateNoLMSE() throws NotReadyException, LockedException,
            CoincidentPointsException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation3D transformation =
                    new EuclideanTransformation3D(q, translation);

            // generate random list of input points and transform them
            final List<Point3D> inputPoints = new ArrayList<>();
            InhomogeneousPoint3D inputPoint;
            for (int i = 0; i < EuclideanTransformation3DEstimator.MINIMUM_SIZE; i++) {
                final double x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final double y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final double z = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint3D(x, y, z);
                inputPoints.add(inputPoint);
            }

            // transform points
            final List<Point3D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);

            final EuclideanTransformation3DEstimator estimator =
                    new EuclideanTransformation3DEstimator(this, inputPoints,
                            outputPoints);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation3D transformation2 = estimator.estimate();
            final EuclideanTransformation3D transformation3 =
                    new EuclideanTransformation3D();

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
            final List<Point3D> outputPoints2 =
                    transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i),
                        ABSOLUTE_ERROR));
            }

            final Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            final double[] translation2 = transformation2.getTranslation();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);


            // transform points using transformation3
            final List<Point3D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i),
                        ABSOLUTE_ERROR));
            }

            final Quaternion q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();

            final double[] translation3 = transformation3.getTranslation();

            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    public void testEstimateLMSE() throws NotReadyException, LockedException,
            CoincidentPointsException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation3D transformation =
                    new EuclideanTransformation3D(q, translation);

            // generate random list of input points and transform them
            final List<Point3D> inputPoints = new ArrayList<>();
            InhomogeneousPoint3D inputPoint;
            for (int i = 0; i < EuclideanTransformation3DEstimator.MINIMUM_SIZE + 1; i++) {
                final double x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final double y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                final double z = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint3D(x, y, z);
                inputPoints.add(inputPoint);
            }

            // transform points
            final List<Point3D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);

            final EuclideanTransformation3DEstimator estimator =
                    new EuclideanTransformation3DEstimator(this, inputPoints,
                            outputPoints);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation3D transformation2 = estimator.estimate();
            final EuclideanTransformation3D transformation3 =
                    new EuclideanTransformation3D();

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
            final List<Point3D> outputPoints2 =
                    transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            boolean valid = true;
            for (int i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR)) {
                    valid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i),
                        ABSOLUTE_ERROR));
            }

            if (!valid) {
                continue;
            }

            final Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            final double[] translation2 = transformation2.getTranslation();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final List<Point3D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i),
                        ABSOLUTE_ERROR));
            }

            final Quaternion q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();

            final double[] translation3 = transformation3.getTranslation();

            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    public void testEstimatePlanar() throws NotReadyException, LockedException,
            CoincidentPointsException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();

            final double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation3D transformation =
                    new EuclideanTransformation3D(q, translation);

            // generate random list of input points and transform them
            // generate random plane
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double d = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Plane plane = new Plane(a, b, c, d);

            final List<Point3D> inputPoints = new ArrayList<>();
            HomogeneousPoint3D inputPoint;
            for (int i = 0; i < MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE; i++) {

                final double homX;
                final double homY;
                final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(inputPoint));

                inputPoints.add(inputPoint);
            }

            // transform points
            final List<Point3D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);

            final EuclideanTransformation3DEstimator estimator =
                    new EuclideanTransformation3DEstimator(this, inputPoints,
                            outputPoints, true);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation3D transformation2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation3D transformation3 =
                    new EuclideanTransformation3D();
            estimator.estimate(transformation3);

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isLocked());


            // check correctness of estimated transformations

            // transform points using transformation2
            final List<Point3D> outputPoints2 =
                    transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            boolean isValid = true;
            for (int i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i),
                        ABSOLUTE_ERROR)) {
                    isValid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i),
                        ABSOLUTE_ERROR));
            }

            if (!isValid) {
                continue;
            }

            final Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            final double[] translation2 = transformation2.getTranslation();

            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            // transform points using transformation3
            final List<Point3D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i),
                        ABSOLUTE_ERROR));
            }

            final Quaternion q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();

            final double[] translation3 = transformation3.getTranslation();

            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final EuclideanTransformation3DEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final EuclideanTransformation3DEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(final EuclideanTransformation3DEstimator estimator) {
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
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
        try {
            estimator.estimate(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setWeakMinimumSizeAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
