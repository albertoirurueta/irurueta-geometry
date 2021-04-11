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
import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Rotation2D;
import com.irurueta.geometry.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class EuclideanTransformation2DEstimatorTest implements
        EuclideanTransformation2DEstimatorListener {

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
        EuclideanTransformation2DEstimator estimator =
                new EuclideanTransformation2DEstimator();

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.MINIMUM_SIZE);


        // constructor with points
        List<Point2D> inputPoints = new ArrayList<>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());

        List<Point2D> outputPoints = new ArrayList<>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());

        estimator = new EuclideanTransformation2DEstimator(inputPoints,
                outputPoints);

        // check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.MINIMUM_SIZE);

        // Force IllegalArgumentException
        final List<Point2D> wrong = new ArrayList<>();
        wrong.add(Point2D.create());

        estimator = null;
        try {
            estimator = new EuclideanTransformation2DEstimator(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(wrong,
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(inputPoints,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with listener
        estimator = new EuclideanTransformation2DEstimator(this);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.MINIMUM_SIZE);


        // constructor with listener and points
        estimator = new EuclideanTransformation2DEstimator(this, inputPoints,
                outputPoints);

        // check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.MINIMUM_SIZE);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation2DEstimator(this, wrong,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(this, wrong,
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(this,
                    inputPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // empty constructor with weak minimum points allowed
        estimator = new EuclideanTransformation2DEstimator(true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE);


        // constructor with points
        inputPoints = new ArrayList<>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());

        outputPoints = new ArrayList<>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());

        estimator = new EuclideanTransformation2DEstimator(inputPoints,
                outputPoints, true);

        // check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation2DEstimator(wrong, wrong,
                    true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(wrong,
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(inputPoints,
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // constructor with listener
        estimator = new EuclideanTransformation2DEstimator(this, true);

        // check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE);


        // constructor with listener and points
        estimator = new EuclideanTransformation2DEstimator(this, inputPoints,
                outputPoints, true);

        // check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation2DEstimator(this, wrong,
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(this, wrong,
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EuclideanTransformation2DEstimator(this,
                    inputPoints, wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final EuclideanTransformation2DEstimator estimator =
                new EuclideanTransformation2DEstimator();

        // initial values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());

        // set values
        final List<Point2D> inputPoints = new ArrayList<>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());

        final List<Point2D> outputPoints = new ArrayList<>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());

        estimator.setPoints(inputPoints, outputPoints);

        // check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);

        // Force IllegalArgumentException
        final List<Point2D> wrong = new ArrayList<>();
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
        final EuclideanTransformation2DEstimator estimator =
                new EuclideanTransformation2DEstimator();

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
    public void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        final EuclideanTransformation2DEstimator estimator =
                new EuclideanTransformation2DEstimator();

        // check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.MINIMUM_SIZE);

        // set new value
        estimator.setWeakMinimumSizeAllowed(true);

        // check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE);
    }

    @Test
    public void testEstimateNoLMSE() throws NotReadyException, LockedException,
            CoincidentPointsException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation =
                    new EuclideanTransformation2D(rotation, translation);

            // generate random list of input points and transform them
            final List<Point2D> inputPoints = new ArrayList<>();
            InhomogeneousPoint2D inputPoint;
            for (int i = 0; i < EuclideanTransformation2DEstimator.MINIMUM_SIZE;
                 i++) {
                final double x = randomizer.nextDouble(MIN_TRANSLATION,
                        MAX_TRANSLATION);
                final double y = randomizer.nextDouble(MIN_TRANSLATION,
                        MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint2D(x, y);
                inputPoints.add(inputPoint);
            }

            // transform points
            final List<Point2D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);

            final EuclideanTransformation2DEstimator estimator =
                    new EuclideanTransformation2DEstimator(this, inputPoints,
                            outputPoints);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();
            final EuclideanTransformation2D transformation3 =
                    new EuclideanTransformation2D();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            estimator.estimate(transformation3);

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());


            // check correctness of estimated transformations

            // transform points using transformation2
            final List<Point2D> outputPoints2 =
                    transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i),
                        ABSOLUTE_ERROR));
            }

            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            final Rotation2D rotation3 = transformation3.getRotation();
            final double[] translation3 = transformation3.getTranslation();

            assertEquals(rotation3.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateLMSE() throws NotReadyException, LockedException,
            CoincidentPointsException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation =
                    new EuclideanTransformation2D(rotation, translation);

            // generate random list of input points and transform them
            final List<Point2D> inputPoints = new ArrayList<>();
            InhomogeneousPoint2D inputPoint;
            for (int i = 0; i < EuclideanTransformation2DEstimator.MINIMUM_SIZE + 1; i++) {
                final double x = randomizer.nextDouble(MIN_TRANSLATION,
                        MAX_TRANSLATION);
                final double y = randomizer.nextDouble(MIN_TRANSLATION,
                        MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint2D(x, y);
                inputPoints.add(inputPoint);
            }

            // transform points
            final List<Point2D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);

            final EuclideanTransformation2DEstimator estimator =
                    new EuclideanTransformation2DEstimator(this, inputPoints,
                            outputPoints);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();
            final EuclideanTransformation2D transformation3 =
                    new EuclideanTransformation2D();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            estimator.estimate(transformation3);

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());


            // check correctness of estimated transformations

            // transform points using transformation2
            final List<Point2D> outputPoints2 =
                    transformation2.transformPointsAndReturnNew(inputPoints);

            // check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i),
                        ABSOLUTE_ERROR));
            }

            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            final Rotation2D rotation3 = transformation3.getRotation();
            final double[] translation3 = transformation3.getTranslation();

            assertEquals(rotation3.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateColinear() throws NotReadyException, LockedException,
            CoincidentPointsException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final Rotation2D rotation = new Rotation2D(theta);

            final double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);

            final EuclideanTransformation2D transformation =
                    new EuclideanTransformation2D(rotation, translation);

            // generate random list of input points and transform them
            // generate random line
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final Line2D line = new Line2D(a, b, c);

            final List<Point2D> inputPoints = new ArrayList<>();
            HomogeneousPoint2D inputPoint;
            for (int i = 0; i < EuclideanTransformation2DEstimator.WEAK_MINIMUM_SIZE;
                 i++) {
                final double homX;
                final double homY;
                final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homW) / a;
                }

                inputPoint = new HomogeneousPoint2D(homX, homY, homW);

                assertTrue(line.isLocus(inputPoint));

                inputPoints.add(inputPoint);
            }

            // transform points
            final List<Point2D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);

            final EuclideanTransformation2DEstimator estimator =
                    new EuclideanTransformation2DEstimator(this, inputPoints,
                            outputPoints, true);

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            final EuclideanTransformation2D transformation2 = estimator.estimate();
            final EuclideanTransformation2D transformation3 =
                    new EuclideanTransformation2D();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());

            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            estimator.estimate(transformation3);

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());


            // check correctness of estimated transformations

            // transform points using transformation2
            final List<Point2D> outputPoints2 =
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

            final Rotation2D rotation2 = transformation2.getRotation();
            final double[] translation2 = transformation2.getTranslation();

            if (Math.abs(rotation2.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation2.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);

            final Rotation2D rotation3 = transformation3.getRotation();
            final double[] translation3 = transformation3.getTranslation();

            if (Math.abs(rotation3.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation3.getTheta(), rotation.getTheta(),
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final EuclideanTransformation2DEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final EuclideanTransformation2DEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(final EuclideanTransformation2DEstimator estimator) {
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
