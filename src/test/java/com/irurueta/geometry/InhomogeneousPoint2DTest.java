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

package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class InhomogeneousPoint2DTest {

    private static final int HOM_COORDS = 3;
    private static final int INHOM_COORDS = 2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int TIMES = 100;

    @Test
    public void testConstructor() {
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D();
        assertEquals(iPoint.getInhomX(), 0.0, 0.0);
        assertEquals(iPoint.getInhomY(), 0.0, 0.0);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iPoint = new InhomogeneousPoint2D(array);

        final double[] array2 = iPoint.asArray();

        assertArrayEquals(array, array2, 0.0);

        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iPoint = new InhomogeneousPoint2D(a, b);
        array = iPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);

        final Point2D point = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomogeneousCoordinates(a, b);
        iPoint = new InhomogeneousPoint2D(point);
        assertEquals(iPoint.getInhomX(), a, 0.0);
        assertEquals(iPoint.getInhomY(), b, 0.0);
        assertEquals(iPoint.getType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
    }

    @Test
    public void testGettersAndSetters() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D();

        iPoint.setX(x);
        iPoint.setY(y);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);

        iPoint.setHomogeneousCoordinates(homX, homY, homW);
        final double constantX = iPoint.getHomX() / homX;
        final double constantY = iPoint.getHomY() / homY;
        final double constantW = iPoint.getHomW() / homW;
        assertEquals(constantX, constantY, ABSOLUTE_ERROR);
        assertEquals(constantY, constantW, ABSOLUTE_ERROR);
        assertEquals(constantW, constantX, ABSOLUTE_ERROR);

        iPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);
    }

    @Test
    public void testToHomogeneous() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);

        final HomogeneousPoint2D hPoint = iPoint.toHomogeneous();

        // check that inhomogeneous coordinates are almost equal
        assertEquals(hPoint.getInhomX(), iPoint.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomY(), iPoint.getInhomY(), ABSOLUTE_ERROR);

        // check that homogeneous coordinates are up to scale
        final double scaleX = hPoint.getHomX() / iPoint.getHomX();
        final double scaleY = hPoint.getHomY() / iPoint.getHomY();
        final double scaleW = hPoint.getHomW() / iPoint.getHomW();

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    public void testSetCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(array);
        double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        array = new double[INHOM_COORDS + 1];
        iPoint = new InhomogeneousPoint2D();
        try {
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        array = new double[INHOM_COORDS - 1];
        iPoint = new InhomogeneousPoint2D();
        try {
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(x, y);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);

        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint2D();
        iPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint2D iPoint2 = new InhomogeneousPoint2D(array);
        iPoint = new InhomogeneousPoint2D();
        // pass another point to set coordinates
        iPoint.setCoordinates(iPoint2);
        array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(hPoint);
        array2 = iPoint.asArray();
        assertEquals(array.length, HOM_COORDS);
        assertEquals(array2.length, INHOM_COORDS);
        assertEquals(array[0] / array[2], array2[0], 0.0);
        assertEquals(array[1] / array[2], array2[1], 0.0);
    }

    @Test
    public void testArray() {
        final double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[INHOM_COORDS];
        iPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    public void testEquals() {
        for (int i = 0; i < TIMES; i++) {
            final double[] array = new double[INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            InhomogeneousPoint2D iPoint1 = new InhomogeneousPoint2D(array);
            InhomogeneousPoint2D iPoint2 = new InhomogeneousPoint2D(array);
            assertTrue(iPoint1.equals(iPoint2, 0.0));
            assertTrue(iPoint1.equals((Point2D) iPoint2, 0.0));

            array[0] = iPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            array[1] = iPoint1.getY();
            iPoint2 = new InhomogeneousPoint2D(array);
            assertFalse(iPoint1.equals(iPoint2, 0.0));
            assertFalse(iPoint1.equals((Point2D) iPoint2, 0.0));

            array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            iPoint1 = new InhomogeneousPoint2D(array);
            array[0] += 1.0;
            iPoint2 = new InhomogeneousPoint2D(array);
            assertTrue(iPoint1.equals(iPoint2, 1.0 + ABSOLUTE_ERROR));
            assertTrue(iPoint1.equals((Point2D) iPoint2, 1.0 + ABSOLUTE_ERROR));
            assertFalse(iPoint1.equals(iPoint2, 0.5));
            assertFalse(iPoint1.equals(iPoint2, 0.5));

            // Testing equals from one homogeneous point
            final double[] hArray = new double[HOM_COORDS];
            randomizer.fill(hArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            array[0] = hArray[0] / hArray[2];
            array[1] = hArray[1] / hArray[2];

            InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
            HomogeneousPoint2D hPoint = new HomogeneousPoint2D(hArray);
            assertTrue(iPoint.equals(hPoint, ABSOLUTE_ERROR));
            assertTrue(iPoint.equals((Point2D) hPoint, ABSOLUTE_ERROR));

            hArray[0] = iPoint.getHomX() + 1.0;
            hArray[1] = iPoint.getHomY() + 1.0;
            hArray[2] = iPoint.getHomW() + 1.0;
            hPoint = new HomogeneousPoint2D(hArray);
            assertFalse(iPoint.equals(hPoint, 0.0));
            assertFalse(iPoint.equals((Point2D) hPoint, 0.0));

            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            iPoint = new InhomogeneousPoint2D(array);

            hArray[0] = iPoint.getHomX() + iPoint.getHomW();
            hArray[1] = iPoint.getHomY() + iPoint.getHomW();
            hArray[2] = iPoint.getHomW();
            hPoint = new HomogeneousPoint2D(hArray);
            assertTrue(iPoint.equals(hPoint, 1.1));
            assertTrue(iPoint.equals((Point2D) hPoint, 1.1));
            assertFalse(iPoint.equals(hPoint, 0.5));
            assertFalse(iPoint.equals((Point2D) hPoint, 0.5));

            // Force IllegalArgumentException
            boolean value = false;
            try {
                value = iPoint.equals(hPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                value = iPoint.equals(iPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                value = iPoint.equals((Point2D) iPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertFalse(value);
        }
    }

    @Test
    public void testIsAtInfinity() {
        final double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        iPoint.setX(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setY(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint2D(array);
        assertFalse(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setX(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setY(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setX(Double.NaN);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setY(Double.NaN);
        assertTrue(iPoint.isAtInfinity());
    }
}
