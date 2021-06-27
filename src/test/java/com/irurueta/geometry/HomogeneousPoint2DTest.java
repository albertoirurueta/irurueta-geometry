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

import java.io.IOException;
import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class HomogeneousPoint2DTest {

    private static final int HOM_COORDS = 3;
    private static final int INHOM_COORDS = 2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    public void testConstructor() {
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        assertEquals(hPoint.getHomX(), 0.0, 0.0);
        assertEquals(hPoint.getHomY(), 0.0, 0.0);
        assertEquals(hPoint.getHomW(), 1.0, 0.0);
        assertFalse(hPoint.isNormalized());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint = new HomogeneousPoint2D(array);

        final double[] array2 = hPoint.asArray();

        assertArrayEquals(array, array2, 0.0);
        assertFalse(hPoint.isNormalized());

        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint = new HomogeneousPoint2D(a, b, c);
        array = hPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertFalse(hPoint.isNormalized());

        final Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setHomogeneousCoordinates(a, b, c);
        hPoint = new HomogeneousPoint2D(point);
        assertEquals(hPoint.getHomX(), a, 0.0);
        assertEquals(hPoint.getHomY(), b, 0.0);
        assertEquals(hPoint.getHomW(), c, 0.0);
        assertEquals(hPoint.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertFalse(hPoint.isNormalized());
    }

    @Test
    public void testGettersAndSetters() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final HomogeneousPoint2D hPoint = new HomogeneousPoint2D();

        hPoint.setX(x);
        hPoint.setY(y);
        hPoint.setW(w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);

        hPoint.setHomogeneousCoordinates(homX, homY, homW);
        assertEquals(hPoint.getHomX(), homX, 0.0);
        assertEquals(hPoint.getHomY(), homY, 0.0);
        assertEquals(hPoint.getHomW(), homW, 0.0);
        assertEquals(hPoint.getX(), homX, 0.0);
        assertEquals(hPoint.getY(), homY, 0.0);
        assertEquals(hPoint.getW(), homW, 0.0);

        hPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
    }

    @Test
    public void testToInhomogeneous() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);

        final InhomogeneousPoint2D iPoint = hPoint.toInhomogeneous();

        assertEquals(iPoint.getX(), hPoint.getX() / hPoint.getW(),
                ABSOLUTE_ERROR);
        assertEquals(iPoint.getY(), hPoint.getY() / hPoint.getW(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testSetCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(array);
        double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        array = new double[HOM_COORDS + 1];
        hPoint = new HomogeneousPoint2D();
        try {
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        array = new double[HOM_COORDS - 1];
        hPoint = new HomogeneousPoint2D();
        try {
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(x, y, w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);

        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D();
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint2D hPoint2 = new HomogeneousPoint2D(array);
        hPoint = new HomogeneousPoint2D();
        // pass another point to set coordinates
        hPoint.setCoordinates(hPoint2);
        array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(iPoint);
        array2 = hPoint.asArray();
        assertEquals(array.length, INHOM_COORDS);
        assertEquals(array2.length, HOM_COORDS);
        assertEquals(array[0], array2[0], 0.0);
        assertEquals(array[1], array2[1], 0.0);
        assertEquals(1.0, array2[2], 0.0);
    }

    @Test
    public void testArray() {
        final double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[HOM_COORDS];
        hPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    public void testEqualsAndHashCode() {
        final double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        HomogeneousPoint2D hPoint1 = new HomogeneousPoint2D(array);
        HomogeneousPoint2D hPoint2 = new HomogeneousPoint2D(array);
        assertTrue(hPoint1.equals(hPoint2, 0.0));
        assertTrue(hPoint1.equals((Point2D) hPoint2, 0.0));
        assertEquals(hPoint1.hashCode(), hPoint2.hashCode());

        array[0] = hPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        array[1] = hPoint1.getY();
        array[2] = hPoint1.getW();
        hPoint2 = new HomogeneousPoint2D(array);
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point2D) hPoint2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint1 = new HomogeneousPoint2D(array);
        array[0] *= 2.0;
        hPoint2 = new HomogeneousPoint2D(array);
        assertTrue(hPoint1.equals(hPoint2, 2.0));
        assertTrue(hPoint1.equals((Point2D) hPoint2, 2.0));
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point2D) hPoint2, 0.0));

        // Testing equals from one inhomogeneous point
        final double[] iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[2];
        iArray[1] = array[1] / array[2];

        HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(iArray);
        assertTrue(hPoint.equals(iPoint, ABSOLUTE_ERROR));
        assertTrue(hPoint.equals((Point2D) iPoint, ABSOLUTE_ERROR));

        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iPoint = new InhomogeneousPoint2D(iArray);
        assertFalse(hPoint.equals(iPoint, 0.0));
        assertFalse(hPoint.equals((Point2D) iPoint, 0.0));

        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D(array);

        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iPoint = new InhomogeneousPoint2D(iArray);
        assertTrue(hPoint.equals(iPoint, 1.1));
        assertTrue(hPoint.equals((Point2D) iPoint, 1.1));
        assertFalse(hPoint.equals(iPoint, 0.5));
        assertFalse(hPoint.equals((Point2D) iPoint, 0.5));

        // Force IllegalArgumentException
        boolean value = false;
        try {
            value = hPoint.equals(hPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = hPoint.equals(iPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = hPoint.equals((Point2D) iPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertFalse(value);

        hPoint = new HomogeneousPoint2D();
        iPoint = new InhomogeneousPoint2D();
        assertTrue(hPoint.equals(iPoint));
        assertEquals(hPoint.hashCode(), iPoint.hashCode());
        hPoint2 = new HomogeneousPoint2D(hPoint);
        assertTrue(hPoint.equals(hPoint2));
        assertEquals(hPoint.hashCode(), hPoint2.hashCode());
        assertTrue(hPoint.equals((Point2D) iPoint));
        assertTrue(hPoint.equals((Point2D) hPoint2));
    }

    @Test
    public void testIsAtInfinity() {
        final HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        // Sets point at infinity
        hPoint.setW(0.0);
        assertTrue(hPoint.isAtInfinity());
        // Forces point not being at infinity
        hPoint.setW(1.0);
        assertFalse(hPoint.isAtInfinity());

        // Testing with threshold
        hPoint.setW(4.0);
        assertTrue(hPoint.isAtInfinity(4.5));
        hPoint.setW(5.0);
        assertFalse(hPoint.isAtInfinity(4.5));

        // Force IllegalArgumentException
        try {
            hPoint.isAtInfinity(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testNormalize() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final HomogeneousPoint2D point = new HomogeneousPoint2D(homX, homY, homW);
        assertEquals(point.getHomX(), homX, 0.0);
        assertEquals(point.getHomY(), homY, 0.0);
        assertEquals(point.getHomW(), homW, 0.0);
        assertFalse(point.isNormalized());

        // compute norm
        final double norm = Math.sqrt(homX * homX + homY * homY + homW * homW);

        // normalize
        point.normalize();
        assertTrue(point.isNormalized());

        // check correctness after normalization
        assertEquals(point.getHomX(), homX / norm, ABSOLUTE_ERROR);
        assertEquals(point.getHomY(), homY / norm, ABSOLUTE_ERROR);
        assertEquals(point.getHomW(), homW / norm, ABSOLUTE_ERROR);

        point.setHomogeneousCoordinates(homX, homY, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setCoordinates(homX, homY, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setInhomogeneousCoordinates(homX / homW, homY / homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        final double[] array = new double[HOM_COORDS];
        // initialize to zeros
        Arrays.fill(array, 0.0);
        point.setCoordinates(array);
        assertFalse(point.isNormalized());
        // normalization is not done because of numerical
        point.normalize();
        // instability
        assertFalse(point.isNormalized());

        // fill array with random values
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setCoordinates(array);
        assertFalse(point.isNormalized());
        // now normalization is done
        point.normalize();
        assertTrue(point.isNormalized());

        final HomogeneousPoint2D point2 = new HomogeneousPoint2D();
        point.setCoordinates(point2);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setX(homX);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setY(homY);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setW(homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint2D hPoint1 = new HomogeneousPoint2D(array);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(hPoint1);
        final HomogeneousPoint2D hPoint2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(hPoint1, hPoint2);
    }
}
