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

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class HomogeneousPoint3DTest {

    private static final int HOM_COORDS = 4;
    private static final int INHOM_COORDS = 3;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    public void testConstructor() {
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        assertEquals(hPoint.getHomX(), 0.0, 0.0);
        assertEquals(hPoint.getHomY(), 0.0, 0.0);
        assertEquals(hPoint.getHomZ(), 0.0, 0.0);
        assertEquals(hPoint.getHomW(), 0.0, 1.0);
        assertFalse(hPoint.isNormalized());

        double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint = new HomogeneousPoint3D(array);

        final double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        assertFalse(hPoint.isNormalized());

        final double a = randomizer.nextDouble();
        final double b = randomizer.nextDouble();
        final double c = randomizer.nextDouble();
        final double d = randomizer.nextDouble();

        hPoint = new HomogeneousPoint3D(a, b, c, d);
        array = hPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertEquals(d, array[3], 0.0);
        assertFalse(hPoint.isNormalized());

        Point3D point = Point3D.create(
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        hPoint = new HomogeneousPoint3D(point);
        assertEquals(hPoint.getType(),
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertFalse(hPoint.isNormalized());

        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        hPoint = new HomogeneousPoint3D(point);
        assertEquals(hPoint.getType(),
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertFalse(hPoint.isNormalized());
    }

    @Test
    public void testGettersAndSetters() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble();
        final double y = randomizer.nextDouble();
        final double z = randomizer.nextDouble();
        final double w = randomizer.nextDouble();
        final double homX = randomizer.nextDouble();
        final double homY = randomizer.nextDouble();
        final double homZ = randomizer.nextDouble();
        final double homW = randomizer.nextDouble();
        final double inhomX = randomizer.nextDouble();
        final double inhomY = randomizer.nextDouble();
        final double inhomZ = randomizer.nextDouble();

        final HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        hPoint.setX(x);
        hPoint.setY(y);
        hPoint.setZ(z);
        hPoint.setW(w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getZ(), z, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);

        hPoint.setHomogeneousCoordinates(homX, homY, homZ, homW);
        assertEquals(hPoint.getHomX(), homX, 0.0);
        assertEquals(hPoint.getHomY(), homY, 0.0);
        assertEquals(hPoint.getHomZ(), homZ, 0.0);
        assertEquals(hPoint.getHomW(), homW, 0.0);
        assertEquals(hPoint.getX(), homX, 0.0);
        assertEquals(hPoint.getY(), homY, 0.0);
        assertEquals(hPoint.getW(), homW, 0.0);

        hPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
        assertEquals(hPoint.getZ() / hPoint.getW(), inhomZ, 0.0);
    }

    @Test
    public void testToInhomogeneous() {
        final double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);

        final InhomogeneousPoint3D iPoint = hPoint.toInhomogeneous();

        assertEquals(iPoint.getX(), hPoint.getX() / hPoint.getW(),
                ABSOLUTE_ERROR);
        assertEquals(iPoint.getY(), hPoint.getY() / hPoint.getW(),
                ABSOLUTE_ERROR);
        assertEquals(iPoint.getZ(), hPoint.getZ() / hPoint.getW(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testSetCoordinates() {
        double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        hPoint.setCoordinates(array);
        double[] array2 = hPoint.asArray();

        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        array = new double[HOM_COORDS + 1];
        hPoint = new HomogeneousPoint3D();
        try {
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        array = new double[HOM_COORDS - 1];
        hPoint = new HomogeneousPoint3D();
        try {
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final double x = randomizer.nextDouble();
        final double y = randomizer.nextDouble();
        final double z = randomizer.nextDouble();
        final double w = randomizer.nextDouble();
        hPoint = new HomogeneousPoint3D();
        hPoint.setCoordinates(x, y, z, w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getZ(), z, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);

        final double inhomX = randomizer.nextDouble();
        final double inhomY = randomizer.nextDouble();
        final double inhomZ = randomizer.nextDouble();
        hPoint = new HomogeneousPoint3D();
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
        assertEquals(hPoint.getZ() / hPoint.getW(), inhomZ, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final HomogeneousPoint3D hPoint2 = new HomogeneousPoint3D(array);
        hPoint = new HomogeneousPoint3D();
        // pass another point to set coordinates
        hPoint.setCoordinates(hPoint2);
        array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        hPoint = new HomogeneousPoint3D();
        // pass another point to set coordinates
        hPoint.setCoordinates(iPoint);
        array2 = hPoint.asArray();
        assertEquals(array[0], array2[0], 0.0);
        assertEquals(array[1], array2[1], 0.0);
        assertEquals(array[2], array2[2], 0.0);
        assertEquals(1.0, array2[3], 0.0);
    }

    @Test
    public void testAsArray() {
        final double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[HOM_COORDS];
        hPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    public void testEquals() {
        final double[] array = new double[HOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        HomogeneousPoint3D hPoint1 = new HomogeneousPoint3D(array);
        HomogeneousPoint3D hPoint2 = new HomogeneousPoint3D(array);
        assertTrue(hPoint1.equals(hPoint2, 0.0));
        assertTrue(hPoint1.equals((Point3D) hPoint2, 0.0));

        array[0] = hPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        array[1] = hPoint1.getY();
        array[2] = hPoint1.getZ();
        array[3] = hPoint1.getW();

        hPoint2 = new HomogeneousPoint3D(array);
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point3D) hPoint2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint1 = new HomogeneousPoint3D(array);
        array[0] *= 2.0;
        hPoint2 = new HomogeneousPoint3D(array);
        assertTrue(hPoint1.equals(hPoint2, 2.0));
        assertTrue(hPoint1.equals((Point3D) hPoint2, 2.0));
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point3D) hPoint2, 0.0));

        // Testing equals from one inhomogeneous point
        final double[] iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[3];
        iArray[1] = array[1] / array[3];
        iArray[2] = array[2] / array[3];

        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(iArray);
        assertTrue(hPoint.equals(iPoint, ABSOLUTE_ERROR));
        assertTrue(hPoint.equals((Point3D) iPoint, ABSOLUTE_ERROR));

        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iArray[2] = hPoint.getInhomZ() + 1.0;
        iPoint = new InhomogeneousPoint3D(iArray);
        assertFalse(hPoint.equals(iPoint, 0.0));
        assertFalse(hPoint.equals((Point3D) iPoint, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint3D(array);

        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iArray[2] = hPoint.getInhomZ() + 1.0;
        iPoint = new InhomogeneousPoint3D(iArray);
        assertTrue(hPoint.equals(iPoint, 1.1));
        assertTrue(hPoint.equals((Point3D) iPoint, 1.1));
        assertFalse(hPoint.equals(iPoint, 0.5));
        assertFalse(hPoint.equals((Point3D) iPoint, 0.5));

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
            value = hPoint.equals((Point3D) iPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertFalse(value);
    }

    @Test
    public void testIsAtInfinity() {
        final HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        // sets point at infinity
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
        final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final HomogeneousPoint3D point = new HomogeneousPoint3D(homX, homY, homZ,
                homW);
        assertEquals(point.getHomX(), homX, 0.0);
        assertEquals(point.getHomY(), homY, 0.0);
        assertEquals(point.getHomZ(), homZ, 0.0);
        assertEquals(point.getHomW(), homW, 0.0);
        assertFalse(point.isNormalized());

        // compute norm
        final double norm = Math.sqrt(homX * homX + homY * homY + homZ * homZ +
                homW * homW);

        // normalize
        point.normalize();
        assertTrue(point.isNormalized());

        // check correctness after normalization
        assertEquals(point.getHomX(), homX / norm, ABSOLUTE_ERROR);
        assertEquals(point.getHomY(), homY / norm, ABSOLUTE_ERROR);
        assertEquals(point.getHomZ(), homZ / norm, ABSOLUTE_ERROR);
        assertEquals(point.getHomW(), homW / norm, ABSOLUTE_ERROR);

        point.setHomogeneousCoordinates(homX, homY, homZ, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setCoordinates(homX, homY, homZ, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setInhomogeneousCoordinates(homX / homW, homY / homW,
                homZ / homW);
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

        final HomogeneousPoint3D point2 = new HomogeneousPoint3D();
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

        point.setZ(homZ);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setW(homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
    }
}
