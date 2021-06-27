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
import java.util.Random;

import static org.junit.Assert.*;

public class InhomogeneousPoint3DTest {

    private static final int INHOM_COORDS = 3;
    private static final int HOM_COORDS = 4;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    public void testConstructor() {
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D();
        assertEquals(iPoint.getInhomX(), 0.0, 0.0);
        assertEquals(iPoint.getInhomY(), 0.0, 0.0);
        assertEquals(iPoint.getInhomZ(), 0.0, 0.0);

        double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D(array);
        final double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iPoint = new InhomogeneousPoint3D(a, b, c);
        array = iPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);

        Point3D point = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        iPoint = new InhomogeneousPoint3D(point);
        assertEquals(iPoint.getType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);

        point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        iPoint = new InhomogeneousPoint3D(point);
        assertEquals(iPoint.getType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
    }

    @Test
    public void testGettersAndSetters() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D();
        iPoint.setX(x);
        iPoint.setY(y);
        iPoint.setZ(z);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        assertEquals(iPoint.getZ(), z, 0.0);

        iPoint.setHomogeneousCoordinates(homX, homY, homZ, homW);
        final double constantX = iPoint.getHomX() / homX;
        final double constantY = iPoint.getHomY() / homY;
        final double constantZ = iPoint.getHomZ() / homZ;
        final double constantW = iPoint.getHomW() / homW;
        assertEquals(constantX, constantY, ABSOLUTE_ERROR);
        assertEquals(constantY, constantZ, ABSOLUTE_ERROR);
        assertEquals(constantZ, constantW, ABSOLUTE_ERROR);
        assertEquals(constantW, constantX, ABSOLUTE_ERROR);

        iPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);
        assertEquals(iPoint.getZ(), inhomZ, 0.0);
    }

    @Test
    public void testToHomogeneous() {
        final double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        final HomogeneousPoint3D hPoint = iPoint.toHomogeneous();

        // check that inhomogeneous coordinates are almost equal
        assertEquals(hPoint.getInhomX(), iPoint.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomY(), iPoint.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomZ(), iPoint.getInhomZ(), ABSOLUTE_ERROR);

        // check that homogeneous coordinates are up to scale
        final double scaleX = hPoint.getHomX() / iPoint.getHomX();
        final double scaleY = hPoint.getHomY() / iPoint.getHomY();
        final double scaleZ = hPoint.getHomZ() / iPoint.getHomZ();
        final double scaleW = hPoint.getHomW() / iPoint.getHomW();

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    public void testSetCoordinates() {
        double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(array);
        double[] array2 = iPoint.asArray();

        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        array = new double[INHOM_COORDS + 1];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        try {
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        array = new double[INHOM_COORDS - 1];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        try {
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(x, y, z);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        assertEquals(iPoint.getZ(), z, 0.0);

        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);
        assertEquals(iPoint.getZ(), inhomZ, 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D iPoint2 = new InhomogeneousPoint3D(array);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(iPoint2);

        array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(hPoint);

        array2 = iPoint.asArray();
        assertEquals(array[0] / array[3], array2[0], 0.0);
        assertEquals(array[1] / array[3], array2[1], 0.0);
        assertEquals(array[2] / array[3], array2[2], 0.0);
    }

    @Test
    public void testAsArray() {
        final double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[INHOM_COORDS];
        iPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    public void testEqualsAndHashCode() {
        double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        InhomogeneousPoint3D iPoint1 = new InhomogeneousPoint3D(array);
        InhomogeneousPoint3D iPoint2 = new InhomogeneousPoint3D(array);

        assertTrue(iPoint1.equals(iPoint2, 0.0));
        assertTrue(iPoint1.equals((Point3D) iPoint2, 0.0));
        assertEquals(iPoint1.hashCode(), iPoint2.hashCode());

        array[0] = iPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        array[1] = iPoint1.getY();
        array[2] = iPoint1.getZ();
        iPoint2 = new InhomogeneousPoint3D(array);
        assertFalse(iPoint1.equals(iPoint2, 0.0));
        assertFalse(iPoint1.equals((Point3D) iPoint2, 0.0));

        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint1 = new InhomogeneousPoint3D(array);
        array[0] += 1.0;
        iPoint2 = new InhomogeneousPoint3D(array);
        assertTrue(iPoint1.equals(iPoint2, 2.0));
        assertTrue(iPoint1.equals((Point3D) iPoint2, 2.0));
        assertFalse(iPoint1.equals(iPoint2, 0.5));
        assertFalse(iPoint1.equals((Point3D) iPoint2, 0.5));

        // Testing equals from one homogeneous point
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double[] iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[3];
        iArray[1] = array[1] / array[3];
        iArray[2] = array[2] / array[3];

        iPoint1 = new InhomogeneousPoint3D(iArray);

        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);

        assertTrue(iPoint1.equals(hPoint, ABSOLUTE_ERROR));
        assertTrue(iPoint1.equals((Point3D) hPoint, ABSOLUTE_ERROR));

        array[0] = iPoint1.getHomX() + 1.0;
        array[1] = iPoint1.getHomY() + 1.0;
        array[2] = iPoint1.getHomZ() + 1.0;
        array[3] = iPoint1.getHomW() + 1.0;
        hPoint = new HomogeneousPoint3D(array);
        assertFalse(iPoint1.equals(hPoint, 0.0));
        assertFalse(iPoint1.equals((Point3D) hPoint, 0.0));

        iArray[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iArray[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iArray[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint1 = new InhomogeneousPoint3D(iArray);

        array[0] = iPoint1.getHomX() + iPoint1.getHomW();
        array[1] = iPoint1.getHomY() + iPoint1.getHomW();
        array[2] = iPoint1.getHomZ() + iPoint1.getHomW();
        array[3] = iPoint1.getHomW();
        hPoint = new HomogeneousPoint3D(array);
        assertTrue(iPoint1.equals(hPoint, 1.0 + ABSOLUTE_ERROR));
        assertTrue(iPoint1.equals((Point3D) hPoint, 1.0 + ABSOLUTE_ERROR));
        assertFalse(iPoint1.equals(hPoint, 0.5));
        assertFalse(iPoint1.equals((Point3D) hPoint, 0.5));

        // Force IllegalArgumentException
        boolean value = false;
        try {
            value = iPoint1.equals(hPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = iPoint1.equals(iPoint1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            value = iPoint1.equals((Point3D) iPoint1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertFalse(value);

        hPoint = new HomogeneousPoint3D();
        iPoint1 = new InhomogeneousPoint3D();
        assertTrue(iPoint1.equals(hPoint));
        assertEquals(iPoint1.hashCode(), hPoint.hashCode());
        iPoint2 = new InhomogeneousPoint3D(iPoint1);
        assertTrue(iPoint1.equals(iPoint2));
        assertEquals(iPoint1.hashCode(), iPoint2.hashCode());
        assertTrue(iPoint1.equals((Point3D) hPoint));
        assertTrue(iPoint1.equals((Point3D) iPoint2));
    }

    @Test
    public void testIsAtInfinity() {
        final double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        iPoint.setX(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setY(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setZ(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setX(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setY(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setZ(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setX(Double.NaN);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setY(Double.NaN);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setZ(Double.NaN);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint3D(array);
        assertFalse(iPoint.isAtInfinity());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        double[] array = new double[INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D iPoint1 = new InhomogeneousPoint3D(array);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(iPoint1);
        final InhomogeneousPoint3D iPoint2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(iPoint1, iPoint2);
        assertNotSame(iPoint1, iPoint2);
    }
}
