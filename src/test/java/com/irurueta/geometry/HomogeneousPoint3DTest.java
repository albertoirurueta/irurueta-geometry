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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class HomogeneousPoint3DTest {

    private static final int HOM_COORDS = 4;
    private static final int INHOM_COORDS = 3;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        var hPoint = new HomogeneousPoint3D();
        assertEquals(0.0, hPoint.getHomX(), 0.0);
        assertEquals(0.0, hPoint.getHomY(), 0.0);
        assertEquals(0.0, hPoint.getHomZ(), 0.0);
        assertEquals(0.0, hPoint.getHomW(), 1.0);
        assertFalse(hPoint.isNormalized());

        var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint = new HomogeneousPoint3D(array);

        final var array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        assertFalse(hPoint.isNormalized());

        final var a = randomizer.nextDouble();
        final var b = randomizer.nextDouble();
        final var c = randomizer.nextDouble();
        final var d = randomizer.nextDouble();

        hPoint = new HomogeneousPoint3D(a, b, c, d);
        array = hPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertEquals(d, array[3], 0.0);
        assertFalse(hPoint.isNormalized());

        var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        hPoint = new HomogeneousPoint3D(point);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, hPoint.getType());
        assertFalse(hPoint.isNormalized());

        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        hPoint = new HomogeneousPoint3D(point);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, hPoint.getType());
        assertFalse(hPoint.isNormalized());
    }

    @Test
    void testGettersAndSetters() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble();
        final var y = randomizer.nextDouble();
        final var z = randomizer.nextDouble();
        final var w = randomizer.nextDouble();
        final var homX = randomizer.nextDouble();
        final var homY = randomizer.nextDouble();
        final var homZ = randomizer.nextDouble();
        final var homW = randomizer.nextDouble();
        final var inhomX = randomizer.nextDouble();
        final var inhomY = randomizer.nextDouble();
        final var inhomZ = randomizer.nextDouble();

        final var hPoint = new HomogeneousPoint3D();
        hPoint.setX(x);
        hPoint.setY(y);
        hPoint.setZ(z);
        hPoint.setW(w);
        assertEquals(x, hPoint.getX(), 0.0);
        assertEquals(y, hPoint.getY(), 0.0);
        assertEquals(z, hPoint.getZ(), 0.0);
        assertEquals(w, hPoint.getW(), 0.0);

        hPoint.setHomogeneousCoordinates(homX, homY, homZ, homW);
        assertEquals(homX, hPoint.getHomX(), 0.0);
        assertEquals(homY, hPoint.getHomY(), 0.0);
        assertEquals(homZ, hPoint.getHomZ(), 0.0);
        assertEquals(homW, hPoint.getHomW(), 0.0);
        assertEquals(homX, hPoint.getX(), 0.0);
        assertEquals(homY, hPoint.getY(), 0.0);
        assertEquals(homW, hPoint.getW(), 0.0);

        hPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(inhomX, hPoint.getInhomX(), 0.0);
        assertEquals(inhomY, hPoint.getInhomY(), 0.0);
        assertEquals(inhomZ, hPoint.getInhomZ(), 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
        assertEquals(hPoint.getZ() / hPoint.getW(), inhomZ, 0.0);
    }

    @Test
    void testToInhomogeneous() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint = new HomogeneousPoint3D(array);

        final var iPoint = hPoint.toInhomogeneous();

        assertEquals(hPoint.getX() / hPoint.getW(), iPoint.getX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getY() / hPoint.getW(), iPoint.getY(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getZ() / hPoint.getW(), iPoint.getZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetCoordinates() {
        var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint = new HomogeneousPoint3D();
        hPoint.setCoordinates(array);
        var array2 = hPoint.asArray();

        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        final var array3 = new double[HOM_COORDS + 1];
        final var hPoint3 = new HomogeneousPoint3D();
        assertThrows(IllegalArgumentException.class, () -> hPoint3.setCoordinates(array3));

        final var array4 = new double[HOM_COORDS - 1];
        final var hPoint4 = new HomogeneousPoint3D();
        assertThrows(IllegalArgumentException.class, () -> hPoint4.setCoordinates(array4));

        final var x = randomizer.nextDouble();
        final var y = randomizer.nextDouble();
        final var z = randomizer.nextDouble();
        final var w = randomizer.nextDouble();
        final var hPoint5 = new HomogeneousPoint3D();
        hPoint5.setCoordinates(x, y, z, w);
        assertEquals(x, hPoint5.getX(), 0.0);
        assertEquals(y, hPoint5.getY(), 0.0);
        assertEquals(z, hPoint5.getZ(), 0.0);
        assertEquals(w, hPoint5.getW(), 0.0);

        final var inhomX = randomizer.nextDouble();
        final var inhomY = randomizer.nextDouble();
        final var inhomZ = randomizer.nextDouble();
        final var hPoint6 = new HomogeneousPoint3D();
        hPoint6.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(inhomX, hPoint6.getInhomX(), 0.0);
        assertEquals(inhomY, hPoint6.getInhomY(), 0.0);
        assertEquals(inhomZ, hPoint6.getInhomZ(), 0.0);
        assertEquals(hPoint6.getX() / hPoint6.getW(), inhomX, 0.0);
        assertEquals(hPoint6.getY() / hPoint6.getW(), inhomY, 0.0);
        assertEquals(hPoint6.getZ() / hPoint6.getW(), inhomZ, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hPoint2 = new HomogeneousPoint3D(array);
        final var hPoint7 = new HomogeneousPoint3D();
        // pass another point to set coordinates
        hPoint7.setCoordinates(hPoint2);
        array2 = hPoint7.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint = new InhomogeneousPoint3D(array);
        final var hPoint8 = new HomogeneousPoint3D();
        // pass another point to set coordinates
        hPoint8.setCoordinates(iPoint);
        array2 = hPoint8.asArray();
        assertEquals(array[0], array2[0], 0.0);
        assertEquals(array[1], array2[1], 0.0);
        assertEquals(array[2], array2[2], 0.0);
        assertEquals(1.0, array2[3], 0.0);
    }

    @Test
    void testAsArray() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint = new HomogeneousPoint3D(array);
        var array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[HOM_COORDS];
        hPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    void testEqualsAndHashCode() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var hPoint1 = new HomogeneousPoint3D(array);
        var hPoint2 = new HomogeneousPoint3D(array);
        assertTrue(hPoint1.equals(hPoint2, 0.0));
        assertTrue(hPoint1.equals((Point3D) hPoint2, 0.0));
        assertEquals(hPoint1.hashCode(), hPoint2.hashCode());

        array[0] = hPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
        final var iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[3];
        iArray[1] = array[1] / array[3];
        iArray[2] = array[2] / array[3];

        final var hPoint = new HomogeneousPoint3D(array);
        final var iPoint = new InhomogeneousPoint3D(iArray);
        assertTrue(hPoint.equals(iPoint, ABSOLUTE_ERROR));
        assertTrue(hPoint.equals((Point3D) iPoint, ABSOLUTE_ERROR));

        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iArray[2] = hPoint.getInhomZ() + 1.0;
        final var iPoint2 = new InhomogeneousPoint3D(iArray);
        assertFalse(hPoint.equals(iPoint2, 0.0));
        assertFalse(hPoint.equals((Point3D) iPoint2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint2 = new HomogeneousPoint3D(array);

        iArray[0] = hPoint2.getInhomX() + 1.0;
        iArray[1] = hPoint2.getInhomY() + 1.0;
        iArray[2] = hPoint2.getInhomZ() + 1.0;
        final var iPoint3 = new InhomogeneousPoint3D(iArray);
        assertTrue(hPoint2.equals(iPoint3, 1.1));
        assertTrue(hPoint2.equals((Point3D) iPoint3, 1.1));
        assertFalse(hPoint2.equals(iPoint3, 0.5));
        assertFalse(hPoint2.equals((Point3D) iPoint3, 0.5));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> hPoint.equals(hPoint, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> hPoint.equals(iPoint, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> hPoint.equals((Point3D) iPoint, -ABSOLUTE_ERROR));

        final var hPoint3 = new HomogeneousPoint3D();
        final var iPoint4 = new InhomogeneousPoint3D();
        assertTrue(hPoint3.equals(iPoint4));
        assertEquals(hPoint3.hashCode(), iPoint4.hashCode());
        hPoint2 = new HomogeneousPoint3D(hPoint3);
        assertTrue(hPoint3.equals(hPoint2));
        assertEquals(hPoint3.hashCode(), hPoint2.hashCode());
        assertTrue(hPoint3.equals((Point3D) iPoint4));
        assertTrue(hPoint3.equals((Point3D) hPoint2));
    }

    @Test
    void testIsAtInfinity() {
        final var hPoint = new HomogeneousPoint3D();
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
        assertThrows(IllegalArgumentException.class, () -> hPoint.isAtInfinity(-ABSOLUTE_ERROR));
    }

    @Test
    void testNormalize() {
        final var randomizer = new UniformRandomizer();
        final var homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var point = new HomogeneousPoint3D(homX, homY, homZ, homW);
        assertEquals(homX, point.getHomX(), 0.0);
        assertEquals(homY, point.getHomY(), 0.0);
        assertEquals(homZ, point.getHomZ(), 0.0);
        assertEquals(homW, point.getHomW(), 0.0);
        assertFalse(point.isNormalized());

        // compute norm
        final var norm = Math.sqrt(homX * homX + homY * homY + homZ * homZ + homW * homW);

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

        point.setInhomogeneousCoordinates(homX / homW, homY / homW, homZ / homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        final var array = new double[HOM_COORDS];
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

        final var point2 = new HomogeneousPoint3D();
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

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint1 = new HomogeneousPoint3D(array);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(hPoint1);
        final var hPoint2 = SerializationHelper.<HomogeneousPoint3D>deserialize(bytes);

        // check
        assertEquals(hPoint1, hPoint2);
        assertNotSame(hPoint1, hPoint2);
    }
}
