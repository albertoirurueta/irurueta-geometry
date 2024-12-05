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

import static org.junit.jupiter.api.Assertions.*;

class InhomogeneousPoint3DTest {

    private static final int INHOM_COORDS = 3;
    private static final int HOM_COORDS = 4;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        var iPoint = new InhomogeneousPoint3D();
        assertEquals(0.0, iPoint.getInhomX(), 0.0);
        assertEquals(0.0, iPoint.getInhomY(), 0.0);
        assertEquals(0.0, iPoint.getInhomZ(), 0.0);

        var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D(array);
        final var array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iPoint = new InhomogeneousPoint3D(a, b, c);
        array = iPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);

        var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        iPoint = new InhomogeneousPoint3D(point);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, iPoint.getType());

        point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        iPoint = new InhomogeneousPoint3D(point);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, iPoint.getType());
    }

    @Test
    void testGettersAndSetters() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint3D();
        iPoint.setX(x);
        iPoint.setY(y);
        iPoint.setZ(z);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        assertEquals(iPoint.getZ(), z, 0.0);

        iPoint.setHomogeneousCoordinates(homX, homY, homZ, homW);
        final var constantX = iPoint.getHomX() / homX;
        final var constantY = iPoint.getHomY() / homY;
        final var constantZ = iPoint.getHomZ() / homZ;
        final var constantW = iPoint.getHomW() / homW;
        assertEquals(constantX, constantY, ABSOLUTE_ERROR);
        assertEquals(constantY, constantZ, ABSOLUTE_ERROR);
        assertEquals(constantZ, constantW, ABSOLUTE_ERROR);
        assertEquals(constantW, constantX, ABSOLUTE_ERROR);

        iPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(inhomX, iPoint.getInhomX(), 0.0);
        assertEquals(inhomY, iPoint.getInhomY(), 0.0);
        assertEquals(inhomZ, iPoint.getInhomZ(), 0.0);
        assertEquals(inhomX, iPoint.getX(), 0.0);
        assertEquals(inhomY, iPoint.getY(), 0.0);
        assertEquals(inhomZ, iPoint.getZ(), 0.0);
    }

    @Test
    void testToHomogeneous() {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint3D(array);
        final var hPoint = iPoint.toHomogeneous();

        // check that inhomogeneous coordinates are almost equal
        assertEquals(hPoint.getInhomX(), iPoint.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomY(), iPoint.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomZ(), iPoint.getInhomZ(), ABSOLUTE_ERROR);

        // check that homogeneous coordinates are up to scale
        final var scaleX = hPoint.getHomX() / iPoint.getHomX();
        final var scaleY = hPoint.getHomY() / iPoint.getHomY();
        final var scaleZ = hPoint.getHomZ() / iPoint.getHomZ();
        final var scaleW = hPoint.getHomW() / iPoint.getHomW();

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    void testSetCoordinates() {
        var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(array);
        var array2 = iPoint.asArray();

        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        final var array3 = new double[INHOM_COORDS + 1];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        assertThrows(IllegalArgumentException.class, () -> iPoint.setCoordinates(array3));

        final var array4 = new double[INHOM_COORDS - 1];
        randomizer.fill(array4, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        assertThrows(IllegalArgumentException.class, () -> iPoint.setCoordinates(array4));

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint3 = new InhomogeneousPoint3D();
        iPoint3.setCoordinates(x, y, z);
        assertEquals(x, iPoint3.getX(), 0.0);
        assertEquals(y, iPoint3.getY(), 0.0);
        assertEquals(z, iPoint3.getZ(), 0.0);

        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint4 = new InhomogeneousPoint3D();
        iPoint4.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(inhomX, iPoint4.getInhomX(), 0.0);
        assertEquals(inhomY, iPoint4.getInhomY(), 0.0);
        assertEquals(inhomZ, iPoint4.getInhomZ(), 0.0);
        assertEquals(inhomX, iPoint4.getX(), 0.0);
        assertEquals(inhomY, iPoint4.getY(), 0.0);
        assertEquals(inhomZ, iPoint4.getZ(), 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint2 = new InhomogeneousPoint3D(array);
        final var iPoint5 = new InhomogeneousPoint3D();
        iPoint5.setCoordinates(iPoint2);

        array2 = iPoint5.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var hPoint = new HomogeneousPoint3D(array);
        final var iPoint6 = new InhomogeneousPoint3D();
        iPoint6.setCoordinates(hPoint);

        array2 = iPoint6.asArray();
        assertEquals(array[0] / array[3], array2[0], 0.0);
        assertEquals(array[1] / array[3], array2[1], 0.0);
        assertEquals(array[2] / array[3], array2[2], 0.0);
    }

    @Test
    void testAsArray() {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint = new InhomogeneousPoint3D(array);
        var array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[INHOM_COORDS];
        iPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    void testEqualsAndHashCode() {
        var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var iPoint1 = new InhomogeneousPoint3D(array);
        var iPoint2 = new InhomogeneousPoint3D(array);

        assertTrue(iPoint1.equals(iPoint2, 0.0));
        assertTrue(iPoint1.equals((Point3D) iPoint2, 0.0));
        assertEquals(iPoint1.hashCode(), iPoint2.hashCode());

        array[0] = iPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final var iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[3];
        iArray[1] = array[1] / array[3];
        iArray[2] = array[2] / array[3];

        final var iPoint3 = new InhomogeneousPoint3D(iArray);

        var hPoint = new HomogeneousPoint3D(array);

        assertTrue(iPoint3.equals(hPoint, ABSOLUTE_ERROR));
        assertTrue(iPoint3.equals((Point3D) hPoint, ABSOLUTE_ERROR));

        array[0] = iPoint3.getHomX() + 1.0;
        array[1] = iPoint3.getHomY() + 1.0;
        array[2] = iPoint3.getHomZ() + 1.0;
        array[3] = iPoint3.getHomW() + 1.0;
        hPoint = new HomogeneousPoint3D(array);
        assertFalse(iPoint3.equals(hPoint, 0.0));
        assertFalse(iPoint3.equals((Point3D) hPoint, 0.0));

        iArray[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iArray[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iArray[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint1 = new InhomogeneousPoint3D(iArray);

        array[0] = iPoint1.getHomX() + iPoint1.getHomW();
        array[1] = iPoint1.getHomY() + iPoint1.getHomW();
        array[2] = iPoint1.getHomZ() + iPoint1.getHomW();
        array[3] = iPoint1.getHomW();
        final var hPoint2 = new HomogeneousPoint3D(array);
        assertTrue(iPoint1.equals(hPoint2, 1.0 + ABSOLUTE_ERROR));
        assertTrue(iPoint1.equals((Point3D) hPoint2, 1.0 + ABSOLUTE_ERROR));
        assertFalse(iPoint1.equals(hPoint2, 0.5));
        assertFalse(iPoint1.equals((Point3D) hPoint2, 0.5));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> iPoint3.equals(hPoint2, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> iPoint3.equals(iPoint3, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> iPoint3.equals((Point3D) iPoint3, -ABSOLUTE_ERROR));

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
    void testIsAtInfinity() {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var iPoint = new InhomogeneousPoint3D(array);
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
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint1 = new InhomogeneousPoint3D(array);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(iPoint1);
        final var iPoint2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(iPoint1, iPoint2);
        assertNotSame(iPoint1, iPoint2);
    }
}
