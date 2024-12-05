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

class InhomogeneousPoint2DTest {

    private static final int HOM_COORDS = 3;
    private static final int INHOM_COORDS = 2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        var iPoint = new InhomogeneousPoint2D();
        assertEquals(0.0, iPoint.getInhomX(), 0.0);
        assertEquals(0.0, iPoint.getInhomY(), 0.0);

        final var randomizer = new UniformRandomizer();
        var array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iPoint = new InhomogeneousPoint2D(array);

        final var array2 = iPoint.asArray();

        assertArrayEquals(array, array2, 0.0);

        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iPoint = new InhomogeneousPoint2D(a, b);
        array = iPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);

        final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomogeneousCoordinates(a, b);
        iPoint = new InhomogeneousPoint2D(point);
        assertEquals(iPoint.getInhomX(), a, 0.0);
        assertEquals(iPoint.getInhomY(), b, 0.0);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, iPoint.getType());
    }

    @Test
    void testGettersAndSetters() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint2D();

        iPoint.setX(x);
        iPoint.setY(y);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);

        iPoint.setHomogeneousCoordinates(homX, homY, homW);
        final var constantX = iPoint.getHomX() / homX;
        final var constantY = iPoint.getHomY() / homY;
        final var constantW = iPoint.getHomW() / homW;
        assertEquals(constantX, constantY, ABSOLUTE_ERROR);
        assertEquals(constantY, constantW, ABSOLUTE_ERROR);
        assertEquals(constantW, constantX, ABSOLUTE_ERROR);

        iPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(inhomX, iPoint.getInhomX(), 0.0);
        assertEquals(inhomY, iPoint.getInhomY(), 0.0);
        assertEquals(inhomX, iPoint.getX(), 0.0);
        assertEquals(inhomY, iPoint.getY(), 0.0);
    }

    @Test
    void testToHomogeneous() {
        final var randomizer = new UniformRandomizer();
        final var array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint2D(array);

        final var hPoint = iPoint.toHomogeneous();

        // check that inhomogeneous coordinates are almost equal
        assertEquals(hPoint.getInhomX(), iPoint.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomY(), iPoint.getInhomY(), ABSOLUTE_ERROR);

        // check that homogeneous coordinates are up to scale
        final var scaleX = hPoint.getHomX() / iPoint.getHomX();
        final var scaleY = hPoint.getHomY() / iPoint.getHomY();
        final var scaleW = hPoint.getHomW() / iPoint.getHomW();

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }

    @Test
    void testSetCoordinates() {
        final var randomizer = new UniformRandomizer();
        var array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(array);
        var array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        final var array3 = new double[INHOM_COORDS + 1];
        final var iPoint3 = new InhomogeneousPoint2D();
        assertThrows(IllegalArgumentException.class, () -> iPoint3.setCoordinates(array3));

        final var array4 = new double[INHOM_COORDS - 1];
        final var iPoint4 = new InhomogeneousPoint2D();
        assertThrows(IllegalArgumentException.class, () -> iPoint4.setCoordinates(array4));

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint5 = new InhomogeneousPoint2D();
        iPoint5.setCoordinates(x, y);
        assertEquals(x, iPoint5.getX(), 0.0);
        assertEquals(y, iPoint5.getY(), 0.0);

        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint6 = new InhomogeneousPoint2D();
        iPoint6.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(inhomX, iPoint6.getInhomX(), 0.0);
        assertEquals(inhomY, iPoint6.getInhomY(), 0.0);
        assertEquals(inhomX, iPoint6.getX(), 0.0);
        assertEquals(inhomY, iPoint6.getY(), 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint2 = new InhomogeneousPoint2D(array);
        final var iPoint7 = new InhomogeneousPoint2D();
        // pass another point to set coordinates
        iPoint7.setCoordinates(iPoint2);
        array2 = iPoint7.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hPoint = new HomogeneousPoint2D(array);
        final var iPoint8 = new InhomogeneousPoint2D();
        iPoint8.setCoordinates(hPoint);
        array2 = iPoint8.asArray();
        assertEquals(HOM_COORDS, array.length);
        assertEquals(INHOM_COORDS, array2.length);
        assertEquals(array[0] / array[2], array2[0], 0.0);
        assertEquals(array[1] / array[2], array2[1], 0.0);
    }

    @Test
    void testArray() {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var iPoint = new InhomogeneousPoint2D(array);
        var array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[INHOM_COORDS];
        iPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    void testEqualsAndHashCode() {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var iPoint1 = new InhomogeneousPoint2D(array);
        var iPoint2 = new InhomogeneousPoint2D(array);
        assertTrue(iPoint1.equals(iPoint2, 0.0));
        assertTrue(iPoint1.equals((Point2D) iPoint2, 0.0));
        assertEquals(iPoint1.hashCode(), iPoint2.hashCode());

        array[0] = iPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = iPoint1.getY();
        iPoint2 = new InhomogeneousPoint2D(array);
        assertFalse(iPoint1.equals(iPoint2, 0.0));
        assertFalse(iPoint1.equals((Point2D) iPoint2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint1 = new InhomogeneousPoint2D(array);
        array[0] += 1.0;
        iPoint2 = new InhomogeneousPoint2D(array);
        assertTrue(iPoint1.equals(iPoint2, 1.0 + ABSOLUTE_ERROR));
        assertTrue(iPoint1.equals((Point2D) iPoint2, 1.0 + ABSOLUTE_ERROR));
        assertFalse(iPoint1.equals(iPoint2, 0.5));
        assertFalse(iPoint1.equals(iPoint2, 0.5));

        // Testing equals from one homogeneous point
        final var hArray = new double[HOM_COORDS];
        randomizer.fill(hArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[0] = hArray[0] / hArray[2];
        array[1] = hArray[1] / hArray[2];

        final var iPoint = new InhomogeneousPoint2D(array);
        var hPoint = new HomogeneousPoint2D(hArray);
        assertTrue(iPoint.equals(hPoint, ABSOLUTE_ERROR));
        assertTrue(iPoint.equals((Point2D) hPoint, ABSOLUTE_ERROR));

        hArray[0] = iPoint.getHomX() + 1.0;
        hArray[1] = iPoint.getHomY() + 1.0;
        hArray[2] = iPoint.getHomW() + 1.0;
        hPoint = new HomogeneousPoint2D(hArray);
        assertFalse(iPoint.equals(hPoint, 0.0));
        assertFalse(iPoint.equals((Point2D) hPoint, 0.0));

        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint3 = new InhomogeneousPoint2D(array);

        hArray[0] = iPoint3.getHomX() + iPoint3.getHomW();
        hArray[1] = iPoint3.getHomY() + iPoint3.getHomW();
        hArray[2] = iPoint3.getHomW();
        final var hPoint2 = new HomogeneousPoint2D(hArray);
        assertTrue(iPoint3.equals(hPoint2, 1.1));
        assertTrue(iPoint3.equals((Point2D) hPoint2, 1.1));
        assertFalse(iPoint3.equals(hPoint, 0.5));
        assertFalse(iPoint3.equals((Point2D) hPoint2, 0.5));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> iPoint.equals(hPoint2, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> iPoint.equals(iPoint3, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> iPoint.equals((Point2D) iPoint3, -ABSOLUTE_ERROR));

        hPoint = new HomogeneousPoint2D();
        final var iPoint4 = new InhomogeneousPoint2D();
        assertTrue(iPoint4.equals(hPoint));
        assertEquals(iPoint4.hashCode(), hPoint.hashCode());
        final var iPoint5 = new InhomogeneousPoint2D(iPoint);
        assertTrue(iPoint.equals(iPoint5));
        assertEquals(iPoint.hashCode(), iPoint5.hashCode());
        assertTrue(iPoint4.equals((Point2D) hPoint));
        assertTrue(iPoint.equals((Point2D) iPoint5));
    }

    @Test
    void testIsAtInfinity() {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var iPoint = new InhomogeneousPoint2D(array);
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

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var array = new double[INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint1 = new InhomogeneousPoint2D(array);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(iPoint1);
        final var iPoint2 = SerializationHelper.<InhomogeneousPoint2D>deserialize(bytes);

        // check
        assertEquals(iPoint1, iPoint2);
    }
}
