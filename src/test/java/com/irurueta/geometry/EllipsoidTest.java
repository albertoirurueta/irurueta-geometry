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

class EllipsoidTest {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    @Test
    void testConstructor() {
        // test empty constructor.
        var ellipsoid = new Ellipsoid();

        // check center is at origin and axes are unitary
        assertTrue(ellipsoid.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0), ABSOLUTE_ERROR));
        assertArrayEquals(new double[]{1.0, 1.0, 1.0}, ellipsoid.getSemiAxesLengths(), ABSOLUTE_ERROR);
        assertEquals(ellipsoid.getRotation(), Rotation3D.create());

        // test constructor with center, axes and rotation
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Quaternion(roll, pitch, yaw);
        ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        // check correctness
        assertEquals(center, ellipsoid.getCenter());
        assertArrayEquals(semiAxesLengths, ellipsoid.getSemiAxesLengths(), 0.0);
        assertEquals(rotation, ellipsoid.getRotation());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Ellipsoid(center, new double[1], rotation));
    }

    @Test
    void testGetSetCenter() {
        final var ellipsoid = new Ellipsoid();

        // check default value
        assertTrue(ellipsoid.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0), ABSOLUTE_ERROR));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        ellipsoid.setCenter(center);

        // check correctness
        assertEquals(center, ellipsoid.getCenter());
    }

    @Test
    void testGetSetSemiAxesLengths() {
        final var ellipsoid = new Ellipsoid();

        // check default value
        assertArrayEquals(new double[]{1.0, 1.0, 1.0}, ellipsoid.getSemiAxesLengths(), ABSOLUTE_ERROR);

        // set new value
        final var axes = new double[Ellipsoid.DIMENSIONS];
        ellipsoid.setSemiAxesLengths(axes);

        // check correctness
        assertSame(axes, ellipsoid.getSemiAxesLengths());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ellipsoid.setSemiAxesLengths(new double[1]));
    }

    @Test
    void testGetSetRotation() {
        final var ellipsoid = new Ellipsoid();

        // check default value
        assertEquals(ellipsoid.getRotation(), Rotation3D.create());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Quaternion(roll, pitch, yaw);
        ellipsoid.setRotation(rotation);

        // check correctness
        assertEquals(rotation, ellipsoid.getRotation());
    }

    @Test
    void testSetCenterAxesAndRotation() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Quaternion(roll, pitch, yaw);
        final var ellipsoid = new Ellipsoid();
        ellipsoid.setCenterAxesAndRotation(center, semiAxesLengths, rotation);

        // check correctness
        assertEquals(center, ellipsoid.getCenter());
        assertArrayEquals(semiAxesLengths, ellipsoid.getSemiAxesLengths(), 0.0);
        assertEquals(rotation, ellipsoid.getRotation());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ellipsoid.setCenterAxesAndRotation(center, new double[1], rotation));
    }

    @Test
    void testGetVolume() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Quaternion(roll, pitch, yaw);
        final var ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        // check correctness
        final var a = semiAxesLengths[0];
        final var b = semiAxesLengths[1];
        final var c = semiAxesLengths[2];

        assertEquals(ellipsoid.getVolume(), 4.0 / 3.0 * Math.PI * a * b * c, ABSOLUTE_ERROR);

        // test from circle
        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var sphere = new Sphere(center, radius);
        ellipsoid.setFromSphere(sphere);

        assertEquals(sphere.getVolume(), ellipsoid.getVolume(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSurface() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Quaternion(roll, pitch, yaw);
        final var ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        // check correctness
        final var a = semiAxesLengths[0];
        final var b = semiAxesLengths[1];
        final var c = semiAxesLengths[2];

        final var p = 1.6075;
        assertEquals(4.0 * Math.PI * Math.pow(
                (Math.pow(a * b, p) + Math.pow(a * c, p) + Math.pow(b * c, p)) / 3.0, 1.0 / p), ellipsoid.getSurface(),
                ABSOLUTE_ERROR);

        // test from circle
        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var sphere = new Sphere(center, radius);
        ellipsoid.setFromSphere(sphere);

        assertEquals(sphere.getSurface(), ellipsoid.getSurface(), ABSOLUTE_ERROR);
    }

    @Test
    void testToQuadric() throws GeometryException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        Arrays.fill(semiAxesLengths, radius);
        final var rotation = Rotation3D.create();
        final var ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);
        final var quadric1 = ellipsoid.toQuadric();

        final var sphere = new Sphere(center, radius);
        final var quadric2 = sphere.toQuadric();

        quadric1.normalize();
        quadric2.normalize();

        assertEquals(Math.abs(quadric1.getA()), Math.abs(quadric2.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getB()), Math.abs(quadric2.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getC()), Math.abs(quadric2.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getD()), Math.abs(quadric2.getD()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getE()), Math.abs(quadric2.getE()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getF()), Math.abs(quadric2.getF()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getG()), Math.abs(quadric2.getG()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getH()), Math.abs(quadric2.getH()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getI()), Math.abs(quadric2.getI()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getJ()), Math.abs(quadric2.getJ()), ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Quaternion(roll, pitch, yaw);

        final var ellipsoid1 = new Ellipsoid(center, semiAxesLengths, rotation);

        // check
        assertSame(center, ellipsoid1.getCenter());
        assertSame(semiAxesLengths, ellipsoid1.getSemiAxesLengths());
        assertSame(rotation, ellipsoid1.getRotation());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(ellipsoid1);
        final var ellipsoid2 = SerializationHelper.<Ellipsoid>deserialize(bytes);

        // check
        assertEquals(ellipsoid1.getCenter(), ellipsoid2.getCenter());
        assertArrayEquals(ellipsoid1.getSemiAxesLengths(), ellipsoid2.getSemiAxesLengths(), 0.0);
        assertEquals(ellipsoid1.getRotation(), ellipsoid2.getRotation());
    }
}
