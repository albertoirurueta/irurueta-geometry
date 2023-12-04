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

public class EllipsoidTest {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    @Test
    public void testConstructor() {
        // test empty constructor.
        Ellipsoid ellipsoid = new Ellipsoid();

        // check center is at origin and axes are unitary
        assertTrue(ellipsoid.getCenter().equals(new InhomogeneousPoint3D(0.0,
                0.0, 0.0), ABSOLUTE_ERROR));
        assertArrayEquals(ellipsoid.getSemiAxesLengths(),
                new double[]{1.0, 1.0, 1.0}, ABSOLUTE_ERROR);
        assertEquals(ellipsoid.getRotation(), Rotation3D.create());

        // test constructor with center, axes and rotation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final Quaternion rotation = new Quaternion(roll, pitch, yaw);
        ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        // check correctness
        assertEquals(center, ellipsoid.getCenter());
        assertArrayEquals(semiAxesLengths, ellipsoid.getSemiAxesLengths(), 0.0);
        assertEquals(rotation, ellipsoid.getRotation());

        // Force IllegalArgumentException
        ellipsoid = null;
        try {
            ellipsoid = new Ellipsoid(center, new double[1], rotation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ellipsoid);
    }

    @Test
    public void testGetSetCenter() {
        final Ellipsoid ellipsoid = new Ellipsoid();

        // check default value
        assertTrue(ellipsoid.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0),
                ABSOLUTE_ERROR));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        ellipsoid.setCenter(center);

        // check correctness
        assertEquals(center, ellipsoid.getCenter());
    }

    @Test
    public void testGetSetSemiAxesLengths() {
        final Ellipsoid ellipsoid = new Ellipsoid();

        // check default value
        assertArrayEquals(ellipsoid.getSemiAxesLengths(),
                new double[]{1.0, 1.0, 1.0}, ABSOLUTE_ERROR);

        // set new value
        final double[] axes = new double[Ellipsoid.DIMENSIONS];
        ellipsoid.setSemiAxesLengths(axes);

        // check correctness
        assertSame(axes, ellipsoid.getSemiAxesLengths());

        // Force IllegalArgumentException
        try {
            ellipsoid.setSemiAxesLengths(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRotation() {
        final Ellipsoid ellipsoid = new Ellipsoid();

        // check default value
        assertEquals(ellipsoid.getRotation(), Rotation3D.create());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final Quaternion rotation = new Quaternion(roll, pitch, yaw);
        ellipsoid.setRotation(rotation);

        // check correctness
        assertEquals(rotation, ellipsoid.getRotation());
    }

    @Test
    public void testSetCenterAxesAndRotation() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final Quaternion rotation = new Quaternion(roll, pitch, yaw);
        final Ellipsoid ellipsoid = new Ellipsoid();
        ellipsoid.setCenterAxesAndRotation(center, semiAxesLengths, rotation);

        // check correctness
        assertEquals(center, ellipsoid.getCenter());
        assertArrayEquals(semiAxesLengths, ellipsoid.getSemiAxesLengths(), 0.0);
        assertEquals(rotation, ellipsoid.getRotation());

        // Force IllegalArgumentException
        try {
            ellipsoid.setCenterAxesAndRotation(center, new double[1], rotation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetVolume() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final Quaternion rotation = new Quaternion(roll, pitch, yaw);
        final Ellipsoid ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        // check correctness
        final double a = semiAxesLengths[0];
        final double b = semiAxesLengths[1];
        final double c = semiAxesLengths[2];

        assertEquals(ellipsoid.getVolume(), 4.0 / 3.0 * Math.PI * a * b * c,
                ABSOLUTE_ERROR);

        // test from circle
        final double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final Sphere sphere = new Sphere(center, radius);
        ellipsoid.setFromSphere(sphere);

        assertEquals(sphere.getVolume(), ellipsoid.getVolume(), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSurface() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final Quaternion rotation = new Quaternion(roll, pitch, yaw);
        final Ellipsoid ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        // check correctness
        final double a = semiAxesLengths[0];
        final double b = semiAxesLengths[1];
        final double c = semiAxesLengths[2];

        final double p = 1.6075;
        assertEquals(4.0 * Math.PI * Math.pow(
                (Math.pow(a * b, p) + Math.pow(a * c, p) + Math.pow(b * c, p)) / 3.0, 1.0 / p),
                ellipsoid.getSurface(),  ABSOLUTE_ERROR);

        // test from circle
        final double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final Sphere sphere = new Sphere(center, radius);
        ellipsoid.setFromSphere(sphere);

        assertEquals(sphere.getSurface(), ellipsoid.getSurface(), ABSOLUTE_ERROR);
    }

    @Test
    public void testToQuadric() throws GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        Arrays.fill(semiAxesLengths, radius);
        final Rotation3D rotation = Rotation3D.create();
        final Ellipsoid ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);
        final Quadric quadric1 = ellipsoid.toQuadric();

        final Sphere sphere = new Sphere(center, radius);
        final Quadric quadric2 = sphere.toQuadric();

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
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final Quaternion rotation = new Quaternion(roll, pitch, yaw);

        final Ellipsoid ellipsoid1 = new Ellipsoid(center, semiAxesLengths, rotation);

        // check
        assertSame(center, ellipsoid1.getCenter());
        assertSame(semiAxesLengths, ellipsoid1.getSemiAxesLengths());
        assertSame(rotation, ellipsoid1.getRotation());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(ellipsoid1);
        final Ellipsoid ellipsoid2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(ellipsoid1.getCenter(), ellipsoid2.getCenter());
        assertArrayEquals(ellipsoid1.getSemiAxesLengths(),
                ellipsoid2.getSemiAxesLengths(), 0.0);
        assertEquals(ellipsoid1.getRotation(), ellipsoid2.getRotation());
    }
}
