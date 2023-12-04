/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class SphereTest {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    @Test
    public void testConstructor() throws CoplanarPointsException {
        // Test empty constructor
        Sphere sphere = new Sphere();

        // check center is at origin and radius is 1.0;
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0,
                0.0), ABSOLUTE_ERROR));
        assertEquals(1.0, sphere.getRadius(), ABSOLUTE_ERROR);

        // Test constructor with center and radius
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        sphere = new Sphere(center, radius);
        // check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(sphere.getRadius(), radius, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        sphere = null;
        try {
            sphere = new Sphere(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(sphere);

        // test constructor with four points

        // pick 4 points belonging to the sphere locus
        Point3D point1;
        Point3D point2;
        Point3D point3;
        Point3D point4;
        boolean areEqual;
        do {
            double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point4 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);

            // ensure that all four points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point4, ABSOLUTE_ERROR) ||
                    point4.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);

        // compute sphere
        Sphere sphere2 = new Sphere(point1, point2, point3, point4);

        // check that both spheres are equal
        sphere = new Sphere(center, radius);
        assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force CoplanarPointsException
        sphere = null;
        try {
            sphere = new Sphere(point1, point2, point2, point4);
            fail("CoplanarPointsException expected but not thrown");
        } catch (final CoplanarPointsException ignore) {
        }
        assertNull(sphere);

        // test from quadric
        sphere = new Sphere(center, radius);
        Quadric quadric = sphere.toQuadric();
        sphere2 = new Sphere(quadric);
        assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        quadric = new Quadric();
        sphere = null;
        try {
            sphere = new Sphere(quadric);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(sphere);
    }

    @Test
    public void testGetSetCenter() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere();
        // check center
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0),
                ABSOLUTE_ERROR));

        // set center
        sphere.setCenter(center);
        // check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));

        // Force NullPointerException
        try {
            sphere.setCenter(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testGetSetRadius() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere();
        // check radius
        assertEquals(1.0, sphere.getRadius(), 0.0);

        // set radius
        sphere.setRadius(radius);
        // check correctness
        assertEquals(radius, sphere.getRadius(), 0.0);

        // Force IllegalArgumentException
        try {
            sphere.setRadius(-radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetCenterAndRadius() {
        // Test constructor with center and radius
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere();
        // check center
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0),
                ABSOLUTE_ERROR));
        // check radius
        assertEquals(1.0, sphere.getRadius(), 0.0);

        // set center and radius
        sphere.setCenterAndRadius(center, radius);
        // check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(radius, sphere.getRadius(), 0.0);

        // Force IllegalArgumentException
        try {
            sphere.setCenterAndRadius(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force NullPointerException
        try {
            sphere.setCenterAndRadius(null, radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testSetParametersFromPoints() throws CoplanarPointsException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE));

        final Sphere sphere1 = new Sphere(center, radius);

        // pick 4 points belonging to the sphere locus
        Point3D point1;
        Point3D point2;
        Point3D point3;
        Point3D point4;
        boolean areEqual;
        do {
            double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point4 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) *
                            Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) *
                            Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);

            // ensure that all four points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point4, ABSOLUTE_ERROR) ||
                    point4.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);

        // create new sphere and set parameters
        final Sphere sphere2 = new Sphere();

        sphere2.setParametersFromPoints(point1, point2, point3, point4);

        // check that both spheres are equal
        assertEquals(0.0, sphere1.getCenter().distanceTo(sphere2.getCenter()),
                ABSOLUTE_ERROR);
        assertEquals(sphere1.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force CoplanarPointsException
        try {
            sphere2.setParametersFromPoints(point1, point2, point2, point4);
            fail("CoplanarPointsException expected but not thrown");
        } catch (final CoplanarPointsException ignore) {
        }
    }

    @Test
    public void testVolume() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere(center, radius);

        final double volume = 4.0 / 3.0 * Math.PI * radius * radius * radius;

        // Check correctness
        assertEquals(volume, sphere.getVolume(), ABSOLUTE_ERROR);
        assertEquals(volume, Sphere.volume(radius), ABSOLUTE_ERROR);
    }

    @Test
    public void testSurface() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere(center, radius);

        final double surface = 4.0 * Math.PI * radius * radius;

        // Check correctness
        assertEquals(surface, sphere.getSurface(), ABSOLUTE_ERROR);
        assertEquals(surface, Sphere.surface(radius), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsInside() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double value = randomizer.nextDouble(0.2, 0.8);
        final double value2 = 1.0 + value;
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        final double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        final Sphere sphere = new Sphere(center, radius);

        final Point3D inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        final Point3D outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));

        // check correctness
        assertTrue(sphere.isInside(inside));
        assertTrue(sphere.isInside(inside, ABSOLUTE_ERROR));

        assertFalse(sphere.isInside(outside));
        assertFalse(sphere.isInside(outside, ABSOLUTE_ERROR));

        // test for a large positive threshold
        assertTrue(sphere.isInside(inside, radius));
        assertTrue(sphere.isInside(outside, radius));

        assertFalse(sphere.isInside(inside, -radius));
        assertFalse(sphere.isInside(outside, -radius));
    }

    @Test
    public void testSignedDistanceDistanceAndIsLocus() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double value = randomizer.nextDouble(0.2, 0.8);
        final double value2 = 1.0 + value;
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        final double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        final Sphere sphere = new Sphere(center, radius);

        // center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));

        final Point3D inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        final Point3D outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        final Point3D zero = new InhomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(phi));

        // check correctness
        assertEquals(sphere.getSignedDistance(inside), (value - 1.0) * radius, ABSOLUTE_ERROR);
        assertEquals(Sphere.signedDistance(sphere, inside), (value - 1.0) * radius, ABSOLUTE_ERROR);

        assertEquals(sphere.getDistance(inside), Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Sphere.distance(sphere, inside), Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);

        // for inside point signed distance is negative
        assertTrue(sphere.getSignedDistance(inside) <= 0.0);

        assertFalse(sphere.isLocus(inside));
        assertFalse(sphere.isLocus(inside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(sphere.isLocus(inside, radius));

        assertEquals(sphere.getSignedDistance(outside), (value2 - 1.0) * radius, ABSOLUTE_ERROR);
        assertEquals(Sphere.signedDistance(sphere, outside), (value2 - 1.0) * radius,
                ABSOLUTE_ERROR);

        assertEquals(sphere.getDistance(outside), Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Sphere.distance(sphere, outside), Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);

        // for outside point distance is positive
        assertTrue(sphere.getSignedDistance(outside) >= 0.0);

        assertFalse(sphere.isLocus(outside));
        assertFalse(sphere.isLocus(outside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(sphere.isLocus(outside, radius));

        // for point at locus of circle, distance is zero
        assertEquals(0.0, sphere.getSignedDistance(zero), ABSOLUTE_ERROR);
        assertEquals(0.0, Sphere.signedDistance(sphere, zero), ABSOLUTE_ERROR);

        // zero is locus
        assertTrue(sphere.isLocus(zero));
        assertTrue(sphere.isLocus(zero, radius));

        // Force IllegalArgumentException
        try {
            sphere.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testClosestPointAndIsLocus() throws UndefinedPointException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double value = randomizer.nextDouble(0.2, 0.8);
        final double value2 = 1.0 + value;
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        final double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        final Sphere sphere = new Sphere(center, radius);

        // center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));

        final Point3D inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        final Point3D outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        final Point3D zero = new InhomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(phi));

        final Point3D expectedPoint = new InhomogeneousPoint3D(zero);

        final Point3D result = Point3D.create();

        // test for point inside (but far from center)
        assertTrue(sphere.getClosestPoint(inside).equals(expectedPoint, ABSOLUTE_ERROR));
        sphere.closestPoint(inside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        assertFalse(sphere.isLocus(inside));
        assertFalse(sphere.isLocus(inside, ABSOLUTE_ERROR));
        assertTrue(sphere.isLocus(inside, radius)); // true for a large enough
        // threshold

        // test for point outside of circle
        assertTrue(sphere.getClosestPoint(outside).equals(expectedPoint, ABSOLUTE_ERROR));
        sphere.closestPoint(outside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        assertFalse(sphere.isLocus(outside));
        assertFalse(sphere.isLocus(outside, ABSOLUTE_ERROR));
        // true for a large enough threshold
        assertTrue(sphere.isLocus(outside, radius));

        // test for point in circle boundary
        assertTrue(sphere.getClosestPoint(zero).equals(expectedPoint, ABSOLUTE_ERROR));
        sphere.closestPoint(zero, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));

        // zero is locus
        assertTrue(sphere.isLocus(zero));
        assertTrue(sphere.isLocus(zero, radius));

        // Force UndefinedPointException (by testing at center)
        try {
            sphere.getClosestPoint(center);
            fail("UndefinedPointException expected but not thrown");
        } catch (final UndefinedPointException ignore) {
        }
        try {
            sphere.closestPoint(center, result);
            fail("UndefinedPointException expected but not thrown");
        } catch (final UndefinedPointException ignore) {
        }

        // Force IllegalArgumentException
        try {
            sphere.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetTangentPlaneAt() throws NotLocusException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere(center, radius);

        final double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        final double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        final Point3D point = new HomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(angle1) *
                        Math.cos(angle2),
                center.getInhomY() + radius * Math.sin(angle1) *
                        Math.cos(angle2),
                center.getInhomZ() + radius * Math.sin(angle2),
                1.0);
        point.normalize();

        assertTrue(sphere.isLocus(point));

        // find tangent plane at locus point
        final Plane plane = sphere.getTangentPlaneAt(point);

        // check that point is also at plane's locus
        assertTrue(plane.isLocus(point));

        final double[] directorVector = plane.getDirectorVector();

        final double[] pointVector = new double[]{
                point.getInhomX() - center.getInhomX(),
                point.getInhomY() - center.getInhomY(),
                point.getInhomZ() - center.getInhomZ()
        };

        // normalize both vectors
        final double norm1 = com.irurueta.algebra.Utils.normF(directorVector);
        final double norm2 = com.irurueta.algebra.Utils.normF(pointVector);

        final double[] vector1 = new double[3];
        final double[] vector2 = new double[3];
        ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm1, vector1);
        ArrayUtils.multiplyByScalar(pointVector, 1.0 / norm2, vector2);
        // check that both normalized vectors are equal
        assertArrayEquals(vector1, vector2, ABSOLUTE_ERROR);
    }

    @Test
    public void testToQuadric() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE));
        final double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        final double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        center.normalize();

        final Sphere sphere = new Sphere(center, radius);
        final Quadric quadric = sphere.toQuadric();

        // center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(quadric.isLocus(center));
        assertFalse(quadric.isLocus(center, ABSOLUTE_ERROR));
        // ut for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));
        assertTrue(quadric.isLocus(center, 2.0 * radius));

        final Point3D locus = new HomogeneousPoint3D(
                center.getInhomX() + radius * Math.sin(theta) * Math.cos(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(theta),
                1.0);
        locus.normalize();

        // check is locus
        assertTrue(sphere.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(locus, ABSOLUTE_ERROR));

        // check correctness of estimated quadric matrix (up to scale)
        final Matrix m = new Matrix(Quadric.BASEQUADRIC_MATRIX_ROW_SIZE,
                Quadric.BASEQUADRIC_MATRIX_COLUMN_SIZE);
        m.setElementAt(0, 0, 1.0);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(2, 0, 0.0);
        m.setElementAt(3, 0, -center.getInhomX());

        m.setElementAt(0, 1, 0.0);
        m.setElementAt(1, 1, 1.0);
        m.setElementAt(2, 1, 0.0);
        m.setElementAt(3, 1, -center.getInhomY());

        m.setElementAt(0, 2, 0.0);
        m.setElementAt(1, 2, 0.0);
        m.setElementAt(2, 2, 1.0);
        m.setElementAt(3, 2, -center.getInhomZ());

        m.setElementAt(0, 3, -center.getInhomX());
        m.setElementAt(1, 3, -center.getInhomY());
        m.setElementAt(2, 3, -center.getInhomZ());
        m.setElementAt(3, 3, center.getInhomX() * center.getInhomX() +
                center.getInhomY() * center.getInhomY() +
                center.getInhomZ() * center.getInhomZ() - radius * radius);

        final double norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        assertTrue(m.equals(quadric.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testSetFromQuadric() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Sphere sphere = new Sphere(center, radius);

        // test from quadric
        Quadric quadric = sphere.toQuadric();
        final Sphere sphere2 = new Sphere();
        sphere2.setFromQuadric(quadric);

        // check correctness
        assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        quadric = new Quadric();
        try {
            sphere2.setFromQuadric(quadric);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Sphere sphere1 = new Sphere(center, radius);

        // check
        assertSame(center, sphere1.getCenter());
        assertEquals(radius, sphere1.getRadius(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(sphere1);
        final Sphere sphere2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(sphere1.getCenter(), sphere2.getCenter());
        assertEquals(sphere1.getRadius(), sphere2.getRadius(), 0.0);
    }
}
