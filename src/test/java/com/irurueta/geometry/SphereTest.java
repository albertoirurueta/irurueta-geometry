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
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class SphereTest {

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;

    @Test
    void testConstructor() throws CoplanarPointsException {
        // Test empty constructor
        var sphere = new Sphere();

        // check center is at origin and radius is 1.0
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0), ABSOLUTE_ERROR));
        assertEquals(1.0, sphere.getRadius(), ABSOLUTE_ERROR);

        // Test constructor with center and radius
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        sphere = new Sphere(center, radius);
        // check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(radius, sphere.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Sphere(center, -radius));

        // test constructor with four points

        // pick 4 points belonging to the sphere locus
        Point3D point1;
        Point3D point2;
        Point3D point3;
        Point3D point4;
        boolean areEqual;
        do {
            var angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            var angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point1 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point2 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point3 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point4 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);

            // ensure that all four points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) || point2.equals(point3, ABSOLUTE_ERROR)
                    || point3.equals(point4, ABSOLUTE_ERROR) || point4.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);

        // compute sphere
        var sphere2 = new Sphere(point1, point2, point3, point4);

        // check that both spheres are equal
        sphere = new Sphere(center, radius);
        assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force CoplanarPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint4 = point4;
        assertThrows(CoplanarPointsException.class,
                () -> new Sphere(finalPoint1, finalPoint2, finalPoint2, finalPoint4));

        // test from quadric
        sphere = new Sphere(center, radius);
        final var quadric1 = sphere.toQuadric();
        sphere2 = new Sphere(quadric1);
        assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var quadric2 = new Quadric();
        assertThrows(IllegalArgumentException.class, () -> new Sphere(quadric2));
    }

    @Test
    void testGetSetCenter() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere();
        // check center
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0), ABSOLUTE_ERROR));

        // set center
        sphere.setCenter(center);
        // check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> sphere.setCenter(null));
    }

    @Test
    void testGetSetRadius() {
        final var randomizer = new UniformRandomizer();
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere();
        // check radius
        assertEquals(1.0, sphere.getRadius(), 0.0);

        // set radius
        sphere.setRadius(radius);
        // check correctness
        assertEquals(radius, sphere.getRadius(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> sphere.setRadius(-radius));
    }

    @Test
    void testSetCenterAndRadius() {
        // Test constructor with center and radius
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere();
        // check center
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0, 0.0), ABSOLUTE_ERROR));
        // check radius
        assertEquals(1.0, sphere.getRadius(), 0.0);

        // set center and radius
        sphere.setCenterAndRadius(center, radius);
        // check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(radius, sphere.getRadius(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> sphere.setCenterAndRadius(center, -radius));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> sphere.setCenterAndRadius(null, radius));
    }

    @Test
    void testSetParametersFromPoints() throws CoplanarPointsException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

        final var sphere1 = new Sphere(center, radius);

        // pick 4 points belonging to the sphere locus
        Point3D point1;
        Point3D point2;
        Point3D point3;
        Point3D point4;
        boolean areEqual;
        do {
            var angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            var angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point1 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point2 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point3 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);
            angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            point4 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2),
                    1.0);

            // ensure that all four points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) || point2.equals(point3, ABSOLUTE_ERROR)
                    || point3.equals(point4, ABSOLUTE_ERROR) || point4.equals(point1, ABSOLUTE_ERROR);
        } while (areEqual);

        // create new sphere and set parameters
        final var sphere2 = new Sphere();

        sphere2.setParametersFromPoints(point1, point2, point3, point4);

        // check that both spheres are equal
        assertEquals(0.0, sphere1.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere1.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force CoplanarPointsException
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint4 = point4;
        assertThrows(CoplanarPointsException.class,
                () -> sphere2.setParametersFromPoints(finalPoint1, finalPoint2, finalPoint2, finalPoint4));
    }

    @Test
    void testVolume() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere(center, radius);

        final var volume = 4.0 / 3.0 * Math.PI * radius * radius * radius;

        // Check correctness
        assertEquals(volume, sphere.getVolume(), ABSOLUTE_ERROR);
        assertEquals(volume, Sphere.volume(radius), ABSOLUTE_ERROR);
    }

    @Test
    void testSurface() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere(center, radius);

        final var surface = 4.0 * Math.PI * radius * radius;

        // Check correctness
        assertEquals(surface, sphere.getSurface(), ABSOLUTE_ERROR);
        assertEquals(surface, Sphere.surface(radius), ABSOLUTE_ERROR);
    }

    @Test
    void testIsInside() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var phi = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var sphere = new Sphere(center, radius);

        final var inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        final var outside = new InhomogeneousPoint3D(
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
    void testSignedDistanceDistanceAndIsLocus() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var phi = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var sphere = new Sphere(center, radius);

        // center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));

        final var inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        final var outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        final var zero = new InhomogeneousPoint3D(
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
        assertEquals(Sphere.signedDistance(sphere, outside), (value2 - 1.0) * radius, ABSOLUTE_ERROR);

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
        assertThrows(IllegalArgumentException.class, () -> sphere.isLocus(zero, -radius));
    }

    @Test
    void testClosestPointAndIsLocus() throws UndefinedPointException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var phi = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var sphere = new Sphere(center, radius);

        // center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        // but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));

        final var inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        final var outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        final var zero = new InhomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(phi));

        final var expectedPoint = new InhomogeneousPoint3D(zero);

        final var result = Point3D.create();

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
        assertThrows(UndefinedPointException.class, () -> sphere.getClosestPoint(center));
        assertThrows(UndefinedPointException.class, () -> sphere.closestPoint(center, result));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> sphere.isLocus(zero, -radius));
    }

    @Test
    void testGetTangentPlaneAt() throws NotLocusException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere(center, radius);

        final var angle1 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var angle2 = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var point = new HomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(angle1) * Math.cos(angle2),
                center.getInhomY() + radius * Math.sin(angle1) * Math.cos(angle2),
                center.getInhomZ() + radius * Math.sin(angle2),
                1.0);
        point.normalize();

        assertTrue(sphere.isLocus(point));

        // find tangent plane at locus point
        final var plane = sphere.getTangentPlaneAt(point);

        // check that point is also at plane's locus
        assertTrue(plane.isLocus(point));

        final var directorVector = plane.getDirectorVector();

        final var pointVector = new double[]{
                point.getInhomX() - center.getInhomX(),
                point.getInhomY() - center.getInhomY(),
                point.getInhomZ() - center.getInhomZ()
        };

        // normalize both vectors
        final var norm1 = com.irurueta.algebra.Utils.normF(directorVector);
        final var norm2 = com.irurueta.algebra.Utils.normF(pointVector);

        final var vector1 = new double[3];
        final var vector2 = new double[3];
        ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm1, vector1);
        ArrayUtils.multiplyByScalar(pointVector, 1.0 / norm2, vector2);
        // check that both normalized vectors are equal
        assertArrayEquals(vector1, vector2, ABSOLUTE_ERROR);
    }

    @Test
    void testToQuadric() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var center = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var phi = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        center.normalize();

        final var sphere = new Sphere(center, radius);
        final var quadric = sphere.toQuadric();

        // center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(quadric.isLocus(center));
        assertFalse(quadric.isLocus(center, ABSOLUTE_ERROR));
        // ut for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));
        assertTrue(quadric.isLocus(center, 2.0 * radius));

        final var locus = new HomogeneousPoint3D(
                center.getInhomX() + radius * Math.sin(theta) * Math.cos(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(theta),
                1.0);
        locus.normalize();

        // check is locus
        assertTrue(sphere.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(locus, ABSOLUTE_ERROR));

        // check correctness of estimated quadric matrix (up to scale)
        final var m = new Matrix(Quadric.BASEQUADRIC_MATRIX_ROW_SIZE, Quadric.BASEQUADRIC_MATRIX_COLUMN_SIZE);
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
        m.setElementAt(3, 3, center.getInhomX() * center.getInhomX()
                + center.getInhomY() * center.getInhomY() + center.getInhomZ() * center.getInhomZ() - radius * radius);

        final var norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        assertTrue(m.equals(quadric.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testSetFromQuadric() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere = new Sphere(center, radius);

        // test from quadric
        final var quadric1 = sphere.toQuadric();
        final var sphere2 = new Sphere();
        sphere2.setFromQuadric(quadric1);

        // check correctness
        assertEquals(0.0, sphere.getCenter().distanceTo(sphere2.getCenter()), ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var quadric2 = new Quadric();
        assertThrows(IllegalArgumentException.class, () -> sphere2.setFromQuadric(quadric2));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var sphere1 = new Sphere(center, radius);

        // check
        assertSame(center, sphere1.getCenter());
        assertEquals(radius, sphere1.getRadius(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(sphere1);
        final var sphere2 = SerializationHelper.<Sphere>deserialize(bytes);

        // check
        assertEquals(sphere1.getCenter(), sphere2.getCenter());
        assertEquals(sphere1.getRadius(), sphere2.getRadius(), 0.0);
    }
}
