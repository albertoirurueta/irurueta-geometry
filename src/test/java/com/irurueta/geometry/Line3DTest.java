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

import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class Line3DTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final int INHOM_COORDS = 3;
    private static final int HOM_COORDS = 4;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testConstants() {
        assertEquals(1e-12, Line3D.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(0.0, Line3D.MIN_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException,
            CoincidentPlanesException, NoIntersectionException,
            CoincidentPointsException {

        // Create random homogeneous coordinates for a point
        Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final HomogeneousPoint3D point = new HomogeneousPoint3D(m.toArray());

        final Plane plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        final Plane plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));
        final Plane plane3 = new Plane(v.getSubmatrixAsArray(0, 3,
                3, 3));

        // ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));

        // test constructor from 2 planes
        Line3D line = new Line3D(plane1, plane2);

        assertEquals(plane1, line.getPlane1());
        assertEquals(plane2, line.getPlane2());

        assertTrue(line.isLocus(point));

        // Force ParallelPlanesException (by using the same plane)
        line = null;
        try {
            line = new Line3D(plane1, plane1);
            fail("CoincidentPlanesException expected but not thrown");
        } catch (final CoincidentPlanesException ignore) {
        }
        assertNull(line);


        // test constructor from 2 non-coincident points
        m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(m) < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0,
                0, 0, HOM_COORDS - 1));
        final Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1,
                0, 1, HOM_COORDS - 1));

        line = new Line3D(point1, point2);

        // ensure that both point1 and point2 belong to line 3D
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));

        // planes get defined, but we don't know their value, just that point1
        // and point2 belong to those planes too
        assertNotNull(line.getPlane1());
        assertNotNull(line.getPlane2());
        assertTrue(line.getPlane1().isLocus(point1));
        assertTrue(line.getPlane1().isLocus(point2));
        assertTrue(line.getPlane2().isLocus(point1));
        assertTrue(line.getPlane2().isLocus(point2));

        // Force CoincidentPointsException (by using the same point)
        line = null;
        try {
            line = new Line3D(point1, point1);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        assertNull(line);
    }

    @Test
    public void testAreParallelPlanes() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, NoIntersectionException {

        // Create random homogeneous coordinates for a point
        final Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final HomogeneousPoint3D point = new HomogeneousPoint3D(m.toArray());

        final Plane plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        final Plane plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));
        final Plane plane3 = new Plane(v.getSubmatrixAsArray(0, 3,
                3, 3));

        // ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));

        // plane1, plane2 and plane3 are perpendicular
        assertFalse(Line3D.areCoincidentPlanes(plane1, plane2));
        assertFalse(Line3D.areCoincidentPlanes(plane1, plane3));
        assertFalse(Line3D.areCoincidentPlanes(plane2, plane3));

        // but using the same plane...
        assertTrue(Line3D.areCoincidentPlanes(plane1, plane1));
        assertTrue(Line3D.areCoincidentPlanes(plane2, plane2));
        assertTrue(Line3D.areCoincidentPlanes(plane3, plane3));
    }

    @Test
    public void testGetSetPlanes() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException,
            CoincidentPlanesException {

        // Create random homogeneous coordinates for a point
        final Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final Plane plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        final Plane plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));
        final Plane plane3 = new Plane(v.getSubmatrixAsArray(0, 3,
                3, 3));

        final Line3D line = new Line3D(plane1, plane2);
        assertEquals(plane1, line.getPlane1());
        assertEquals(plane2, line.getPlane2());

        // set new planes
        line.setPlanes(plane2, plane3);
        assertEquals(plane2, line.getPlane1());
        assertEquals(plane3, line.getPlane2());

        // Force ParallelPlanesException
        try {
            line.setPlanes(plane1, plane1);
            fail("CoincidentPlanesException expected but not thrown");
        } catch (final CoincidentPlanesException ignore) {
        }
    }

    @Test
    public void testSetPlanesFromPoints() throws WrongSizeException,
            DecomposerException, CoincidentPointsException {

        // test constructor from 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(m) < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0,
                0, 0, HOM_COORDS - 1));
        final Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1,
                0, 1, HOM_COORDS - 1));

        final Line3D line = new Line3D(point1, point2);
        // ensure that both point1 and point2 belong to line 3D
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));

        // planes get defined, but we don't know their value, just that point1
        // and point2 belong to those planes too
        assertNotNull(line.getPlane1());
        assertNotNull(line.getPlane2());
        assertTrue(line.getPlane1().isLocus(point1));
        assertTrue(line.getPlane1().isLocus(point2));
        assertTrue(line.getPlane2().isLocus(point1));
        assertTrue(line.getPlane2().isLocus(point2));

        // now build another set of points
        m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(m) < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Point3D point3 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0,
                0, 0, HOM_COORDS - 1));
        final Point3D point4 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1,
                0, 1, HOM_COORDS - 1));

        line.setPlanesFromPoints(point3, point4);
        // ensure that both point3 and point4 belong to line 3D
        assertTrue(line.isLocus(point3));
        assertTrue(line.isLocus(point4));

        // planes get defined, but we don't know their value, just that point1
        // and point2 belong to those planes too
        assertNotNull(line.getPlane1());
        assertNotNull(line.getPlane2());
        assertTrue(line.getPlane1().isLocus(point3));
        assertTrue(line.getPlane1().isLocus(point4));
        assertTrue(line.getPlane2().isLocus(point3));
        assertTrue(line.getPlane2().isLocus(point4));
    }

    @Test
    public void testIsLocusDistanceAndClosestPoint() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            NotAvailableException, NoIntersectionException,
            CoincidentPlanesException {

        // Create random homogeneous coordinates for a point
        Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        Matrix v = decomposer.getV();

        final HomogeneousPoint3D point = new HomogeneousPoint3D(m.toArray());

        final Plane plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        final Plane plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));
        final Plane plane3 = new Plane(v.getSubmatrixAsArray(0, 3,
                3, 3));

        // ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));

        // test constructor from 2 planes
        final Line3D line = new Line3D(plane1, plane2);

        // test point is locus of line
        assertTrue(line.isLocus(point));
        assertTrue(line.isLocus(point, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            line.isLocus(point, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }

        // build another point at a given distance in perpendicular direction of
        // line.
        m = new Matrix(1, INHOM_COORDS);
        m.setSubmatrix(0, 0, 0,
                INHOM_COORDS - 1, line.getDirection());

        decomposer.setInputMatrix(m);
        decomposer.decompose();

        v = decomposer.getV();

        // because m has rank 1, the null space of m has rank 2, which are the
        // last 2 columns of V, which are perpendicular to m
        final Matrix perpendicular = v.getSubmatrix(0, 2,
                2, 2); //pick last column of V

        // determine amount of distance
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double dist = randomizer.nextDouble(ABSOLUTE_ERROR, MAX_RANDOM_VALUE);

        final Point3D distPoint = Point3D.create();
        distPoint.setInhomogeneousCoordinates(
                point.getInhomX() + dist * perpendicular.getElementAtIndex(0),
                point.getInhomY() + dist * perpendicular.getElementAtIndex(1),
                point.getInhomZ() + dist * perpendicular.getElementAtIndex(2));

        // check that distPoint is indeed at distance dist
        assertEquals(line.getDistance(distPoint), dist, ABSOLUTE_ERROR);
        // check that distPoint is no longer locus of line 3D
        assertFalse(line.isLocus(distPoint));
        // but increasing enough the threshold, it becomes locus
        assertTrue(line.isLocus(distPoint, dist));

        // because distPoint has moved in perpendicular direction from point,
        // the closest point will be point
        assertTrue(line.getClosestPoint(distPoint).equals(
                point.toInhomogeneous(), 2.0 * ABSOLUTE_ERROR));
        assertTrue(line.getClosestPoint(distPoint, ABSOLUTE_ERROR).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));

        final Point3D closestPoint = Point3D.create();
        line.closestPoint(distPoint, closestPoint);
        assertTrue(closestPoint.equals(point.toInhomogeneous(),
                ABSOLUTE_ERROR));
        line.closestPoint(distPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point.toInhomogeneous(),
                ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            line.getClosestPoint(distPoint, -dist);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            line.closestPoint(distPoint, closestPoint, -dist);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testNormalizeAndIsNormalized() throws WrongSizeException,
            DecomposerException, CoincidentPointsException {

        // test constructor from 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(m) < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0,
                0, 0, HOM_COORDS - 1));
        final Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1,
                0, 1, HOM_COORDS - 1));

        final Line3D line = new Line3D(point1, point2);

        assertFalse(line.getPlane1().isNormalized());
        assertFalse(line.getPlane2().isNormalized());
        assertFalse(line.isNormalized());

        // normalize
        line.normalize();

        // check correctness
        assertTrue(line.getPlane1().isNormalized());
        assertTrue(line.getPlane2().isNormalized());
        assertTrue(line.isNormalized());
    }

    @Test
    public void testGetDirection() throws WrongSizeException,
            DecomposerException, CoincidentPointsException {

        // test constructor from 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(m) < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        final Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0,
                0, 0, HOM_COORDS - 1));
        final Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1,
                0, 1, HOM_COORDS - 1));

        final Line3D line = new Line3D(point1, point2);

        final double[] expectedDirection = Utils.crossProduct(
                line.getPlane1().getDirectorVector(),
                line.getPlane2().getDirectorVector());

        final double[] direction = line.getDirection();

        // check that direction is equal up to scale
        final double norm1 = Utils.normF(expectedDirection);
        final double norm2 = Utils.normF(direction);

        ArrayUtils.multiplyByScalar(expectedDirection, 1.0 / norm1,
                expectedDirection);
        ArrayUtils.multiplyByScalar(direction, 1.0 / norm2, direction);

        final double[] diff = ArrayUtils.subtractAndReturnNew(direction,
                expectedDirection);

        final double normDiff = Utils.normF(diff);
        assertTrue(normDiff < LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testIntersection() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException,
            NoIntersectionException, CoincidentPlanesException {
        // Create random homogeneous coordinates for a point
        final Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final HomogeneousPoint3D point = new HomogeneousPoint3D(m.toArray());

        final Plane plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        final Plane plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));
        final Plane plane3 = new Plane(v.getSubmatrixAsArray(0, 3,
                3, 3));

        // ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));

        // test constructor from 2 planes
        final Line3D line = new Line3D(plane1, plane2);

        // because line is made of plane1 and plane2, the intersection with
        // plane3 will be point
        assertTrue(line.getIntersection(plane3).equals(point.toInhomogeneous(),
                ABSOLUTE_ERROR));

        final Point3D intersection = Point3D.create();
        line.intersection(plane3, intersection);
        assertTrue(intersection.equals(point.toInhomogeneous(),
                ABSOLUTE_ERROR));
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException,
            LockedException, NotReadyException, DecomposerException, NotAvailableException, CoincidentPlanesException, IOException, ClassNotFoundException {
        // Create random homogeneous coordinates for a point
        Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        final Matrix v = decomposer.getV();

        final Plane plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        final Plane plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));

        // create line from 2 planes
        final Line3D line1 = new Line3D(plane1, plane2);

        assertEquals(plane1, line1.getPlane1());
        assertEquals(plane2, line1.getPlane2());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(line1);
        final Line3D line2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(line1.getPlane1(), line2.getPlane1());
        assertEquals(line1.getPlane2(), line2.getPlane2());
    }
}
