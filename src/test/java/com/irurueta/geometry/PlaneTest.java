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

import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class PlaneTest {

    private static final int HOM_COORDS = 4;
    private static final int INHOM_COORDS = 3;

    private static final double PRECISION_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    private static final double MIN_RANDOM_DISTANCE = -100.0;
    private static final double MAX_RANDOM_DISTANCE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstants() {
        assertEquals(4, Plane.PLANE_NUMBER_PARAMS);
        assertEquals(1e-12, Plane.DEFAULT_LOCUS_THRESHOLD, 0.0);
        assertEquals(0.0, Plane.MIN_THRESHOLD, 0.0);
        assertEquals(1e-10, Plane.DEFAULT_COMPARISON_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException, ColinearPointsException, IllegalArgumentException,
            ParallelVectorsException {

        final var randomizer = new UniformRandomizer();

        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var plane = new Plane();
        assertEquals(0.0, plane.getA(), 0.0);
        assertEquals(0.0, plane.getB(), 0.0);
        assertEquals(0.0, plane.getC(), 0.0);
        assertEquals(0.0, plane.getD(), 0.0);
        assertFalse(plane.isNormalized());

        plane = new Plane(a, b, c, d);
        assertEquals(a, plane.getA(), 0.0);
        assertEquals(b, plane.getB(), 0.0);
        assertEquals(c, plane.getC(), 0.0);
        assertEquals(d, plane.getD(), 0.0);
        assertFalse(plane.isNormalized());

        // instantiate plane using array
        var array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        plane = new Plane(array);
        assertEquals(array[0], plane.getA(), 0.0);
        assertEquals(array[1], plane.getB(), 0.0);
        assertEquals(array[2], plane.getC(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Plane(new double[HOM_COORDS + 1]));

        // find 1st point
        final var array1 = new double[HOM_COORDS];
        randomizer.fill(array1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // find 2nd point
        final var array2 = new double[HOM_COORDS];
        randomizer.fill(array2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var lambda = randomizer.nextDouble();

        // find 3rd point as a co-linear point of 1st and 2nd point
        final var array3 = new double[HOM_COORDS];
        // array3 = lamda * array2
        ArrayUtils.multiplyByScalar(array2, lambda, array3);
        // array3 = array1 + array3 = array1 + lambda * array2
        ArrayUtils.sum(array1, array3, array3);

        var point1 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array1);
        var point2 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array2);
        var point3 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array3);

        // Force ColinearPointsException
        assertTrue(Plane.areColinearPoints(point1, point2, point3));
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint3 = point3;
        assertThrows(ColinearPointsException.class, () -> new Plane(finalPoint1, finalPoint2, finalPoint3));

        // try with 3 non co-linear points
        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 3 non-linear dependent rows
        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        // V contains the right null-space of m in the last column, which is the
        // plane where the points belong to in the rows of m
        var v = decomposer.getV();

        point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        final var arrayPlane = new double[HOM_COORDS];
        arrayPlane[0] = v.getElementAt(0, 3);
        arrayPlane[1] = v.getElementAt(1, 3);
        arrayPlane[2] = v.getElementAt(2, 3);
        arrayPlane[3] = v.getElementAt(3, 3);

        assertFalse(Plane.areColinearPoints(point1, point2, point3));
        plane = new Plane(point1, point2, point3);

        // check correctness of obtained plane
        final var scaleA = plane.getA() / arrayPlane[0];
        final var scaleB = plane.getB() / arrayPlane[1];
        final var scaleC = plane.getC() / arrayPlane[2];
        final var scaleD = plane.getD() / arrayPlane[3];

        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleD, PRECISION_ERROR);
        assertEquals(scaleD, scaleA, PRECISION_ERROR);

        // create plane passing through a point and laying on provided direction
        // vectors
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);

        final var direction1 = new double[INHOM_COORDS];
        randomizer.fill(direction1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var direction2 = new double[INHOM_COORDS];
        randomizer.fill(direction2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // normalize to compare with greater accuracy
        final var norm1 = com.irurueta.algebra.Utils.normF(direction1);
        final var direction1b = ArrayUtils.multiplyByScalarAndReturnNew(direction1, 1.0 / norm1);
        final var norm2 = com.irurueta.algebra.Utils.normF(direction2);
        final var direction2b = ArrayUtils.multiplyByScalarAndReturnNew(direction2, 1.0 / norm2);

        // director vector will already be normalized because direction vectors are normalized
        final var directorVector = com.irurueta.algebra.Utils.crossProduct(direction1b, direction2b);

        // create plane using point and directions
        var plane1 = new Plane(point, direction1, direction2);
        // and create plane using point and director vector
        var plane2 = new Plane(point, directorVector);

        // check correctness of instantiated planes
        assertTrue(plane1.isLocus(point));
        assertTrue(plane2.isLocus(point));

        final var directorVector1 = plane1.getDirectorVector();
        assertEquals(directorVector1[0], plane1.getA(), 0.0);
        assertEquals(directorVector1[1], plane1.getB(), 0.0);
        assertEquals(directorVector1[2], plane1.getC(), 0.0);
        final var directorVector2 = plane2.getDirectorVector();
        assertEquals(directorVector2[0], plane2.getA(), 0.0);
        assertEquals(directorVector2[1], plane2.getB(), 0.0);
        assertEquals(directorVector2[2], plane2.getC(), 0.0);

        // check that both director vectors are equal
        assertEquals(directorVector1[0], directorVector2[0], PRECISION_ERROR);
        assertEquals(directorVector1[1], directorVector2[1], PRECISION_ERROR);
        assertEquals(directorVector1[2], directorVector2[2], PRECISION_ERROR);

        // and equal to the estimated director vector
        final var scale1 = directorVector1[0] / directorVector[0];
        final var scale2 = directorVector1[1] / directorVector[1];
        final var scale3 = directorVector1[2] / directorVector[2];

        assertEquals(scale1, scale2, PRECISION_ERROR);
        assertEquals(scale2, scale3, PRECISION_ERROR);
        assertEquals(scale3, scale1, PRECISION_ERROR);

        // both planes are equal
        assertEquals(plane1.getA(), plane2.getA(), PRECISION_ERROR);
        assertEquals(plane1.getB(), plane2.getB(), PRECISION_ERROR);
        assertEquals(plane1.getC(), plane2.getC(), PRECISION_ERROR);
        assertEquals(plane1.getD(), plane2.getD(), PRECISION_ERROR);

        // Force ParallelVectorsException
        assertThrows(ParallelVectorsException.class, () -> new Plane(point, direction1, direction1));

        // Force IllegalArgumentException
        final var wrongArray = new double[HOM_COORDS];
        assertThrows(IllegalArgumentException.class, () -> new Plane(point, wrongArray, direction2));
        assertThrows(IllegalArgumentException.class, () -> new Plane(point, direction1, wrongArray));
        assertThrows(IllegalArgumentException.class, () -> new Plane(point, wrongArray));
    }

    @Test
    void testGettersAndSetters() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var plane = new Plane();

        // set parameters using array
        plane.setParameters(array);
        assertEquals(array[0], plane.getA(), 0.0);
        assertEquals(array[1], plane.getB(), 0.0);
        assertEquals(array[2], plane.getC(), 0.0);
        assertEquals(array[3], plane.getD(), 0.0);

        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        plane.setA(a);
        plane.setB(b);
        plane.setC(c);
        plane.setD(d);

        assertEquals(a, plane.getA(), 0.0);
        assertEquals(b, plane.getB(), 0.0);
        assertEquals(c, plane.getC(), 0.0);
        assertEquals(d, plane.getD(), 0.0);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        plane.setParameters(a, b, c, d);
        assertEquals(a, plane.getA(), 0.0);
        assertEquals(b, plane.getB(), 0.0);
        assertEquals(c, plane.getC(), 0.0);
        assertEquals(d, plane.getD(), 0.0);
    }

    @Test
    void testSetParametersFromThreePoints() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException, ColinearPointsException {

        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        // the right null-space of m (last column of V) contains the plane where
        // the 3 points belong to
        final var v = decomposer.getV();

        Point3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        Point3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        Point3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        final var planeArray = new double[HOM_COORDS];
        planeArray[0] = v.getElementAt(0, 3);
        planeArray[1] = v.getElementAt(1, 3);
        planeArray[2] = v.getElementAt(2, 3);
        planeArray[3] = v.getElementAt(3, 3);

        var plane = new Plane();
        assertFalse(Plane.areColinearPoints(point1, point2, point3));
        plane.setParametersFromThreePoints(point1, point2, point3);

        // compare plane parameters respect to computed plane array parameters
        // They will be equal up to scale
        final var scaleA = plane.getA() / planeArray[0];
        final var scaleB = plane.getB() / planeArray[1];
        final var scaleC = plane.getC() / planeArray[2];
        final var scaleD = plane.getD() / planeArray[3];

        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleD, PRECISION_ERROR);
        assertEquals(scaleD, scaleA, PRECISION_ERROR);

        // all three points belong to the plane
        // to increase accuracy
        plane.normalize();
        assertTrue(plane.isLocus(point1, PRECISION_ERROR));
        assertTrue(plane.isLocus(point2, PRECISION_ERROR));
        assertTrue(plane.isLocus(point3, PRECISION_ERROR));

        // Force ColinearPointsException
        // find co-linear points using a third point being a linear combination
        // of two other points so that a plane cannot be defined
        final var randomizer = new UniformRandomizer();

        // find 1st point
        final var array1 = new double[HOM_COORDS];
        randomizer.fill(array1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // find 2nd point
        final var array2 = new double[HOM_COORDS];
        randomizer.fill(array2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var lambda = randomizer.nextDouble();

        // find 3rd point as a co-linear point of 1st and 2nd point
        final var array3 = new double[HOM_COORDS];
        // array3 = lamda * array2
        ArrayUtils.multiplyByScalar(array2, lambda, array3);
        // array3 = array1 + array3 = array1 + lambda * array2
        ArrayUtils.sum(array1, array3, array3);

        point1 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array1);
        point2 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array2);
        point3 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array3);

        // Force ColinearPointsException
        assertTrue(Plane.areColinearPoints(point1, point2, point3));
        final var finalPoint1 = point1;
        final var finalPoint2 = point2;
        final var finalPoint3 = point3;
        assertThrows(ColinearPointsException.class, () -> new Plane(finalPoint1, finalPoint2, finalPoint3));
    }

    @Test
    void testSetParametersFrom1PointAnd2Vectors() throws WrongSizeException, IllegalArgumentException,
            ParallelVectorsException {
        // create plane passing through a point and laying on provided direction
        // vectors
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);

        final var direction1 = new double[INHOM_COORDS];
        randomizer.fill(direction1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var direction2 = new double[INHOM_COORDS];
        randomizer.fill(direction2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // normalize to compare with greater accuracy
        final var norm1 = com.irurueta.algebra.Utils.normF(direction1);
        final var direction1b = ArrayUtils.multiplyByScalarAndReturnNew(direction1, 1.0 / norm1);
        final var norm2 = com.irurueta.algebra.Utils.normF(direction2);
        final var direction2b = ArrayUtils.multiplyByScalarAndReturnNew(direction2, 1.0 / norm2);

        // director vector will already be normalized because direction vectors
        // are normalized
        final var directorVector = com.irurueta.algebra.Utils.crossProduct(direction1b, direction2b);

        final var plane = new Plane();

        // set parameters using point and directions
        plane.setParametersFrom1PointAnd2Vectors(point, direction1, direction2);

        // check correctness of plane
        assertTrue(plane.isLocus(point));

        final var directorVectorB = plane.getDirectorVector();
        assertEquals(directorVectorB[0], plane.getA(), 0.0);
        assertEquals(directorVectorB[1], plane.getB(), 0.0);
        assertEquals(directorVectorB[2], plane.getC(), 0.0);

        // and equal to the estimated director vector
        final var scale1 = directorVectorB[0] / directorVector[0];
        final var scale2 = directorVectorB[1] / directorVector[1];
        final var scale3 = directorVectorB[2] / directorVector[2];

        assertEquals(scale1, scale2, PRECISION_ERROR);
        assertEquals(scale2, scale3, PRECISION_ERROR);
        assertEquals(scale3, scale1, PRECISION_ERROR);

        // Force ParallelVectorsException
        assertThrows(ParallelVectorsException.class, () -> plane.setParametersFrom1PointAnd2Vectors(point, direction1,
                direction1));

        // Force IllegalArgumentException
        final var wrongArray = new double[HOM_COORDS];
        assertThrows(IllegalArgumentException.class,
                () -> plane.setParametersFrom1PointAnd2Vectors(point, wrongArray, direction2));
        assertThrows(IllegalArgumentException.class, () -> plane.setParametersFrom1PointAnd2Vectors(point, direction1,
                wrongArray));
    }

    @Test
    void testSetParametersFrom1PointAndDirectorVector() throws WrongSizeException {
        // create plane passing through a point and laying on provided direction
        // vectors
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);

        final var direction1 = new double[INHOM_COORDS];
        randomizer.fill(direction1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var direction2 = new double[INHOM_COORDS];
        randomizer.fill(direction2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // normalize to compare with greater accuracy
        final var norm1 = com.irurueta.algebra.Utils.normF(direction1);
        final var direction1b = ArrayUtils.multiplyByScalarAndReturnNew(direction1, 1.0 / norm1);
        final var norm2 = com.irurueta.algebra.Utils.normF(direction2);
        final var direction2b = ArrayUtils.multiplyByScalarAndReturnNew(direction2, 1.0 / norm2);

        // director vector will already be normalized because direction vectors are normalized
        final var directorVector = com.irurueta.algebra.Utils.crossProduct(direction1b, direction2b);

        final var plane = new Plane();

        // create plane using point and directions
        // and create plane using point and director vector
        plane.setParametersFromPointAndDirectorVector(point, directorVector);

        // check correctness of plane
        assertTrue(plane.isLocus(point));

        final var directorVectorB = plane.getDirectorVector();
        assertEquals(directorVectorB[0], plane.getA(), 0.0);
        assertEquals(directorVectorB[1], plane.getB(), 0.0);
        assertEquals(directorVectorB[2], plane.getC(), 0.0);

        // and equal to the estimated director vector
        final var scale1 = directorVectorB[0] / directorVector[0];
        final var scale2 = directorVectorB[1] / directorVector[1];
        final var scale3 = directorVectorB[2] / directorVector[2];

        assertEquals(scale1, scale2, PRECISION_ERROR);
        assertEquals(scale2, scale3, PRECISION_ERROR);
        assertEquals(scale3, scale1, PRECISION_ERROR);

        // Force IllegalArgumentException
        final var wrongArray = new double[HOM_COORDS];
        assertThrows(IllegalArgumentException.class,
                () -> plane.setParametersFromPointAndDirectorVector(point, wrongArray));
    }

    @Test
    void testIsLocus() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // randomly choose 3 points to find their corresponding plane
        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var v = decomposer.getV();

        final var plane = new Plane(v.getElementAt(0, 3),
                v.getElementAt(1, 3),
                v.getElementAt(2, 3),
                v.getElementAt(3, 3));

        final var point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));

        final var point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));

        final var point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        // points belong to plane
        assertTrue(plane.isLocus(point1, PRECISION_ERROR));
        assertTrue(plane.isLocus(point2, PRECISION_ERROR));
        assertTrue(plane.isLocus(point3, PRECISION_ERROR));

        // use plane director vector to find another point outside of plane
        final var point4 = new HomogeneousPoint3D(point1);

        final var normDirectorVector = Math.sqrt(plane.getA() * plane.getA() + plane.getB() * plane.getB()
                + plane.getC() * plane.getC());
        point4.setInhomogeneousCoordinates(point4.getInhomX() + plane.getA(),
                point4.getInhomY() + plane.getB(),
                point4.getInhomZ() + plane.getC());

        assertFalse(plane.isLocus(point4, PRECISION_ERROR));

        // indeed point4 is at normDirectorVector distance from plane
        assertEquals(normDirectorVector, plane.signedDistance(point4), PRECISION_ERROR);
    }

    @Test
    void testSignedDistance() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // randomly choose 3 points to find their corresponding plane
        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var v = decomposer.getV();

        final var plane = new Plane(v.getElementAt(0, 3),
                v.getElementAt(1, 3),
                v.getElementAt(2, 3),
                v.getElementAt(3, 3));

        final var point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));

        final var point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));

        final var point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        // points belong to plane, hence their distance is zero up to machine precision
        assertEquals(0.0, plane.signedDistance(point1), PRECISION_ERROR);
        assertEquals(0.0, plane.signedDistance(point2), PRECISION_ERROR);
        assertEquals(0.0, plane.signedDistance(point3), PRECISION_ERROR);

        // use plane director vector to find another point at desired signed distance
        final var randomizer = new UniformRandomizer();
        final var signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE, MAX_RANDOM_DISTANCE);

        final var point4 = new HomogeneousPoint3D(point1);

        final var normDirectorVector = com.irurueta.algebra.Utils.normF(plane.getDirectorVector());
        point4.setInhomogeneousCoordinates(
                point4.getInhomX() + signedDistance * plane.getA() / normDirectorVector,
                point4.getInhomY() + signedDistance * plane.getB() / normDirectorVector,
                point4.getInhomZ() + signedDistance * plane.getC() / normDirectorVector);

        assertEquals(plane.signedDistance(point4), signedDistance, PRECISION_ERROR);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();

        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var plane = new Plane(a, b, c, d);
        final var array = plane.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertEquals(d, array[3], 0.0);

        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        plane.setParameters(a, b, c, d);
        plane.asArray(array);

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertEquals(d, array[3], 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> plane.asArray(new double[HOM_COORDS + 1]));
    }

    @Test
    void testNormalize() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var plane = new Plane(array);
        assertFalse(plane.isNormalized());

        // normalize plane
        plane.normalize();
        assertTrue(plane.isNormalized());

        // return plane as array
        final var array2 = plane.asArray();

        // compare that both arrays are equal up to scale
        // check correctness of obtained plane
        final var scaleA = array[0] / array2[0];
        final var scaleB = array[1] / array2[1];
        final var scaleC = array[2] / array2[2];
        final var scaleD = array[3] / array2[3];

        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleD, PRECISION_ERROR);
        assertEquals(scaleD, scaleA, PRECISION_ERROR);

        // if we provide zero values, then normalization has no effect
        plane = new Plane(0.0, 0.0, 0.0, 0.0);

        assertFalse(plane.isNormalized());

        plane.normalize();

        assertFalse(plane.isNormalized());
    }

    @Test
    void testDirectorVector() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var plane = new Plane(array);

        final var n = plane.getDirectorVector();

        assertEquals(3, n.length);
        assertEquals(array[0], n[0],0.0);
        assertEquals(array[1], n[1], 0.0);
        assertEquals(array[2], n[2], 0.0);

        // try again
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        plane.setParameters(array);

        plane.directorVector(n);

        assertEquals(3, n.length);
        assertEquals(n[0], array[0], 0.0);
        assertEquals(n[1], array[1], 0.0);
        assertEquals(n[2], array[2], 0.0);
    }

    @Test
    void testIntersection() throws WrongSizeException, NotReadyException, com.irurueta.algebra.NotAvailableException,
            LockedException, DecomposerException, NoIntersectionException, ColinearPointsException {

        // Create random homogeneous coordinates for a point
        final var m = Matrix.createWithUniformRandomValues(1, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // M is a 1x4 matrix having rank 1, hence its right null-space will have
        // dimension 3. Each vector of the right null-space will follow equation:
        // m * P = 0, hence each of those vectors will be a plane where the point
        // will be locus, and hence the point will be the intersection of those
        // 3 planes, which will be perpendicular among them
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        final var v = decomposer.getV();

        final var point = new HomogeneousPoint3D(m.toArray());

        var plane1 = new Plane(v.getSubmatrixAsArray(0, 1,
                3, 1));
        var plane2 = new Plane(v.getSubmatrixAsArray(0, 2,
                3, 2));
        var plane3 = new Plane(v.getSubmatrixAsArray(0, 3,
                3, 3));

        assertTrue(plane1.getIntersection(plane2, plane3).equals(point, ABSOLUTE_ERROR));

        final var intersection = Point3D.create();
        plane1.intersection(plane2, plane3, intersection);
        assertTrue(intersection.equals(point, ABSOLUTE_ERROR));

        assertTrue(plane2.getIntersection(plane1, plane3).equals(point, ABSOLUTE_ERROR));

        plane2.intersection(plane1, plane3, intersection);
        assertTrue(intersection.equals(point, ABSOLUTE_ERROR));

        assertTrue(plane3.getIntersection(plane1, plane2).equals(point, ABSOLUTE_ERROR));

        plane3.intersection(plane1, plane2, intersection);
        assertTrue(intersection.equals(point, ABSOLUTE_ERROR));

        // Force NoIntersectionException by using two coincident or parallel planes
        final var finalPlane1 = plane1;
        final var finalPlane2 = plane2;
        assertThrows(NoIntersectionException.class, () -> finalPlane1.getIntersection(finalPlane1, finalPlane2));
        assertThrows(NoIntersectionException.class,
                () -> finalPlane1.intersection(finalPlane1, finalPlane2, intersection));

        // we could also find 7 random points to find 3 planes having one point
        // in common, which will be their intersection
        final var point1 = new HomogeneousPoint3D();
        final var point2 = new HomogeneousPoint3D();
        final var point3 = new HomogeneousPoint3D();
        final var point4 = new HomogeneousPoint3D();
        final var point5 = new HomogeneousPoint3D();
        final var point6 = new HomogeneousPoint3D();
        final var point7 = new HomogeneousPoint3D();

        // create three intersecting planes using all points (each point is
        // defined as one row of the matrix)
        var m1 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var m2 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var m3 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // set 1st point in common in all three matrices
        m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1,
                m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
        m3.setSubmatrix(0, 0, 0, HOM_COORDS - 1,
                m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));

        // ensure that all matrices have rank 3 (points are not co-linear)
        while (com.irurueta.algebra.Utils.rank(m1) < 3 || com.irurueta.algebra.Utils.rank(m2) < 3
                || com.irurueta.algebra.Utils.rank(m3) < 3) {
            // create random matrices again
            m1 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            m2 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            m3 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            // set 1st point in common in all three matrices
            m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1,
                    m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
            m3.setSubmatrix(0, 0, 0, HOM_COORDS - 1,
                    m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
        }

        point1.setCoordinates(m1.getSubmatrixAsArray(0, 0, 0,
                HOM_COORDS - 1));
        point2.setCoordinates(m1.getSubmatrixAsArray(1, 0, 1,
                HOM_COORDS - 1));
        point3.setCoordinates(m1.getSubmatrixAsArray(2, 0, 2,
                HOM_COORDS - 1));

        point4.setCoordinates(m2.getSubmatrixAsArray(1, 0, 1,
                HOM_COORDS - 1));
        point5.setCoordinates(m2.getSubmatrixAsArray(2, 0, 2,
                HOM_COORDS - 1));

        point6.setCoordinates(m3.getSubmatrixAsArray(1, 0, 1,
                HOM_COORDS - 1));
        point7.setCoordinates(m3.getSubmatrixAsArray(2, 0, 2,
                HOM_COORDS - 1));

        // Create three planes between point1-point2-point3,
        // point1-point4-point5 and point1-point6-point7
        plane1 = new Plane(point1, point2, point3);
        plane2 = new Plane(point1, point4, point5);
        plane3 = new Plane(point1, point6, point7);

        // because all three planes have in common point1, then their
        // intersection must be point1
        assertTrue(plane1.getIntersection(plane2, plane3).equals(point1, ABSOLUTE_ERROR));
        plane1.intersection(plane2, plane3, intersection);
        assertTrue(intersection.equals(point1, ABSOLUTE_ERROR));

        assertTrue(plane2.getIntersection(plane1, plane3).equals(point1, ABSOLUTE_ERROR));
        plane2.intersection(plane1, plane3, intersection);
        assertTrue(intersection.equals(point1, ABSOLUTE_ERROR));

        assertTrue(plane3.getIntersection(plane1, plane2).equals(point1, ABSOLUTE_ERROR));
        plane3.intersection(plane1, plane2, intersection);
        assertTrue(intersection.equals(point1, ABSOLUTE_ERROR));
    }

    @Test
    void testClosestPoint() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException, ColinearPointsException {

        // randomly choose 3 points to find their corresponding plane.
        // Each point is represented as one row of matrix below
        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure points are not co-linear
        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0,
                HOM_COORDS - 1));
        final var point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1,
                HOM_COORDS - 1));
        final var point3 = new HomogeneousPoint3D(m.getSubmatrixAsArray(2, 0, 2,
                HOM_COORDS - 1));

        final var plane = new Plane(point1, point2, point3);

        // point1, point2 and point3 belong to plane, hence their distance to the
        // plane is zero up to machine precision
        assertEquals(0.0, plane.signedDistance(point1), PRECISION_ERROR);
        assertEquals(0.0, plane.signedDistance(point2), PRECISION_ERROR);
        assertEquals(0.0, plane.signedDistance(point3), PRECISION_ERROR);

        // because they belong to plane, their closest point to plane is
        // themselves
        final var closestPoint1 = plane.getClosestPoint(point1);
        final var closestPoint1b = plane.getClosestPoint(point1, PRECISION_ERROR);
        final var closestPoint1c = Point3D.create();
        plane.closestPoint(point1, closestPoint1c);
        final var closestPoint1d = Point3D.create();
        plane.closestPoint(point1, closestPoint1d, ABSOLUTE_ERROR);

        assertTrue(closestPoint1.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1b.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1c.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1d.equals(point1, ABSOLUTE_ERROR));

        final var closestPoint2 = plane.getClosestPoint(point2);
        final var closestPoint2b = plane.getClosestPoint(point2, PRECISION_ERROR);
        final var closestPoint2c = Point3D.create();
        plane.closestPoint(point2, closestPoint2c);
        final var closestPoint2d = Point3D.create();
        plane.closestPoint(point2, closestPoint2d, ABSOLUTE_ERROR);

        assertTrue(closestPoint2.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2b.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2c.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2d.equals(point2, ABSOLUTE_ERROR));

        final var closestPoint3 = plane.getClosestPoint(point3);
        final var closestPoint3b = plane.getClosestPoint(point3, PRECISION_ERROR);
        final var closestPoint3c = Point3D.create();
        plane.closestPoint(point3, closestPoint3c);
        final var closestPoint3d = Point3D.create();
        plane.closestPoint(point3, closestPoint3d, ABSOLUTE_ERROR);

        assertTrue(closestPoint3.equals(point3, ABSOLUTE_ERROR));
        assertTrue(closestPoint3b.equals(point3, ABSOLUTE_ERROR));
        assertTrue(closestPoint3c.equals(point3, ABSOLUTE_ERROR));
        assertTrue(closestPoint3d.equals(point3, ABSOLUTE_ERROR));

        // use plane director vector to find another point at desired signed
        // distance
        final var randomizer = new UniformRandomizer();
        final var signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE, MAX_RANDOM_DISTANCE);

        final var point4 = new HomogeneousPoint3D();

        final var normDirectorVector = Math.sqrt(plane.getA() * plane.getA() + plane.getB() * plane.getB()
                + plane.getC() * plane.getC());
        point4.setInhomogeneousCoordinates(
                point1.getInhomX() + signedDistance * plane.getA() / normDirectorVector,
                point1.getInhomY() + signedDistance * plane.getB() / normDirectorVector,
                point1.getInhomZ() + signedDistance * plane.getC() / normDirectorVector);

        assertEquals(plane.signedDistance(point4), signedDistance, PRECISION_ERROR);

        // because point4 goes in plane's perpendicular direction from point1,
        // its closest point belonging to the plane will be point1
        final var closestPoint = plane.getClosestPoint(point4);
        final var closestPointB = plane.getClosestPoint(point4, PRECISION_ERROR);
        final var closestPointC = Point3D.create();
        plane.closestPoint(point4, closestPointC);
        final var closestPointD = Point3D.create();
        plane.closestPoint(point4, closestPointD, ABSOLUTE_ERROR);

        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointB.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointC.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointD.equals(point1, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> plane.getClosestPoint(point4, -PRECISION_ERROR));
        assertThrows(IllegalArgumentException.class, () -> plane.closestPoint(point4, closestPointD, -ABSOLUTE_ERROR));
    }

    @Test
    void testDotProduct() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // a random line
        final var plane1 = new Plane(array);

        // opposed sign line
        final var plane2 = new Plane(-plane1.getA(), -plane1.getB(), -plane1.getC(), -plane1.getD());

        // another random line
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var plane3 = new Plane(array);

        plane1.normalize();
        plane2.normalize();
        plane3.normalize();

        // test for equal lines
        assertEquals(1.0, plane1.dotProduct(plane1), ABSOLUTE_ERROR);
        // test for opposed sign lines
        assertEquals(-1.0, plane1.dotProduct(plane2), ABSOLUTE_ERROR);
        // test for 2 random lines
        assertEquals(plane1.getA() * plane3.getA() + plane1.getB() * plane3.getB()
                        + plane1.getC() * plane3.getC() + plane1.getD() * plane3.getD(),
                plane1.dotProduct(plane3), ABSOLUTE_ERROR);
    }

    @Test
    void testEqualsAndHashCode() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var plane1 = new Plane(array);
        var plane2 = new Plane(array);
        assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
        assertTrue(plane1.equals(plane2));
        assertEquals(plane1, plane2);
        assertEquals(plane1.hashCode(), plane2.hashCode());

        array[0] = plane1.getA() + randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = plane1.getB();
        array[2] = plane1.getC();
        array[3] = plane1.getD();

        plane2 = new Plane(array);
        assertFalse(plane1.equals(plane2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        plane1 = new Plane(array);
        array[0] *= 2.0;
        plane2 = new Plane(array);
        assertTrue(plane1.equals(plane2, 2.0));
        assertFalse(plane1.equals(plane2, 0.0));

        // Force IllegalArgumentException
        final var finalPlane1 = plane1;
        assertThrows(IllegalArgumentException.class, () -> finalPlane1.equals(finalPlane1, -ABSOLUTE_ERROR));
    }

    @Test
    void testCreateCanonicalPlaneAtInfinity() {
        final var plane = Plane.createCanonicalPlaneAtInfinity();

        assertEquals(0.0, plane.getA(), 0.0);
        assertEquals(0.0, plane.getB(), 0.0);
        assertEquals(0.0, plane.getC(), 0.0);
        assertEquals(1.0, plane.getD(), 0.0);

        // create a point at infinity
        final var randomizer = new UniformRandomizer();
        final var point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);

        // check that point at infinity is locus of plane
        assertTrue(plane.isLocus(point));
    }

    @Test
    void testSetAsCanonicalPlaneAtInfinity() {
        // create a point at infinity
        final var randomizer = new UniformRandomizer();
        final var point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);

        final var plane = new Plane(1.0, 1.0, 1.0, 1.0);

        // initially plane is not at infinity
        assertFalse(plane.isLocus(point));

        // set plane at infinity
        Plane.setAsCanonicalPlaneAtInfinity(plane);

        // check correctness
        assertEquals(0.0, plane.getA(), 0.0);
        assertEquals(0.0, plane.getB(), 0.0);
        assertEquals(0.0, plane.getC(), 0.0);
        assertEquals(1.0, plane.getD(), 0.0);

        // check that point at infinity is now locus of plane
        assertTrue(plane.isLocus(point));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var plane1 = new Plane(a, b, c, d);

        // check
        assertEquals(a, plane1.getA(), 0.0);
        assertEquals(b, plane1.getB(), 0.0);
        assertEquals(c, plane1.getC(), 0.0);
        assertEquals(d, plane1.getD(), 0.0);
        assertFalse(plane1.isNormalized());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(plane1);
        final var plane2 = SerializationHelper.<Plane>deserialize(bytes);

        // check
        assertEquals(plane1, plane2);
        assertNotSame(plane1, plane2);
    }
}
