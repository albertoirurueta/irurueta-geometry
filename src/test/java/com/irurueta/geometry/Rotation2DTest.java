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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class Rotation2DTest {

    private static final double MIN_THETA = -Math.PI;
    private static final double MAX_THETA = Math.PI;
    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int INHOM_COORDS = 2;

    @Test
    void testConstants() {
        assertEquals(2, Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS);
        assertEquals(2, Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        assertEquals(3, Rotation2D.ROTATION2D_HOM_MATRIX_ROWS);
        assertEquals(3, Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        assertEquals(1e-9, Rotation2D.MATRIX_VALID_THRESHOLD, 0.0);
        assertEquals(0.0, Rotation2D.MIN_THRESHOLD, 0.0);
        assertEquals(1e-9, Rotation2D.DEFAULT_COMPARISON_THRESHOLD, 0.0);
        assertEquals(0.0, Rotation2D.MIN_COMPARISON_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() throws WrongSizeException, InvalidRotationMatrixException {

        Rotation2D rotation;

        // test empty constructor
        assertNotNull(rotation = new Rotation2D());
        assertEquals(0.0, rotation.getTheta(), 0.0);

        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        // test from rotation angle
        rotation = new Rotation2D(theta);
        assertEquals(rotation.getTheta(), theta, 0.0);

        // test from another rotation
        final var rotation2 = new Rotation2D(rotation);
        assertEquals(theta, rotation2.getTheta(), 0.0);

        // test from inhomogeneous rotation matrix
        final var inhomRotMat = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        final var cosTheta = Math.cos(theta);
        final var sinTheta = Math.sin(theta);
        inhomRotMat.setElementAt(0, 0, cosTheta);
        inhomRotMat.setElementAt(1, 0, sinTheta);
        inhomRotMat.setElementAt(0, 1, -sinTheta);
        inhomRotMat.setElementAt(1, 1, cosTheta);

        rotation = new Rotation2D(inhomRotMat);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // now using threshold
        rotation = new Rotation2D(inhomRotMat, ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        assertThrows(IllegalArgumentException.class, () -> new Rotation2D(inhomRotMat, -ABSOLUTE_ERROR));

        // test from homogeneous rotation matrix
        final var homRotMat = Matrix.identity(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        homRotMat.setSubmatrix(0, 0, 1, 1, inhomRotMat);

        rotation = new Rotation2D(homRotMat);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // now using threshold
        rotation = new Rotation2D(homRotMat, ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        assertThrows(IllegalArgumentException.class, () -> new Rotation2D(inhomRotMat, -ABSOLUTE_ERROR));

        // Force InvalidRotationMatrixException
        // because of wrong size
        final var m1 = new Matrix(2, 3);
        assertThrows(InvalidRotationMatrixException.class, () -> new Rotation2D(m1));

        // because determinant is not 1
        final var m2 = Matrix.identity(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        m2.multiplyByScalar(2.0);
        assertThrows(InvalidRotationMatrixException.class, () -> new Rotation2D(m2));
    }

    @Test
    void testGetSetTheta() {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D();
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // set new theta
        rotation.setTheta(theta);
        // check correctness
        assertEquals(theta, rotation.getTheta(), 0.0);
    }

    @Test
    void testInverseRotation() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D(theta);
        assertEquals(theta, rotation.getTheta(), 0.0);

        // get inverse rotation
        final var invRotation = rotation.inverseRotation();
        assertEquals(-theta, invRotation.getTheta(), 0.0);

        // check inverse rotation matrix is indeed the inverse
        final var rotInhomMatrix = rotation.asInhomogeneousMatrix();
        final var rotHomMatrix = rotation.asHomogeneousMatrix();
        var invRotInhomMatrix = invRotation.asInhomogeneousMatrix();
        var invRotHomMatrix = invRotation.asHomogeneousMatrix();

        assertTrue(rotInhomMatrix.multiplyAndReturnNew(invRotInhomMatrix).equals(Matrix.identity(
                Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS, Rotation2D.ROTATION2D_INHOM_MATRIX_COLS), ABSOLUTE_ERROR));
        assertTrue(rotHomMatrix.multiplyAndReturnNew(invRotHomMatrix).equals(Matrix.identity(
                Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, Rotation2D.ROTATION2D_HOM_MATRIX_COLS), ABSOLUTE_ERROR));

        // inverse rotation using provided rotation instance
        rotation.inverseRotation(rotation);
        assertEquals(-theta, rotation.getTheta(), 0.0);


        // check again inverse rotation matrix is indeed the inverse
        invRotInhomMatrix = rotation.asInhomogeneousMatrix();
        invRotHomMatrix = rotation.asHomogeneousMatrix();

        assertTrue(rotInhomMatrix.multiplyAndReturnNew(invRotInhomMatrix).equals(Matrix.identity(
                Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS, Rotation2D.ROTATION2D_INHOM_MATRIX_COLS), ABSOLUTE_ERROR));
        assertTrue(rotHomMatrix.multiplyAndReturnNew(invRotHomMatrix).equals(Matrix.identity(
                Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, Rotation2D.ROTATION2D_HOM_MATRIX_COLS), ABSOLUTE_ERROR));
    }

    @Test
    void testAsInhomogeneousMatrix() throws InvalidRotationMatrixException, WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D(theta);

        final var rotMatrix1 = rotation.asInhomogeneousMatrix();

        final var sinTheta = Math.sin(theta);
        final var cosTheta = Math.cos(theta);

        // check matrix correctness
        assertEquals(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS, rotMatrix1.getRows());
        assertEquals(Rotation2D.ROTATION2D_INHOM_MATRIX_COLS, rotMatrix1.getColumns());
        assertEquals(cosTheta, rotMatrix1.getElementAt(0, 0), 0.0);
        assertEquals(sinTheta, rotMatrix1.getElementAt(1, 0), 0.0);
        assertEquals(-sinTheta, rotMatrix1.getElementAt(0, 1), 0.0);
        assertEquals(cosTheta, rotMatrix1.getElementAt(1, 1), 0.0);

        // build rotation from matrix and check correctness
        var rotation2 = new Rotation2D(rotMatrix1);
        assertEquals(theta, rotation2.getTheta(), ABSOLUTE_ERROR);

        // try again providing matrix where rotation will be stored
        final var rotMatrix2 = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        rotation.asInhomogeneousMatrix(rotMatrix2);

        // check matrix correctness
        assertEquals(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS, rotMatrix2.getRows());
        assertEquals(Rotation2D.ROTATION2D_INHOM_MATRIX_COLS, rotMatrix2.getColumns());
        assertEquals(cosTheta, rotMatrix2.getElementAt(0, 0), 0.0);
        assertEquals(sinTheta, rotMatrix2.getElementAt(1, 0), 0.0);
        assertEquals(-sinTheta, rotMatrix2.getElementAt(0, 1), 0.0);
        assertEquals(cosTheta, rotMatrix2.getElementAt(1, 1), 0.0);

        // build rotation from matrix and check correctness
        rotation2 = new Rotation2D(rotMatrix2);
        assertEquals(theta, rotation2.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var rotMatrix3 = new Matrix(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        assertThrows(IllegalArgumentException.class, () -> rotation.asInhomogeneousMatrix(rotMatrix3));
    }

    @Test
    void testAsHomogeneousMatrix() throws InvalidRotationMatrixException, WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D(theta);

        final var rotMatrix1 = rotation.asHomogeneousMatrix();

        final var sinTheta = Math.sin(theta);
        final var cosTheta = Math.cos(theta);

        // check matrix correctness
        assertEquals(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, rotMatrix1.getRows());
        assertEquals(Rotation2D.ROTATION2D_HOM_MATRIX_COLS, rotMatrix1.getColumns());
        assertEquals(cosTheta, rotMatrix1.getElementAt(0, 0), 0.0);
        assertEquals(sinTheta, rotMatrix1.getElementAt(1, 0), 0.0);
        assertEquals(0.0, rotMatrix1.getElementAt(2, 0), 0.0);
        assertEquals(-sinTheta, rotMatrix1.getElementAt(0, 1), 0.0);
        assertEquals(cosTheta, rotMatrix1.getElementAt(1, 1), 0.0);
        assertEquals(0.0, rotMatrix1.getElementAt(2, 1), 0.0);
        assertEquals(0.0, rotMatrix1.getElementAt(0, 2), 0.0);
        assertEquals(0.0, rotMatrix1.getElementAt(1, 2), 0.0);
        assertEquals(1.0, rotMatrix1.getElementAt(2, 2), 0.0);

        // build rotation from matrix and check correctness
        var rotation2 = new Rotation2D(rotMatrix1);
        assertEquals(theta, rotation2.getTheta(), ABSOLUTE_ERROR);

        // try again providing matrix where rotation will be stored
        final var rotMatrix2 = new Matrix(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        rotation.asHomogeneousMatrix(rotMatrix2);

        // check matrix correctness
        assertEquals(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, rotMatrix2.getRows());
        assertEquals(Rotation2D.ROTATION2D_HOM_MATRIX_COLS, rotMatrix2.getColumns());
        assertEquals(cosTheta, rotMatrix2.getElementAt(0, 0), 0.0);
        assertEquals(sinTheta, rotMatrix2.getElementAt(1, 0), 0.0);
        assertEquals(0.0, rotMatrix2.getElementAt(2, 0), 0.0);
        assertEquals(-sinTheta, rotMatrix2.getElementAt(0, 1), 0.0);
        assertEquals(cosTheta, rotMatrix2.getElementAt(1, 1), 0.0);
        assertEquals(0.0, rotMatrix2.getElementAt(2, 1), 0.0);
        assertEquals(0.0, rotMatrix2.getElementAt(0, 2), 0.0);
        assertEquals(0.0, rotMatrix2.getElementAt(1, 2), 0.0);
        assertEquals(1.0, rotMatrix2.getElementAt(2, 2), 0.0);

        // build rotation from matrix and check correctness
        rotation2 = new Rotation2D(rotMatrix2);
        assertEquals(theta, rotation2.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var rotMatrix3 = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        assertThrows(IllegalArgumentException.class, () -> rotation.asHomogeneousMatrix(rotMatrix3));
    }

    @Test
    void testFromMatrixAndIsValidRotationMatrix() throws WrongSizeException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D();
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // test from inhomogeneous rotation matrix
        final var inhomRotMat = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        final var cosTheta = Math.cos(theta);
        final var sinTheta = Math.sin(theta);
        inhomRotMat.setElementAt(0, 0, cosTheta);
        inhomRotMat.setElementAt(1, 0, sinTheta);
        inhomRotMat.setElementAt(0, 1, -sinTheta);
        inhomRotMat.setElementAt(1, 1, cosTheta);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat, ABSOLUTE_ERROR));

        // set rotation from inhomogeneous matrix
        rotation.fromMatrix(inhomRotMat);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // try again with threshold
        rotation.fromMatrix(inhomRotMat, ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // test from homogeneous rotation matrix
        final var homRotMat = Matrix.identity(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        homRotMat.setSubmatrix(0, 0, 1, 1, inhomRotMat);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat, ABSOLUTE_ERROR));

        rotation.fromMatrix(homRotMat);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // try again with threshold
        rotation.fromMatrix(homRotMat, ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        assertThrows(IllegalArgumentException.class, () -> rotation.fromMatrix(homRotMat, -ABSOLUTE_ERROR));

        // Force InvalidRotationMatrix
        // because of wrong size
        final var m1 = new Matrix(2, 3);
        assertFalse(Rotation2D.isValidRotationMatrix(m1));
        assertFalse(Rotation2D.isValidRotationMatrix(m1, ABSOLUTE_ERROR));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromMatrix(m1));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromMatrix(m1, ABSOLUTE_ERROR));

        // because determinant is not 1
        final var m2 = Matrix.identity(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        m2.multiplyByScalar(2.0);
        assertFalse(Rotation2D.isValidRotationMatrix(m2));
        assertFalse(Rotation2D.isValidRotationMatrix(m2, ABSOLUTE_ERROR));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromMatrix(m2));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromMatrix(m2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException into isValidRotationMatrix because threshold is negative
        assertThrows(IllegalArgumentException.class, () -> Rotation2D.isValidRotationMatrix(m2, -ABSOLUTE_ERROR));
    }

    @Test
    void testFromInhomogeneousMatrixAndIsValidRotationMatrix() throws WrongSizeException,
            InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D();
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // test from inhomogeneous rotation matrix
        final var inhomRotMat = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        final var cosTheta = Math.cos(theta);
        final var sinTheta = Math.sin(theta);
        inhomRotMat.setElementAt(0, 0, cosTheta);
        inhomRotMat.setElementAt(1, 0, sinTheta);
        inhomRotMat.setElementAt(0, 1, -sinTheta);
        inhomRotMat.setElementAt(1, 1, cosTheta);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat, ABSOLUTE_ERROR));

        // set rotation from inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(inhomRotMat);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // try again with threshold
        rotation.fromInhomogeneousMatrix(inhomRotMat, ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        assertThrows(IllegalArgumentException.class,
                () -> rotation.fromInhomogeneousMatrix(inhomRotMat, -ABSOLUTE_ERROR));

        // Force InvalidRotationMatrix
        // because of wrong size
        final var m1 = new Matrix(2, 3);
        assertFalse(Rotation2D.isValidRotationMatrix(m1));
        assertFalse(Rotation2D.isValidRotationMatrix(m1, ABSOLUTE_ERROR));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromInhomogeneousMatrix(m1));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromInhomogeneousMatrix(m1, ABSOLUTE_ERROR));

        // because determinant is not 1
        final var m2 = Matrix.identity(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        m2.multiplyByScalar(2.0);
        assertFalse(Rotation2D.isValidRotationMatrix(m2));
        assertFalse(Rotation2D.isValidRotationMatrix(m2, ABSOLUTE_ERROR));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromInhomogeneousMatrix(m2));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromInhomogeneousMatrix(m2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException into isValidRotationMatrix because threshold is negative
        assertThrows(IllegalArgumentException.class, () -> Rotation2D.isValidRotationMatrix(m2, -ABSOLUTE_ERROR));
    }

    @Test
    void testFromHomogeneousMatrixAndIsValidRotationMatrix() throws WrongSizeException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D();
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // test from inhomogeneous rotation matrix
        final var homRotMat = new Matrix(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        final var cosTheta = Math.cos(theta);
        final var sinTheta = Math.sin(theta);
        homRotMat.setElementAt(0, 0, cosTheta);
        homRotMat.setElementAt(1, 0, sinTheta);
        homRotMat.setElementAt(2, 0, 0.0);
        homRotMat.setElementAt(0, 1, -sinTheta);
        homRotMat.setElementAt(1, 1, cosTheta);
        homRotMat.setElementAt(2, 1, 0.0);
        homRotMat.setElementAt(0, 2, 0.0);
        homRotMat.setElementAt(1, 2, 0.0);
        homRotMat.setElementAt(2, 2, 1.0);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat, ABSOLUTE_ERROR));

        // set rotation from homogeneous matrix
        rotation.fromHomogeneousMatrix(homRotMat);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(0.0, rotation.getTheta(), 0.0);

        // try again with threshold
        rotation.fromHomogeneousMatrix(homRotMat, ABSOLUTE_ERROR);
        assertEquals(theta, rotation.getTheta(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        assertThrows(IllegalArgumentException.class, () -> rotation.fromHomogeneousMatrix(homRotMat, -ABSOLUTE_ERROR));

        // Force InvalidRotationMatrix
        // because of wrong size
        final var m1 = new Matrix(2, 3);
        assertFalse(Rotation2D.isValidRotationMatrix(m1));
        assertFalse(Rotation2D.isValidRotationMatrix(m1, ABSOLUTE_ERROR));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromHomogeneousMatrix(m1));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromHomogeneousMatrix(m1, ABSOLUTE_ERROR));

        // because determinant is not 1
        final var m2 = Matrix.identity(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS, Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        m2.multiplyByScalar(2.0);
        assertFalse(Rotation2D.isValidRotationMatrix(m2));
        assertFalse(Rotation2D.isValidRotationMatrix(m2, ABSOLUTE_ERROR));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromHomogeneousMatrix(m2));
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromHomogeneousMatrix(m2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException into isValidRotationMatrix because threshold is negative
        assertThrows(IllegalArgumentException.class, () -> Rotation2D.isValidRotationMatrix(m2, -ABSOLUTE_ERROR));
    }

    @Test
    void testRotate() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation = new Rotation2D(theta);

        // create 2 random points
        final var point1 = new HomogeneousPoint2D();
        point1.setInhomogeneousCoordinates(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var point2 = new HomogeneousPoint2D();

        // ensure that points are not coincident
        do {
            point2.setInhomogeneousCoordinates(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        } while (point1.equals(point2));

        // create line passing through both points
        final var line = new Line2D(point1, point2);
        assertTrue(line.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(line.isLocus(point2, ABSOLUTE_ERROR));

        // now rotate points and line
        final var rotPoint1A = rotation.rotate(point1);
        final var rotPoint1B = Point2D.create();
        rotation.rotate(point1, rotPoint1B);

        final var rotPoint2A = rotation.rotate(point2);
        final var rotPoint2B = Point2D.create();
        rotation.rotate(point2, rotPoint2B);

        // check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));

        // check points have been correctly rotated
        final var point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final var point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final var r = rotation.asHomogeneousMatrix();
        final var rotPoint1Mat = r.multiplyAndReturnNew(point1Mat);
        final var rotPoint2Mat = r.multiplyAndReturnNew(point2Mat);

        // check correctness
        var scaleX = rotPoint1A.getHomX() / rotPoint1Mat.getElementAtIndex(0);
        var scaleY = rotPoint1A.getHomY() / rotPoint1Mat.getElementAtIndex(1);
        var scaleW = rotPoint1A.getHomW() / rotPoint1Mat.getElementAtIndex(2);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint2A.getHomX() / rotPoint2Mat.getElementAtIndex(0);
        scaleY = rotPoint2A.getHomY() / rotPoint2Mat.getElementAtIndex(1);
        scaleW = rotPoint2A.getHomW() / rotPoint2Mat.getElementAtIndex(2);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // ensure that points where correctly rotated using inhomogeneous
        // coordinates
        final var inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        final var inhomR = rotation.asInhomogeneousMatrix();
        final var inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final var rotP = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final var rotLineA = rotation.rotate(line);
        final var rotLineB = new Line2D();
        rotation.rotate(line, rotLineB);

        // check both rotated lines are equal
        var scaleA = rotLineA.getA() / rotLineB.getA();
        var scaleB = rotLineA.getB() / rotLineB.getB();
        var scaleC = rotLineA.getC() / rotLineB.getC();

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleA, ABSOLUTE_ERROR);

        // check line has been correctly rotated by ensuring that rotated points
        // belong into rotated line
        assertTrue(rotLineA.isLocus(rotPoint1A, ABSOLUTE_ERROR));
        assertTrue(rotLineA.isLocus(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotLineA.isLocus(rotPoint2A, ABSOLUTE_ERROR));
        assertTrue(rotLineA.isLocus(rotPoint2B, ABSOLUTE_ERROR));

        // and by ensuring that rotated line follow appropriate equation
        final var lineMat = Matrix.newFromArray(line.asArray(), true);
        final var rotLineMat = r.multiplyAndReturnNew(lineMat);

        scaleA = rotLineA.getA() / rotLineMat.getElementAtIndex(0);
        scaleB = rotLineA.getB() / rotLineMat.getElementAtIndex(1);
        scaleC = rotLineA.getC() / rotLineMat.getElementAtIndex(2);

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleA, ABSOLUTE_ERROR);
    }

    @Test
    void testCombine() {
        final var randomizer = new UniformRandomizer();
        final var theta1 = randomizer.nextDouble(MIN_THETA, MAX_THETA);
        final var theta2 = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rot1 = new Rotation2D(theta1);
        final var rot2 = new Rotation2D(theta2);

        final var rot = new Rotation2D();
        Rotation2D.combine(rot1, rot2, rot);
        assertEquals(theta1 + theta2, rot.getTheta(), 0.0);

        final var rot3 = rot.combineAndReturnNew(rot2);
        assertEquals(theta1 + 2.0 * theta2, rot3.getTheta(), ABSOLUTE_ERROR);

        rot.combine(rot1);
        assertEquals(2.0 * theta1 + theta2, rot.getTheta(), ABSOLUTE_ERROR);
    }

    @Test
    void testEqualsAndHashCode() {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);
        final var threshold = randomizer.nextDouble(Rotation2D.DEFAULT_COMPARISON_THRESHOLD,
                2.0 * Rotation2D.DEFAULT_COMPARISON_THRESHOLD);
        final var theta2 = theta + threshold;

        // test from rotation angle
        final var rotation1 = new Rotation2D(theta);
        assertEquals(theta, rotation1.getTheta(), 0.0);

        final var rotation2 = new Rotation2D(theta);
        final var rotation3 = new Rotation2D(theta2);

        // check equal-ness
        //noinspection EqualsWithItself
        assertEquals(rotation1, rotation1);
        assertEquals(rotation1.hashCode(), rotation1.hashCode());

        assertEquals(rotation1, rotation2);
        assertEquals(rotation1.hashCode(), rotation2.hashCode());

        assertNotEquals(rotation1, rotation3);
        assertTrue(rotation1.equals(rotation2, Rotation2D.DEFAULT_COMPARISON_THRESHOLD));
        assertFalse(rotation1.equals(rotation3, Rotation2D.DEFAULT_COMPARISON_THRESHOLD));
        // check with larger threshold
        assertTrue(rotation1.equals(rotation3, 2.0 * threshold));
        assertTrue(rotation1.equals(rotation2));
        assertFalse(rotation1.equals(rotation3));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final var rotation1 = new Rotation2D(theta);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(rotation1);
        final var rotation2 = SerializationHelper.<Rotation2D>deserialize(bytes);

        // check
        assertEquals(rotation1, rotation2);
        assertNotSame(rotation1, rotation2);
    }
}
