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

import com.irurueta.algebra.Utils;
import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class EuclideanTransformation2DTest {

    private static final int HOM_COORDS = 3;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int MIN_POINTS = 3;
    private static final int MAX_POINTS = 50;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_ANGLE_DEGREES2 = -90.0;
    private static final double MAX_ANGLE_DEGREES2 = 90.0;

    private static final double MIN_TRANSLATION2 = -100.0;
    private static final double MAX_TRANSLATION2 = 100.0;

    @Test
    void testConstructor() throws CoincidentPointsException {

        // Test empty constructor
        var transformation = new EuclideanTransformation2D();

        // check correctness
        assertEquals(0.0, transformation.getRotation().getTheta(), 0.0);
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // Test constructor with rotation
        final var randomizer = new UniformRandomizer();
        var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        var rotation = new Rotation2D(theta);

        transformation = new EuclideanTransformation2D(rotation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), 0.0);
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new EuclideanTransformation2D((Rotation2D) null));

        // Test constructor with translation
        var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new EuclideanTransformation2D(translation);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getTheta(), 0.0);
        assertEquals(translation.length, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new EuclideanTransformation2D((double[]) null));

        // Force IllegalArgumentException
        final var badTranslation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new EuclideanTransformation2D(badTranslation));

        // Test constructor with rotation and translation
        transformation = new EuclideanTransformation2D(rotation, translation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), 0.0);
        assertEquals(translation.length, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);

        // Force NullPointerException
        final var finalTranslation = translation;
        assertThrows(NullPointerException.class, () -> new EuclideanTransformation2D(null, finalTranslation));
        final var finalRotation = rotation;
        assertThrows(NullPointerException.class, () -> new EuclideanTransformation2D(finalRotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new EuclideanTransformation2D(finalRotation, badTranslation));

        // test constructor with corresponding points
        theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        rotation = new Rotation2D(theta);
        translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);

        final var inputPoint1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        transformation = new EuclideanTransformation2D(rotation, translation);

        final var outputPoint1 = transformation.transformAndReturnNew(inputPoint1);
        final var outputPoint2 = transformation.transformAndReturnNew(inputPoint2);
        final var outputPoint3 = transformation.transformAndReturnNew(inputPoint3);

        final var transformation2 = new EuclideanTransformation2D(inputPoint1, inputPoint2, inputPoint3, outputPoint1,
                outputPoint2, outputPoint3);

        // check correctness
        final var rotation2 = transformation2.getRotation();
        final var translation2 = transformation2.getTranslation();

        assertEquals(rotation2.getTheta(), rotation.getTheta(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetRotation() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new Rotation2D(theta);

        // test default values
        assertEquals(0.0, transformation.getRotation().getTheta(), 0.0);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), 0.0);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> transformation.setRotation(null));
    }

    @Test
    void testAddRotation() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var combinedTheta = theta1 + theta2;

        final var rotation1 = new Rotation2D(theta1);
        final var rotation2 = new Rotation2D(theta2);

        // set rotation1
        transformation.setRotation(rotation1);

        // check correctness
        assertEquals(theta1, transformation.getRotation().getTheta(), 0.0);

        // add second rotation
        transformation.addRotation(rotation2);

        // check correctness
        assertEquals(combinedTheta, transformation.getRotation().getTheta(), 0.0);
    }

    @Test
    void testGetSetTranslation() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);

        // Force IllegalArgumentException
        final var badTranslation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.setTranslation(badTranslation));
    }

    @Test
    void testAddTranslation() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, EuclideanTransformation2D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation1[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation1[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation1[1], transformation.getTranslationY(), 0.0);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), 0.0);

        // Force IllegalArgumentException
        final var badTranslation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.addTranslation(badTranslation));
    }

    @Test
    void testGetSetTranslationX() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);

        // set new value
        transformation.setTranslationX(translationX);

        // check correctness
        assertEquals(translationX, transformation.getTranslationX(), 0.0);
    }

    @Test
    void testGetSetTranslationY() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set new value
        transformation.setTranslationY(translationY);

        // check correctness
        assertEquals(translationY, transformation.getTranslationY(), 0.0);
    }

    @Test
    void testSetTranslationCoordinates() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY);

        // check correctness
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationPoint() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation = new InhomogeneousPoint2D(translationX, translationY);

        // check default value
        assertEquals(EuclideanTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(AffineTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);

        final var translation2 = transformation.getTranslationPoint();
        final var translation3 = Point2D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    void testAddTranslationX() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);

        // set value
        transformation.setTranslationX(translationX1);

        // check correctness
        assertEquals(translationX1, transformation.getTranslationX(), 0.0);

        // add translation x
        transformation.addTranslationX(translationX2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
    }

    @Test
    void testAddTranslationY() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set value
        transformation.setTranslationY(translationY1);

        // check correctness
        assertEquals(translationY1, transformation.getTranslationY(), 0.0);

        // add translation y
        transformation.addTranslationY(translationY2);

        // check correctness
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
    }

    @Test
    void testAddTranslationCoordinates() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1);

        // add translation
        transformation.addTranslation(translationX2, translationY2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
    }

    @Test
    void testAddTranslationPoint() {
        final var transformation = new EuclideanTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1);

        // add translation
        final var translation2 = new InhomogeneousPoint2D(translationX2, translationY2);
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var rotation = new Rotation2D(theta);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var m = Matrix.identity(3, 3);
        m.setSubmatrix(0, 0, 1, 1,
                rotation.asInhomogeneousMatrix());
        m.setSubmatrix(0, 2, 1, 2, translation);

        final var transMatrix1 = transformation.asMatrix();
        final var transMatrix2 = new Matrix(EuclideanTransformation2D.HOM_COORDS, EuclideanTransformation2D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var t = new Matrix(EuclideanTransformation2D.HOM_COORDS + 1,
                EuclideanTransformation2D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.asMatrix(t));
    }

    @Test
    void testTransformPoint() {
        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var expectedPoint = Point2D.create();
        transformPoint(point, expectedPoint, rotation, translation);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var outPoint1 = transformation.transformAndReturnNew(point);
        final var outPoint2 = Point2D.create();
        transformation.transform(point, outPoint2);

        // check correctness
        assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
        assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));

        // update point
        transformation.transform(point);

        // check correctness
        assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
    }

    @Test
    void testTransformPoints() {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation);

            expectedPoints.add(expectedPoint);
        }

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        final var outPoints2 = new ArrayList<Point2D>();
        transformation.transformPoints(inputPoints, outPoints2);

        // check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPoints.get(i);

            final var outPoint1 = outPoints1.get(i);
            final var outPoint2 = outPoints2.get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformAndOverwritePoints() {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation);

            expectedPoints.add(expectedPoint);
        }

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        transformation.transformAndOverwritePoints(inputPoints);

        // check correctness
        assertEquals(inputPoints.size(), size);
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPoints.get(i);

            final var point = inputPoints.get(i);

            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformConic() throws AlgebraException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var conic = new Conic(a, b, c, d, e, f);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        // compute expected value
        final var expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        // make transformation
        final var outConic1 = transformation.transformAndReturnNew(conic);
        final var outConic2 = new Conic();
        transformation.transform(conic, outConic2);

        // check correctness
        outConic1.normalize();
        outConic2.normalize();

        assertEquals(expectedConic.getA(), outConic1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic1.getF(), ABSOLUTE_ERROR);

        assertEquals(expectedConic.getA(), outConic2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic2.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic2.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic2.getF(), ABSOLUTE_ERROR);

        transformation.transform(conic);

        // check correctness
        conic.normalize();

        assertEquals(expectedConic.getA(), conic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), conic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), conic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), conic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), conic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), conic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformConicAndPoints() throws AlgebraException, GeometryException {
        // create Conic from 5 points
        Conic conic = null;
        Point2D point1;
        Point2D point2;
        Point2D point3;
        Point2D point4;
        Point2D point5;
        do {
            final var m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1), 1.0);
            point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1), 1.0);
            point3 = new HomogeneousPoint2D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1), 1.0);
            point4 = new HomogeneousPoint2D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1), 1.0);
            point5 = new HomogeneousPoint2D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1), 1.0);

            try {
                conic = new Conic(point1, point2, point3, point4, point5);
            } catch (final GeometryException ignore) {
                // if points are not valid, ignore and continue
            }
        } while (conic == null);

        // check that points belong to conic
        assertTrue(conic.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point5, ABSOLUTE_ERROR));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        // compute expected value
        final var expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        // transform conic and points
        final var outConic = transformation.transformAndReturnNew(conic);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point2);
        final var outPoint3 = transformation.transformAndReturnNew(point3);
        final var outPoint4 = transformation.transformAndReturnNew(point4);
        final var outPoint5 = transformation.transformAndReturnNew(point5);

        // check that transformed points still belong to transformed conic
        assertTrue(outConic.isLocus(outPoint1, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint2, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint3, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint4, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint5, ABSOLUTE_ERROR));

        // check conic correctness
        outConic.normalize();

        assertEquals(expectedConic.getA(), outConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    void tesTransformDualConic() throws NonSymmetricMatrixException, AlgebraException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dualConic = new DualConic(a, b, c, d, e, f);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        // compute expected value
        final var expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        // make transformation
        final var outDualConic1 = transformation.transformAndReturnNew(dualConic);
        final var outDualConic2 = new DualConic();
        transformation.transform(dualConic, outDualConic2);

        // check correctness
        outDualConic1.normalize();
        outDualConic2.normalize();

        assertEquals(expectedDualConic.getA(), outDualConic1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic1.getF(), ABSOLUTE_ERROR);

        assertEquals(expectedDualConic.getA(), outDualConic2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic2.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic2.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic2.getF(), ABSOLUTE_ERROR);

        transformation.transform(dualConic);

        // check correctness
        dualConic.normalize();

        assertEquals(expectedDualConic.getA(), dualConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), dualConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), dualConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), dualConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), dualConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), dualConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformDualConicAndLines() throws AlgebraException, GeometryException {
        // create dual conic from 5 lines
        DualConic dualConic = null;
        Line2D line1;
        Line2D line2;
        Line2D line3;
        Line2D line4;
        Line2D line5;
        do {
            final var m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line1 = new Line2D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();

            try {
                dualConic = new DualConic(line1, line2, line3, line4, line5);
            } catch (final GeometryException ignore) {
                // if lines are not valid, ignore and continue
            }
        } while (dualConic == null);

        // check that lines belong to dual conic
        assertTrue(dualConic.isLocus(line1, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line2, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line3, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line4, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line5, ABSOLUTE_ERROR));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        // compute expected value
        final var expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        // transform dual conic and lines
        final var outDualConic = transformation.transformAndReturnNew(dualConic);
        final var outLine1 = transformation.transformAndReturnNew(line1);
        final var outLine2 = transformation.transformAndReturnNew(line2);
        final var outLine3 = transformation.transformAndReturnNew(line3);
        final var outLine4 = transformation.transformAndReturnNew(line4);
        final var outLine5 = transformation.transformAndReturnNew(line5);

        // check that transformed lines still belong to transformed dual conic
        assertTrue(outDualConic.isLocus(outLine1, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine2, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine3, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine4, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine5, ABSOLUTE_ERROR));

        // check dual conic correctness
        outDualConic.normalize();

        assertEquals(expectedDualConic.getA(), outDualConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformLine() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var params = new double[Line2D.LINE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var line = new Line2D(params);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final var outLine1 = transformation.transformAndReturnNew(line);
        final var outLine2 = new Line2D();
        transformation.transform(line, outLine2);

        outLine1.normalize();
        outLine2.normalize();

        // check correctness
        assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);

        transformation.transform(line);

        line.normalize();

        // check correctness
        assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformLineAndPoints() throws AlgebraException {
        // create line from 2 points
        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 2 non-linear dependent rows
        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1), m.getElementAt(0, 2));
        final var point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1), m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        final var line = new Line2D(point1, point2);

        // check that points belong to the line
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        // transform line and points
        final var outLine = transformation.transformAndReturnNew(line);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point2);

        // check that transformed points still belong to transformed line
        assertTrue(outLine.isLocus(outPoint1));
        assertTrue(outLine.isLocus(outPoint2));

        // check line correctness
        outLine.normalize();

        assertEquals(expectedLine.getA(), outLine.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine.getC(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformLines() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var inputLines = new ArrayList<Line2D>(size);
        final var expectedLines = new ArrayList<Line2D>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var line = new Line2D(params);
            inputLines.add(line);

            final var expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        final var outLines1 = transformation.transformLinesAndReturnNew(inputLines);
        final var outLines2 = new ArrayList<Line2D>();
        transformation.transformLines(inputLines, outLines2);

        // check correctness
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
        for (var i = 0; i < size; i++) {
            final var expectedLine = expectedLines.get(i);

            final var outLine1 = outLines1.get(i);
            final var outLine2 = outLines2.get(i);

            expectedLine.normalize();
            outLine1.normalize();
            outLine2.normalize();

            // check correctness
            assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformAndOverwriteLines() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var inputLines = new ArrayList<Line2D>(size);
        final var expectedLines = new ArrayList<Line2D>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var line = new Line2D(params);
            inputLines.add(line);

            final var expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        transformation.transformAndOverwriteLines(inputLines);

        // check correctness
        assertEquals(inputLines.size(), size);
        for (var i = 0; i < size; i++) {
            final var expectedLine = expectedLines.get(i);

            final var line = inputLines.get(i);

            expectedLine.normalize();
            line.normalize();

            // check correctness
            assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformPolygon() throws NotEnoughVerticesException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation);

            expectedPoints.add(expectedPoint);
        }

        final var inputPolygon = new Polygon2D(inputPoints);
        final var expectedPolygon = new Polygon2D(expectedPoints);

        final var outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        final var outPolygon2 = new Polygon2D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);

        // check correctness
        assertEquals(outPolygon1.getVertices().size(), inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(), inputPolygon.getVertices().size());
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPolygon.getVertices().get(i);

            final var outPoint1 = outPolygon1.getVertices().get(i);
            final var outPoint2 = outPolygon2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputPolygon);

        // check correctness
        assertEquals(expectedPolygon.getVertices().size(), inputPolygon.getVertices().size());
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPolygon.getVertices().get(i);

            final var outPoint = outPolygon1.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformTriangle() {

        final var randomizer = new UniformRandomizer();
        final var size = Triangle2D.NUM_VERTICES;

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation);

            expectedPoints.add(expectedPoint);
        }

        final var inputTriangle = new Triangle2D(inputPoints.get(0), inputPoints.get(1), inputPoints.get(2));
        final var expectedTriangle = new Triangle2D(expectedPoints.get(0), expectedPoints.get(1),
                expectedPoints.get(2));

        final var outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        final var outTriangle2 = new Triangle2D(
                new InhomogeneousPoint2D(inputPoints.get(0)),
                new InhomogeneousPoint2D(inputPoints.get(1)),
                new InhomogeneousPoint2D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);

        // check correctness
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedTriangle.getVertices().get(i);

            final var outPoint1 = outTriangle1.getVertices().get(i);
            final var outPoint2 = outTriangle2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputTriangle);

        // check correctness
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedTriangle.getVertices().get(i);

            final var outPoint = inputTriangle.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testToMetric() {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var expectedMatrix = transformation.asMatrix();

        final var metricMatrix = transformation.toMetric().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testInverse() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var invTransformation1 = transformation.inverseAndReturnNew();
        final var invTransformation2 = new EuclideanTransformation2D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(transformation.asMatrix()).equals(Matrix.identity(
                EuclideanTransformation2D.HOM_COORDS, EuclideanTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(transformation.asMatrix()).equals(Matrix.identity(
                EuclideanTransformation2D.HOM_COORDS, EuclideanTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final var params = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoint = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

        final var transfPoint = transformation.transformAndReturnNew(inputPoint);

        final var invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        final var invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);

        // check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));

        // try inverting original transformation
        transformation.inverse();
        final var outPoint = transformation.transformAndReturnNew(transfPoint);

        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }

    @Test
    void testCombine() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation1 = new Rotation2D(theta1);
        final var rotation2 = new Rotation2D(theta2);

        final var translation1 = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation1 = new EuclideanTransformation2D(rotation1, translation1);
        final var transformation2 = new EuclideanTransformation2D(rotation2, translation2);

        final var expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(transformation2.asMatrix());

        final var expectedRotation = rotation1.combineAndReturnNew(rotation2);
        final var rotM1 = rotation1.asInhomogeneousMatrix();
        final var t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        final var expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);

        // combine and return result as a new transformation
        final var transformation3 = transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        final var m3 = transformation3.asMatrix();
        final var m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check also correctness of rotation and translation
        assertEquals(transformation1.getRotation().getTheta(), expectedRotation.getTheta(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getTheta(), expectedRotation.getTheta(), ABSOLUTE_ERROR);

        assertArrayEquals(transformation1.getTranslation(), expectedTranslation, ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation, ABSOLUTE_ERROR);
    }

    @Test
    void testSetTransformationFromPoints() throws CoincidentPointsException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);

        final var transformation = new EuclideanTransformation2D(rotation, translation);

        final var inputPoint1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var outputPoint1 = transformation.transformAndReturnNew(inputPoint1);
        final var outputPoint2 = transformation.transformAndReturnNew(inputPoint2);
        final var outputPoint3 = transformation.transformAndReturnNew(inputPoint3);

        final var transformation2 = new EuclideanTransformation2D();
        transformation2.setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, outputPoint1, outputPoint2,
                outputPoint3);

        // check correctness
        final var rotation2 = transformation2.getRotation();
        final var translation2 = transformation2.getTranslation();

        assertEquals(rotation2.getTheta(), rotation.getTheta(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);

        final var translation = new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation1 = new EuclideanTransformation2D(rotation, translation);

        // check
        assertSame(rotation, transformation1.getRotation());
        assertSame(translation, transformation1.getTranslation());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(transformation1);
        final var transformation2 = SerializationHelper.<EuclideanTransformation2D>deserialize(bytes);

        // check
        assertEquals(transformation1.getRotation(), transformation2.getRotation());
        assertNotSame(transformation1.getRotation(), transformation2.getRotation());
        assertArrayEquals(transformation1.getTranslation(), transformation2.getTranslation(), 0.0);
        assertNotSame(transformation1.getTranslation(), transformation2.getTranslation());
    }

    private static void transformPoint(
            final Point2D inputPoint, final Point2D outputPoint, final Rotation2D rotation,
            final double[] translation) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(outputPoint.getInhomX() + translation[0],
                outputPoint.getInhomY() + translation[1]);
    }

    private static void transformLine(
            final Line2D inputLine, final Line2D outputLine, final EuclideanTransformation2D transformation)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {
        inputLine.normalize();
        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();
        final var l = Matrix.newFromArray(inputLine.asArray(), true);

        outputLine.setParameters(transInvT.multiplyAndReturnNew(l).toArray());
    }

    private static void transformConic(
            final Conic inputConic, final Conic outputConic, final EuclideanTransformation2D transformation)
            throws AlgebraException, NonSymmetricMatrixException {

        final var t = transformation.asMatrix();
        final var invT = Utils.inverse(t);
        var norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();

        inputConic.normalize();
        final var c = inputConic.asMatrix();

        final var transC = transInvT.multiplyAndReturnNew(c.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transC);
        transC.multiplyByScalar(1.0 / norm);

        outputConic.setParameters(transC);
    }

    private static void transformDualConic(
            final DualConic inputDualConic, final DualConic outputDualConic,
            final EuclideanTransformation2D transformation) throws WrongSizeException, NonSymmetricMatrixException {

        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var transT = t.transposeAndReturnNew();

        inputDualConic.normalize();
        final var dualC = inputDualConic.asMatrix();

        final var transDualC = t.multiplyAndReturnNew(dualC.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualC);
        transDualC.multiplyByScalar(1.0 / norm);

        outputDualConic.setParameters(transDualC);
    }
}
