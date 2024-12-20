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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class EllipseTest {

    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_DEGREES = -90.0;
    private static final double MAX_RANDOM_DEGREES = 90.0;

    private static final int TIMES = 500;

    @Test
    void testConstants() {
        assertEquals(1e-9, Ellipse.DEFAULT_THRESHOLD, 0.0);
        assertEquals(0.0, Ellipse.MIN_THRESHOLD, 0.0);
    }

    @Test
    void testConstructor() throws ColinearPointsException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // test empty constructor
            var ellipse = new Ellipse();

            // check default values
            assertEquals(new InhomogeneousPoint2D(), ellipse.getCenter());
            assertEquals(1.0, ellipse.getSemiMajorAxis(), 0.0);
            assertEquals(1.0, ellipse.getSemiMinorAxis(), 0.0);
            assertEquals(0.0, ellipse.getRotationAngle(), 0.0);

            // test constructor with center, axes and rotation
            final var randomizer = new UniformRandomizer();
            final var center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
            final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
            final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
            ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

            // check correctness
            assertEquals(center, ellipse.getCenter());
            assertEquals(semiMajorAxis, ellipse.getSemiMajorAxis(), 0.0);
            assertEquals(semiMinorAxis, ellipse.getSemiMinorAxis(), 0.0);
            assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);

            ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, new Rotation2D(rotationAngle));

            // check correctness
            assertEquals(center, ellipse.getCenter());
            assertEquals(semiMajorAxis, ellipse.getSemiMajorAxis(), 0.0);
            assertEquals(semiMinorAxis, ellipse.getSemiMinorAxis(), 0.0);
            assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);

            // test with 2 points, center and rotation angle
            var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

            Point2D point1;
            Point2D point2;
            boolean areEqual;
            do {
                var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point1 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point2 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);

                // ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR);
            } while (areEqual);

            ellipse = new Ellipse(point1, point2, center, rotationAngle);

            // check parameters
            assertEquals(ellipse.getCenter(), center);
            assertEquals(radius, ellipse.getSemiMajorAxis(), ABSOLUTE_ERROR);
            assertEquals(radius, ellipse.getSemiMinorAxis(), ABSOLUTE_ERROR);
            assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);

            if (!ellipse.isLocus(point1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, ABSOLUTE_ERROR));
            if (ellipse.isLocus(center, ABSOLUTE_ERROR)) {
                continue;
            }
            assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));

            // test with 5 points (without threshold)
            radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

            Point2D point3;
            Point2D point4;
            Point2D point5;
            do {
                var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point1 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point2 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point3 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point4 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point5 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);

                // ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR) || point2.equals(point3, ABSOLUTE_ERROR)
                        || point3.equals(point1, ABSOLUTE_ERROR) || point4.equals(point1, ABSOLUTE_ERROR)
                        || point5.equals(point1, ABSOLUTE_ERROR);
            } while (areEqual);

            try {
                ellipse = new Ellipse(point1, point2, point3, point4, point5);
            } catch (final ColinearPointsException e) {
                continue;
            }

            // check parameters
            if (!ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - radius) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), radius, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - radius) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), radius, LARGE_ABSOLUTE_ERROR);

            if (!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));

            ellipse = new Ellipse(point1, point2, point3, point4, point5, ABSOLUTE_ERROR);

            // check parameters
            assertTrue(ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR));
            assertEquals(ellipse.getSemiMajorAxis(), radius, LARGE_ABSOLUTE_ERROR);
            assertEquals(ellipse.getSemiMinorAxis(), radius, LARGE_ABSOLUTE_ERROR);

            if (!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));

            // constructor with parameters
            var a = ellipse.getA();
            var b = ellipse.getB();
            var c = ellipse.getC();
            var d = ellipse.getD();
            var e = ellipse.getE();
            var f = ellipse.getF();

            var ellipse2 = new Ellipse(a, b, c, d, e, f);

            // check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);

            ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

            a = ellipse.getA();
            b = ellipse.getB();
            c = ellipse.getC();
            d = ellipse.getD();
            e = ellipse.getE();
            f = ellipse.getF();

            ellipse2 = new Ellipse(a, b, c, d, e, f, ABSOLUTE_ERROR);

            // check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            assertEquals(ellipse.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);

            if (Math.abs(ellipse.getRotationAngle() - ellipse2.getRotationAngle()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getRotationAngle(), ellipse2.getRotationAngle(), ABSOLUTE_ERROR);

            final var conic = ellipse.toConic();

            ellipse2 = new Ellipse(conic);

            // check correctness
            if (ellipse.getCenter().distanceTo(ellipse2.getCenter()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, ellipse.getCenter().distanceTo(ellipse2.getCenter()), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getRotationAngle() - ellipse2.getRotationAngle()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getRotationAngle(), ellipse2.getRotationAngle(), ABSOLUTE_ERROR);

            // Force IllegalArgumentException
            final var conic2 = new Conic();
            assertThrows(IllegalArgumentException.class, () -> new Ellipse(conic2));

            final var circle = new Circle(center, radius);
            ellipse = new Ellipse(circle);

            // check correctness
            assertEquals(circle.getCenter(), ellipse.getCenter());
            assertEquals(circle.getRadius(), ellipse.getSemiMajorAxis(), 0.0);
            assertEquals(circle.getRadius(), ellipse.getSemiMinorAxis(), 0.0);
            assertEquals(0.0, ellipse.getRotationAngle(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetSetCenter() {
        final var ellipse = new Ellipse();

        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // new value
        ellipse.setCenter(center);

        // check correctness
        assertEquals(center, ellipse.getCenter());
    }

    @Test
    void testGetSetSemiMajorAxis() {
        final var ellipse = new Ellipse();

        // initial value
        assertEquals(1.0, ellipse.getSemiMajorAxis(), 0.0);

        // new value
        final var randomizer = new UniformRandomizer();
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);

        ellipse.setSemiMajorAxis(semiMajorAxis);

        // check correctness
        assertEquals(semiMajorAxis, ellipse.getSemiMajorAxis(), 0.0);
    }

    @Test
    void testGetSetSemiMinorAxis() {
        final var ellipse = new Ellipse();

        // initial value
        assertEquals(1.0, ellipse.getSemiMinorAxis(), 0.0);

        // new value
        final var randomizer = new UniformRandomizer();
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);

        ellipse.setSemiMinorAxis(semiMinorAxis);

        // check correctness
        assertEquals(semiMinorAxis, ellipse.getSemiMinorAxis(), 0.0);
    }

    @Test
    void testGetSetRotationAngle() {
        final var ellipse = new Ellipse();

        // initial value
        assertEquals(0.0, ellipse.getRotationAngle(), 0.0);

        // new value
        final var randomizer = new UniformRandomizer();
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        ellipse.setRotationAngle(rotationAngle);

        // check correctness
        assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);
    }

    @Test
    void testGetSetRotation() {
        final var ellipse = new Ellipse();

        // initial value
        assertEquals(0.0, ellipse.getRotation().getTheta(), 0.0);

        // new value
        final var randomizer = new UniformRandomizer();
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var rotation = new Rotation2D(rotationAngle);
        ellipse.setRotation(rotation);

        // check correctness
        assertEquals(rotationAngle, ellipse.getRotation().getTheta(), 0.0);
        assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);
    }

    @Test
    void testGetSetA() {
        final var ellipse = new Ellipse();

        assertEquals(1.0, ellipse.getA(), ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var a = randomizer.nextDouble(ABSOLUTE_ERROR, MAX_RANDOM_VALUE);

        ellipse.setA(a);

        // check correctness
        assertEquals(1.0, ellipse.getA(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetB() {
        final var ellipse = new Ellipse();

        assertEquals(0.0, ellipse.getB(), ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = randomizer.nextDouble(0.0, Math.sqrt(4.0 * ellipse.getA() * ellipse.getC()));

        ellipse.setB(b);

        // check correctness
        assertTrue(Math.abs(ellipse.getB()) > ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetC() {
        final var ellipse = new Ellipse();

        assertEquals(1.0, ellipse.getC(), ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var c = randomizer.nextDouble(ellipse.getB() * ellipse.getB() / (4.0 * ellipse.getA()), MAX_RANDOM_VALUE);

        ellipse.setC(c);

        // check correctness
        assertEquals(1.0, ellipse.getC(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetD() {
        final var ellipse = new Ellipse();

        assertEquals(0.0, ellipse.getD(), ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        ellipse.setD(d);

        // check correctness
        assertTrue(Math.abs(ellipse.getD()) > ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetE() {
        final var ellipse = new Ellipse();

        assertEquals(0.0, ellipse.getE(), ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        ellipse.setE(e);

        // check correctness
        assertTrue(Math.abs(ellipse.getE()) > ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetF() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ellipse = new Ellipse();

            assertEquals(-1.0, ellipse.getF(), ABSOLUTE_ERROR);

            // set new value
            final var randomizer = new UniformRandomizer();
            final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            ellipse.setF(f);

            if (Math.abs(ellipse.getF() + 1.0) > ABSOLUTE_ERROR) {
                numValid++;
            }
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testSetCenterAxesAndRotationAngle() {
        final var ellipse = new Ellipse();

        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        ellipse.setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        // check correctness
        assertEquals(center, ellipse.getCenter());
        assertEquals(semiMajorAxis, ellipse.getSemiMajorAxis(), 0.0);
        assertEquals(semiMinorAxis, ellipse.getSemiMinorAxis(), 0.0);
        assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);
    }

    @Test
    void testSetCenterAxesAndRotation() {
        final var ellipse = new Ellipse();

        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var rotation = new Rotation2D(rotationAngle);

        ellipse.setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, rotation);

        // check correctness
        assertEquals(center, ellipse.getCenter());
        assertEquals(semiMajorAxis, ellipse.getSemiMajorAxis(), 0.0);
        assertEquals(semiMinorAxis, ellipse.getSemiMinorAxis(), 0.0);
        assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);
    }

    @Test
    void testSetParametersFromPointsCenterAndRotation() throws ColinearPointsException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ellipse = new Ellipse();

            final var randomizer = new UniformRandomizer();
            final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

            final var center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

            Point2D point1;
            Point2D point2;
            boolean areEqual;
            do {
                var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point1 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point2 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);

                // ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR);
            } while (areEqual);

            ellipse.setParametersFromPointsCenterAndRotation(point1, point2, center, rotationAngle);

            // check parameters
            assertEquals(center, ellipse.getCenter());
            assertEquals(radius, ellipse.getSemiMajorAxis(), ABSOLUTE_ERROR);
            assertEquals(radius, ellipse.getSemiMinorAxis(), ABSOLUTE_ERROR);
            assertEquals(rotationAngle, ellipse.getRotationAngle(), 0.0);
            if (!ellipse.isLocus(point1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, ABSOLUTE_ERROR));
            if (ellipse.isLocus(center, ABSOLUTE_ERROR)) {
                continue;
            }
            assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetParametersFromPoints() throws ColinearPointsException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ellipse = new Ellipse();

            final var randomizer = new UniformRandomizer();
            final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
            final var center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            Point2D point1;
            Point2D point2;
            Point2D point3;
            Point2D point4;
            Point2D point5;
            boolean areEqual;
            do {
                var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point1 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point2 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point3 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point4 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
                point5 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);

                // ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR) || point2.equals(point3, ABSOLUTE_ERROR)
                        || point3.equals(point1, ABSOLUTE_ERROR) || point4.equals(point1, ABSOLUTE_ERROR)
                        || point5.equals(point1, ABSOLUTE_ERROR);
            } while (areEqual);

            try {
                ellipse.setParametersFromPoints(point1, point2, point3, point4, point5);
            } catch (final ColinearPointsException e) {
                continue;
            }

            // check parameters
            if (!ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - radius) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), radius, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - radius) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(radius, ellipse.getSemiMinorAxis(), LARGE_ABSOLUTE_ERROR);

            if (!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));

            // use threshold
            ellipse.setParametersFromPoints(point1, point2, point3, point4, point5, ABSOLUTE_ERROR);

            // check parameters
            if (!ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - radius) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), radius, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - radius) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(radius, ellipse.getSemiMinorAxis(), LARGE_ABSOLUTE_ERROR);

            if (!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetParameters() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
            final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
            final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

            final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

            final var a = ellipse.getA();
            final var b = ellipse.getB();
            final var c = ellipse.getC();
            final var d = ellipse.getD();
            final var e = ellipse.getE();
            final var f = ellipse.getF();

            final var ellipse2 = new Ellipse();
            ellipse2.setParameters(a, b, c, d, e, f);

            // check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);

            // test again with threshold
            ellipse2.setParameters(a, b, c, d, e, f, ABSOLUTE_ERROR);

            // check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetFocus() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        assertEquals(ellipse.getFocus(), Math.sqrt(Math.pow(semiMajorAxis, 2.0) - Math.pow(semiMinorAxis, 2.0)),
                ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetSemiMajorAxisCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        final var coords1 = new double[2];
        ellipse.getSemiMajorAxisCoordinates(coords1);

        final var coords2 = ellipse.getSemiMajorAxisCoordinates();

        assertArrayEquals(coords1, coords2, ABSOLUTE_ERROR);

        assertEquals(coords1[0], semiMajorAxis * Math.cos(rotationAngle), ABSOLUTE_ERROR);
        assertEquals(coords1[1], semiMajorAxis * Math.sin(rotationAngle), ABSOLUTE_ERROR);

        // semi major and semi minor coordinates are orthogonal
        assertEquals(0.0, ArrayUtils.dotProduct(coords1, ellipse.getSemiMinorAxisCoordinates()),
                ABSOLUTE_ERROR);

        final var ellipse2 = new Ellipse();
        ellipse2.setSemiMajorAxisCoordinates(coords1);

        assertArrayEquals(coords1, ellipse2.getSemiMajorAxisCoordinates(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetSemiMinorAxisCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        final var coords1 = new double[2];
        ellipse.getSemiMinorAxisCoordinates(coords1);

        final var coords2 = ellipse.getSemiMinorAxisCoordinates();

        assertArrayEquals(coords1, coords2, ABSOLUTE_ERROR);

        assertEquals(coords1[0], -semiMinorAxis * Math.sin(rotationAngle), ABSOLUTE_ERROR);
        assertEquals(coords1[1], semiMinorAxis * Math.cos(rotationAngle), ABSOLUTE_ERROR);

        // semi major and semi minor coordinates are orthogonal
        assertEquals(0.0, ArrayUtils.dotProduct(ellipse.getSemiMajorAxisCoordinates(), coords1),
                ABSOLUTE_ERROR);

        final var ellipse2 = new Ellipse();
        ellipse2.setSemiMinorAxisCoordinates(coords1);

        assertArrayEquals(ellipse2.getSemiMinorAxisCoordinates(), coords1, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetFocusPoints() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
            final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
            final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

            final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

            final var focus = ellipse.getFocus();

            final var p1a = Point2D.create();
            ellipse.getFocusPoint1(p1a);

            final var p1b = ellipse.getFocusPoint1();

            assertEquals(p1a.distanceTo(center), focus, ABSOLUTE_ERROR);
            assertEquals(p1b.distanceTo(center), focus, ABSOLUTE_ERROR);

            final var p2a = Point2D.create();
            ellipse.getFocusPoint2(p2a);

            final var p2b = ellipse.getFocusPoint2();

            assertEquals(p2a.distanceTo(center), focus, ABSOLUTE_ERROR);
            assertEquals(p2b.distanceTo(center), focus, ABSOLUTE_ERROR);

            final var line = new Line2D(p1a, p2a);
            assertTrue(line.isLocus(center, ABSOLUTE_ERROR));

            final var ellipse2 = new Ellipse(center, semiMajorAxis, semiMinorAxis,
                    0.0);

            ellipse2.setFocusPoints(p1a, p2a, true);

            assertEquals(ellipse2.getSemiMinorAxis(), semiMinorAxis, ABSOLUTE_ERROR);
            if (!ellipse2.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse2.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR));
            if (!ellipse2.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse2.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR));

            final var ellipse3 = new Ellipse(center, semiMajorAxis, semiMinorAxis, 0.0);

            ellipse3.setFocusPoints(p1a, p2a, false);

            assertEquals(ellipse3.getSemiMajorAxis(), semiMajorAxis, ABSOLUTE_ERROR);
            if (!ellipse3.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse3.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR));
            if (!ellipse3.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse3.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testGetEccentricity() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        final var focus = ellipse.getFocus();

        assertEquals(ellipse.getEccentricity(), focus / semiMajorAxis, ABSOLUTE_ERROR);
    }

    @Test
    void testGetArea() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        assertEquals(ellipse.getArea(), Math.PI * semiMajorAxis * semiMinorAxis, ABSOLUTE_ERROR);

        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var c = new Circle(center, radius);

        ellipse.setFromConic(c.toConic());

        assertEquals(ellipse.getArea(), c.getArea(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetPerimeter() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        assertEquals(ellipse.getPerimeter(),
                Math.PI * (3.0 * (semiMajorAxis + semiMinorAxis)
                        - Math.sqrt((3.0 * semiMajorAxis + semiMinorAxis) * (semiMajorAxis + 3.0 * semiMinorAxis))),
                ABSOLUTE_ERROR);

        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var c = new Circle(center, radius);

        ellipse.setFromConic(c.toConic());

        assertEquals(ellipse.getPerimeter(), c.getPerimeter(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetCurvature() {
        // build an ellipse like a circle
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var ellipse = new Ellipse(center, radius, radius, 0.0);

        final var angle = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var point1 = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(angle),
                center.getInhomY() + radius * Math.sin(angle), 1.0);

        assertTrue(ellipse.isLocus(point1, ABSOLUTE_ERROR));

        // curvature for a circle is equal to the reciprocal of its radius
        assertEquals(ellipse.getCurvature(point1), 1.0 / radius, ABSOLUTE_ERROR);
    }

    @Test
    void testToConic() {
        final var randomizer = new UniformRandomizer();
        final var center = new HomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        center.normalize();

        final var ellipse = new Ellipse(center, radius, radius, 0.0);
        final var circle = new Circle(center, radius);
        final var conic = ellipse.toConic();
        final var conic2 = circle.toConic();

        conic.normalize();
        conic2.normalize();

        // center is not locus
        assertFalse(ellipse.isLocus(center));
        assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(conic.isLocus(center));
        assertFalse(conic.isLocus(center, ABSOLUTE_ERROR));

        final var locus = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0);
        locus.normalize();

        // test is locus
        assertTrue(ellipse.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(locus, ABSOLUTE_ERROR));

        // check correctness of estimated conic matrix (up to scale)
        assertTrue(conic.asMatrix().equals(conic2.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testSetFromConic() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
            final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
            final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

            final var ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

            // test from conic
            var conic = ellipse.toConic();
            final var ellipse2 = new Ellipse();
            ellipse2.setFromConic(conic);

            // check correctness
            if (ellipse.getCenter().distanceTo(ellipse2.getCenter()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, ellipse.getCenter().distanceTo(ellipse2.getCenter()), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getRotationAngle() - ellipse2.getRotationAngle()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getRotationAngle(), ellipse2.getRotationAngle(), ABSOLUTE_ERROR);

            // Force IllegalArgumentException
            final var wrong = new Conic();
            assertThrows(IllegalArgumentException.class, () -> ellipse2.setFromConic(wrong));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetFromCircle() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);

        final var circle = new Circle(center, radius);

        final var ellipse = new Ellipse();
        ellipse.setFromCircle(circle);

        // check correctness
        assertEquals(circle.getCenter(), ellipse.getCenter());
        assertEquals(circle.getRadius(), ellipse.getSemiMajorAxis(), 0.0);
        assertEquals(circle.getRadius(), ellipse.getSemiMinorAxis(), 0.0);
        assertEquals(0.0, ellipse.getRotationAngle(), 0.0);
    }

    @Test
    void testIsInside() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, radius, radius, 0.0);

        final var inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final var outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));

        // check correctness
        assertTrue(ellipse.isInside(inside));
        assertTrue(ellipse.isInside(inside, ABSOLUTE_ERROR));

        assertFalse(ellipse.isInside(outside));
        assertFalse(ellipse.isInside(outside, ABSOLUTE_ERROR));

        // test for a large positive threshold
        assertTrue(ellipse.isInside(inside, radius * radius));

        assertFalse(ellipse.isInside(outside, -radius * radius));
    }

    @Test
    void testIsLocus() {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var value = randomizer.nextDouble(0.2, 0.8);
        final var value2 = 1.0 + value;
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

        final var ellipse = new Ellipse(center, radius, radius, 0.0);

        // center is not locus
        assertFalse(ellipse.isLocus(center));
        assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));

        final var inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        final var outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        final var zero = new InhomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta));

        // check correctness
        assertFalse(ellipse.isLocus(inside));
        assertFalse(ellipse.isLocus(inside, ABSOLUTE_ERROR));

        assertFalse(ellipse.isLocus(outside));
        assertFalse(ellipse.isLocus(outside, ABSOLUTE_ERROR));

        // for point at locus of circle, distance is zero

        // zero is locus
        assertTrue(ellipse.isLocus(zero, ABSOLUTE_ERROR));
        assertTrue(ellipse.isLocus(zero, radius));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ellipse.isLocus(zero, -radius));
    }

    @Test
    void testGetTangentLineAt() throws NotLocusException {
        final var randomizer = new UniformRandomizer();
        final var center = new HomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        final var radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        // angle corresponding to line slope
        final double theta2;
        if (theta > Math.PI / 2.0) {
            theta2 = theta - Math.PI;
        } else if (theta < -Math.PI / 2.0) {
            theta2 = theta + Math.PI;
        } else {
            theta2 = theta;
        }

        final var ellipse = new Ellipse(center, radius, radius, 0.0);

        final var point = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0);
        point.normalize();

        assertTrue(ellipse.isLocus(point, ABSOLUTE_ERROR));

        // find tangent line at locus point
        final var line = ellipse.getTangentLineAt(point, ABSOLUTE_ERROR);
        final var line2 = new Line2D();
        ellipse.tangentLineAt(point, line2, ABSOLUTE_ERROR);

        assertEquals(line, line2);

        // check that point is also at line's locus
        assertTrue(line.isLocus(point));

        // check that line angle is equal to theta
        final var lineAngle = line.getAngle();
        var theta3 = theta2 - Math.PI / 2.0;
        if (theta3 < -Math.PI / 2.0) {
            theta3 += Math.PI;
        } else if (theta3 > Math.PI / 2.0) {
            theta3 -= Math.PI;
        }
        assertEquals(Math.toDegrees(lineAngle), Math.toDegrees(theta3), ABSOLUTE_ERROR);

        // Force NotLocusException
        final var point2 = new HomogeneousPoint2D(
                center.getInhomX() + 2.0 * radius * Math.cos(theta),
                center.getInhomY() + 2.0 * radius * Math.sin(theta), 1.0);
        assertThrows(NotLocusException.class, () -> ellipse.getTangentLineAt(point2));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE);
        final var semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, semiMajorAxis);
        final var rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        final var ellipse1 = new Ellipse(center, semiMajorAxis, semiMinorAxis, rotationAngle);

        // check
        assertSame(center, ellipse1.getCenter());
        assertEquals(Math.max(semiMajorAxis, semiMinorAxis), ellipse1.getSemiMajorAxis(), 0.0);
        assertEquals(Math.min(semiMajorAxis, semiMinorAxis), ellipse1.getSemiMinorAxis(), 0.0);
        assertEquals(rotationAngle, ellipse1.getRotationAngle(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(ellipse1);
        final var ellipse2 = SerializationHelper.<Ellipse>deserialize(bytes);

        // check
        assertNotSame(ellipse1, ellipse2);
        assertEquals(ellipse1.getCenter(), ellipse2.getCenter());
        assertNotSame(ellipse1.getCenter(), ellipse2.getCenter());
        assertEquals(ellipse1.getSemiMajorAxis(), ellipse2.getSemiMajorAxis(), 0.0);
        assertEquals(ellipse1.getSemiMinorAxis(), ellipse2.getSemiMinorAxis(), 0.0);
        assertEquals(ellipse1.getRotationAngle(), ellipse2.getRotationAngle(), 0.0);
    }
}
