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

class MetricTransformation3DTest {

    private static final int PINHOLE_CAMERA_ROWS = 3;
    private static final int PINHOLE_CAMERA_COLS = 4;

    private static final int HOM_COORDS = 4;

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

    private static final double MIN_SCALE2 = 0.5;
    private static final double MAX_SCALE2 = 2.0;

    private static final double MIN_RANDOM_POINT_VALUE = 50.0;
    private static final double MAX_RANDOM_POINT_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES3 = 10.0;
    private static final double MAX_ANGLE_DEGREES3 = 15.0;

    private static final int TIMES = 50;

    @Test
    void testConstructor() throws RotationException, CoincidentPointsException {
        // Test empty constructor
        var transformation = new MetricTransformation3D();

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(MetricTransformation2D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // Test constructor with rotation
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        transformation = new MetricTransformation3D(rotation);

        // check correctness
        var sign = Math.signum(transformation.getRotation().getRotationAngle() * theta);
        assertEquals(theta * sign, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(rotAxis[0] * sign, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(rotAxis[1] * sign, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(rotAxis[2] * sign, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(MetricTransformation2D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new MetricTransformation3D((Rotation3D) null));

        // Test constructor with translation
        var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new MetricTransformation3D(translation);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(MetricTransformation2D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new MetricTransformation3D((double[]) null));

        // Force IllegalArgumentException
        final var badTranslation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new MetricTransformation3D(badTranslation));

        // Test constructor with scale
        var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        transformation = new MetricTransformation3D(scale);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getScale(), 0.0);

        // Test constructor with rotation and translation
        transformation = new MetricTransformation3D(rotation, translation, scale);

        // check correctness
        sign = Math.signum(transformation.getRotation().getRotationAngle() * theta);
        assertEquals(theta * sign, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(rotAxis[0] * sign, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(rotAxis[1] * sign, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(rotAxis[2] * sign, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(scale, transformation.getScale(), 0.0);

        // Force NullPointerException
        final var finalTranslation = translation;
        final var finalScale = scale;
        assertThrows(NullPointerException.class,
                () -> new MetricTransformation3D(null, finalTranslation, finalScale));
        assertThrows(NullPointerException.class,
                () -> new MetricTransformation3D(rotation, null, finalScale));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MetricTransformation3D(rotation, badTranslation, finalScale));

        // test constructor with corresponding points
        final var roll = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var pitch = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var yaw = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final var q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
        scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);

        final var inputPoint1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        transformation = new MetricTransformation3D(q, translation, scale);

        final var outputPoint1 = transformation.transformAndReturnNew(inputPoint1);
        final var outputPoint2 = transformation.transformAndReturnNew(inputPoint2);
        final var outputPoint3 = transformation.transformAndReturnNew(inputPoint3);
        final var outputPoint4 = transformation.transformAndReturnNew(inputPoint4);

        final var transformation2 = new MetricTransformation3D(inputPoint1, inputPoint2, inputPoint3, inputPoint4,
                outputPoint1, outputPoint2, outputPoint3, outputPoint4);

        final var q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();

        final var translation2 = transformation2.getTranslation();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, transformation2.getScale(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetRotation() throws RotationException {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize rotation axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        // test default values
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        final var sign = Math.signum(transformation.getRotation().getRotationAngle() * theta);
        assertEquals(theta * sign, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(rotAxis[0] * sign, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(rotAxis[1] * sign, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(rotAxis[2] * sign, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> transformation.setRotation(null));
    }

    @Test
    void testAddRotation() throws RotationException {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        final var rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

        final var rotation1 = Rotation3D.create(rotAxis1, theta1);
        final var rotation2 = Rotation3D.create(rotAxis2, theta2);

        final var combinedRotation = rotation1.combineAndReturnNew(rotation2);

        // set rotation1
        transformation.setRotation(rotation1);

        // check correctness
        final var sign = Math.signum(transformation.getRotation().getRotationAngle() * theta1);
        assertEquals(theta1 * sign, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(rotAxis1[0] * sign, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(rotAxis1[1] * sign, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(rotAxis1[2] * sign, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        // add second rotation
        transformation.addRotation(rotation2);

        // check correctness
        assertEquals(combinedRotation.getRotationAngle(), transformation.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(combinedRotation.getRotationAxis()[0], transformation.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(combinedRotation.getRotationAxis()[1], transformation.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(combinedRotation.getRotationAxis()[2], transformation.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslation() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.setTranslation(badTranslation));
    }

    @Test
    void testAddTranslation() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, MetricTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.addTranslation(badTranslation));
    }

    @Test
    void testGetSetTranslationX() {
        final var transformation = new MetricTransformation3D();

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
        final var transformation = new MetricTransformation3D();

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
    void testGetSetTranslationZ() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set new value
        transformation.setTranslationZ(translationZ);

        // check correctness
        assertEquals(translationZ, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testSetTranslationCoordinates() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY, translationZ);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationPoint() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation = new InhomogeneousPoint3D(translationX, translationY, translationZ);

        // check default value
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        final var translation2 = transformation.getTranslationPoint();
        final var translation3 = Point3D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    void testAddTranslationX() {
        final var transformation = new MetricTransformation3D();

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
        final var transformation = new MetricTransformation3D();

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
    void testAddTranslationZ() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set value
        transformation.setTranslationZ(translationZ1);

        // Check correctness
        assertEquals(translationZ1, transformation.getTranslationZ(), 0.0);

        // add translationZ
        transformation.addTranslationZ(translationZ2);

        // check correctness
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testAddTranslationCoordinates() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1, translationZ1);

        // add translation
        transformation.addTranslation(translationX2, translationY2, translationZ2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testAddTranslationPoint() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1, translationZ1);

        // add translation
        final var translation2 = new InhomogeneousPoint3D(translationX2, translationY2, translationZ2);
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testGetSetScale() {
        final var transformation = new MetricTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(scale, transformation.getScale(), 0.0);
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var m = Matrix.identity(4, 4);
        m.setSubmatrix(0, 0, 2, 2,
                rotation.asInhomogeneousMatrix().multiplyByScalarAndReturnNew(scale));
        m.setSubmatrix(0, 3, 2, 3, translation);

        final var transMatrix1 = transformation.asMatrix();
        final var transMatrix2 = new Matrix(MetricTransformation3D.HOM_COORDS, MetricTransformation3D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        final var t = new Matrix(MetricTransformation3D.HOM_COORDS + 1, MetricTransformation3D.HOM_COORDS);
        assertThrows(IllegalArgumentException.class, () -> transformation.asMatrix(t));
    }

    @Test
    void testTransformPoint() {
        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var expectedPoint = Point3D.create();
        transformPoint(point, expectedPoint, rotation, translation, scale);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var outPoint1 = transformation.transformAndReturnNew(point);
        final var outPoint2 = Point3D.create();
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
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        final var outPoints2 = new ArrayList<Point3D>();
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
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

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
    void testTransformQuadric() throws NonSymmetricMatrixException, AlgebraException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final var expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // make transformation
        final var outQuadric1 = transformation.transformAndReturnNew(quadric);
        final var outQuadric2 = new Quadric();
        transformation.transform(quadric, outQuadric2);

        // check correctness
        outQuadric1.normalize();
        outQuadric2.normalize();

        assertEquals(expectedQuadric.getA(), outQuadric1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric1.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric1.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric1.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric1.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric1.getJ(), ABSOLUTE_ERROR);

        assertEquals(expectedQuadric.getA(), outQuadric2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric2.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric2.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric2.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric2.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric2.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric2.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric2.getJ(), ABSOLUTE_ERROR);

        transformation.transform(quadric);

        // check correctness
        quadric.normalize();

        assertEquals(expectedQuadric.getA(), quadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), quadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), quadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), quadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), quadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), quadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), quadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), quadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), quadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), quadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformQuadricAndPoints() throws AlgebraException, GeometryException {

        // create Quadric from 9 points
        Quadric quadric = null;
        Point3D point1;
        Point3D point2;
        Point3D point3;
        Point3D point4;
        Point3D point5;
        Point3D point6;
        Point3D point7;
        Point3D point8;
        Point3D point9;
        do {
            final var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1), m.getElementAt(0, 2), 1.0);
            point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1), m.getElementAt(1, 2), 1.0);
            point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1), m.getElementAt(2, 2), 1.0);
            point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1), m.getElementAt(3, 2), 1.0);
            point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1), m.getElementAt(4, 2), 1.0);
            point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                    m.getElementAt(5, 1), m.getElementAt(5, 2), 1.0);
            point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                    m.getElementAt(6, 1), m.getElementAt(6, 2), 1.0);
            point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                    m.getElementAt(7, 1), m.getElementAt(7, 2), 1.0);
            point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                    m.getElementAt(8, 1), m.getElementAt(8, 2), 1.0);

            try {
                quadric = new Quadric(point1, point2, point3, point4, point5, point6, point7, point8, point9);
            } catch (final GeometryException ignore) {
                // if points are not valid, ignore and continue
            }

        } while (quadric == null);

        // check that points belong to quadric
        assertTrue(quadric.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point5, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point6, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point7, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point8, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point9, ABSOLUTE_ERROR));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final var expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // transform quadric and points
        final var outQuadric = transformation.transformAndReturnNew(quadric);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point2);
        final var outPoint3 = transformation.transformAndReturnNew(point3);
        final var outPoint4 = transformation.transformAndReturnNew(point4);
        final var outPoint5 = transformation.transformAndReturnNew(point5);
        final var outPoint6 = transformation.transformAndReturnNew(point6);
        final var outPoint7 = transformation.transformAndReturnNew(point7);
        final var outPoint8 = transformation.transformAndReturnNew(point8);
        final var outPoint9 = transformation.transformAndReturnNew(point9);

        // check that transformed points still belong to transformed quadric
        assertTrue(outQuadric.isLocus(outPoint1, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint2, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint3, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint4, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint5, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint6, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint7, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint8, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint9, ABSOLUTE_ERROR));

        // check quadric correctness
        outQuadric.normalize();

        assertEquals(expectedQuadric.getA(), outQuadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void tesTransformDualQuadric() throws NonSymmetricMatrixException, AlgebraException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final var expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // make transformation
        final var outDualQuadric1 = transformation.transformAndReturnNew(dualQuadric);
        final var outDualQuadric2 = new DualQuadric();
        transformation.transform(dualQuadric, outDualQuadric2);

        // check correctness
        outDualQuadric1.normalize();
        outDualQuadric2.normalize();

        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(), ABSOLUTE_ERROR);

        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(), ABSOLUTE_ERROR);

        transformation.transform(dualQuadric);

        // check correctness
        dualQuadric.normalize();

        assertEquals(expectedDualQuadric.getA(), dualQuadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), dualQuadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), dualQuadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), dualQuadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), dualQuadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), dualQuadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), dualQuadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), dualQuadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), dualQuadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), dualQuadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformDualQuadricAndPlanes() throws AlgebraException, GeometryException {

        // create dual quadric from 9 planes
        DualQuadric dualQuadric = null;
        Plane plane1;
        Plane plane2;
        Plane plane3;
        Plane plane4;
        Plane plane5;
        Plane plane6;
        Plane plane7;
        Plane plane8;
        Plane plane9;
        do {
            final var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), m.getElementAt(0, 3));
            plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), m.getElementAt(1, 3));
            plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), m.getElementAt(2, 3));
            plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), m.getElementAt(3, 3));
            plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), m.getElementAt(4, 3));
            plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), m.getElementAt(5, 3));
            plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), m.getElementAt(6, 3));
            plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), m.getElementAt(7, 3));
            plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), m.getElementAt(8, 3));

            plane1.normalize();
            plane2.normalize();
            plane3.normalize();
            plane4.normalize();
            plane5.normalize();
            plane6.normalize();
            plane7.normalize();
            plane8.normalize();
            plane9.normalize();

            try {
                dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5, plane6, plane7, plane8, plane9);
            } catch (final GeometryException ignore) {
                // if planes are not valid, ignore and continue
            }

        } while (dualQuadric == null);

        // check that planes belong to dual quadric
        assertTrue(dualQuadric.isLocus(plane1, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane2, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane3, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane4, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane5, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane6, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane7, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane8, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane9, ABSOLUTE_ERROR));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final var expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // transform dual quadric and planes
        final var outDualQuadric = transformation.transformAndReturnNew(dualQuadric);
        final var outPlane1 = transformation.transformAndReturnNew(plane1);
        final var outPlane2 = transformation.transformAndReturnNew(plane2);
        final var outPlane3 = transformation.transformAndReturnNew(plane3);
        final var outPlane4 = transformation.transformAndReturnNew(plane4);
        final var outPlane5 = transformation.transformAndReturnNew(plane5);
        final var outPlane6 = transformation.transformAndReturnNew(plane6);
        final var outPlane7 = transformation.transformAndReturnNew(plane7);
        final var outPlane8 = transformation.transformAndReturnNew(plane8);
        final var outPlane9 = transformation.transformAndReturnNew(plane9);

        // check that transformed planes still belong to transformed dual quadric
        assertTrue(outDualQuadric.isLocus(outPlane1, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane2, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane3, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane4, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane5, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane6, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane7, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane8, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane9, ABSOLUTE_ERROR));

        // check dual quadric correctness
        outDualQuadric.normalize();

        assertEquals(expectedDualQuadric.getA(), outDualQuadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformPlane() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var params = new double[Plane.PLANE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var plane = new Plane(params);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        final var outPlane1 = transformation.transformAndReturnNew(plane);
        final var outPlane2 = new Plane();
        transformation.transform(plane, outPlane2);

        outPlane1.normalize();
        outPlane2.normalize();

        // check correctness
        assertEquals(expectedPlane.getA(), outPlane1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), outPlane1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), outPlane1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), outPlane1.getD(), ABSOLUTE_ERROR);

        assertEquals(expectedPlane.getA(), outPlane2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), outPlane2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), outPlane2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), outPlane2.getD(), ABSOLUTE_ERROR);

        transformation.transform(plane);

        plane.normalize();

        // check correctness
        assertEquals(expectedPlane.getA(), plane.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), plane.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), plane.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), plane.getD(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformPlaneAndPoints() throws AlgebraException, GeometryException {

        // create plane from 3 points
        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 3 non-linear dependent rows
        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

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

        final var plane = new Plane(point1, point2, point3);

        // check that points belong to the plane
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        // transform plane and points
        final var outPlane = transformation.transformAndReturnNew(plane);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point1);
        final var outPoint3 = transformation.transformAndReturnNew(point1);

        // check that transformed points still belong to transformed plane
        assertTrue(outPlane.isLocus(outPoint1));
        assertTrue(outPlane.isLocus(outPoint2));
        assertTrue(outPlane.isLocus(outPoint3));

        // check plane correctness
        outPlane.normalize();

        assertEquals(expectedPlane.getA(), outPlane.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), outPlane.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), outPlane.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), outPlane.getD(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformPlanes() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var inputPlanes = new ArrayList<Plane>(size);
        final var expectedPlanes = new ArrayList<Plane>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var plane = new Plane(params);
            inputPlanes.add(plane);

            final var expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        final var outPlanes1 = transformation.transformPlanesAndReturnNew(inputPlanes);
        final var outPlanes2 = new ArrayList<Plane>();
        transformation.transformPlanes(inputPlanes, outPlanes2);

        // check correctness
        assertEquals(outPlanes1.size(), inputPlanes.size());
        assertEquals(outPlanes2.size(), inputPlanes.size());
        for (var i = 0; i < size; i++) {
            final var expectedPlane = expectedPlanes.get(i);

            final var outPlane1 = outPlanes1.get(i);
            final var outPlane2 = outPlanes2.get(i);

            expectedPlane.normalize();
            outPlane1.normalize();
            outPlane2.normalize();

            // check correctness
            assertEquals(expectedPlane.getA(), outPlane1.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), outPlane1.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), outPlane1.getC(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), outPlane1.getD(), ABSOLUTE_ERROR);

            assertEquals(expectedPlane.getA(), outPlane2.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), outPlane2.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), outPlane2.getC(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), outPlane2.getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformAndOverwritePlanes() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var inputPlanes = new ArrayList<Plane>(size);
        final var expectedPlanes = new ArrayList<Plane>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var plane = new Plane(params);
            inputPlanes.add(plane);

            final var expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        transformation.transformAndOverwritePlanes(inputPlanes);

        // check correctness
        assertEquals(inputPlanes.size(), size);
        for (var i = 0; i < size; i++) {
            final var expectedPlane = expectedPlanes.get(i);

            final var plane = inputPlanes.get(i);

            expectedPlane.normalize();
            plane.normalize();

            // check correctness
            assertEquals(expectedPlane.getA(), plane.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), plane.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), plane.getC(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), plane.getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformLine() throws CoincidentPointsException, CoincidentPlanesException, AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final var line = new Line3D(point1, point2);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var expectedLine = new Line3D(point1, point2);
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final var outLine1 = transformation.transformAndReturnNew(line);
        final var outLine2 = new Line3D(point1, point2);
        transformation.transform(line, outLine2);

        outLine1.normalize();
        outLine2.normalize();

        // check correctness
        assertEquals(expectedLine.getPlane1().getA(), outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), outLine1.getPlane2().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane1().getA(), outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), outLine2.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), outLine2.getPlane2().getD(), ABSOLUTE_ERROR);

        transformation.transform(line);

        line.normalize();

        // check correctness
        assertEquals(expectedLine.getPlane1().getA(), line.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), line.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), line.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), line.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), line.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), line.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), line.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), line.getPlane2().getD(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformLines() throws CoincidentPlanesException, CoincidentPointsException, AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var inputLines = new ArrayList<Line3D>(size);
        final var expectedLines = new ArrayList<Line3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final var line = new Line3D(point1, point2);
            inputLines.add(line);

            final var expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }


        final var outLines1 = transformation.transformLines(inputLines);
        final var outLines2 = new ArrayList<Line3D>();
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
            assertEquals(expectedLine.getPlane1().getA(), outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), outLine1.getPlane2().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane1().getA(), outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), outLine2.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), outLine2.getPlane2().getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformAndOverwriteLines() throws CoincidentPointsException, CoincidentPlanesException,
            AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var inputLines = new ArrayList<Line3D>(size);
        final var expectedLines = new ArrayList<Line3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final var line = new Line3D(point1, point2);
            inputLines.add(line);

            final var expectedLine = new Line3D(point1, point2);
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
            assertEquals(expectedLine.getPlane1().getA(), line.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), line.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), line.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), line.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), line.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), line.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), line.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), line.getPlane2().getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformPolygon() throws NotEnoughVerticesException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final var inputPolygon = new Polygon3D(inputPoints);
        final var expectedPolygon = new Polygon3D(expectedPoints);

        final var outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        final var outPolygon2 = new Polygon3D(inputPoints);
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
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final var inputTriangle = new Triangle3D(inputPoints.get(0), inputPoints.get(1), inputPoints.get(2));
        final var expectedTriangle = new Triangle3D(expectedPoints.get(0), expectedPoints.get(1),
                expectedPoints.get(2));

        final var outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        final var outTriangle2 = new Triangle3D(
                new InhomogeneousPoint3D(inputPoints.get(0)),
                new InhomogeneousPoint3D(inputPoints.get(1)),
                new InhomogeneousPoint3D(inputPoints.get(2)));
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
    void testTransformCamera() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        // generate random metric 3D point
        final var metricPoint = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // generate random camera
        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var camera = new PinholeCamera(cameraMatrix);

        // project metric point
        final var p1 = camera.project(metricPoint);

        // generate arbitrary transformation
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        // transform metric point and camera
        final var affinePoint = transformation.transformAndReturnNew(metricPoint);
        final var affineCamera1 = transformation.transformAndReturnNew(camera);
        final var affineCamera2 = new PinholeCamera();
        transformation.transform(camera, affineCamera2);

        transformation.transform(camera);

        // project affine point with affine camera
        final var p2 = affineCamera1.project(affinePoint);
        final var p3 = affineCamera2.project(affinePoint);
        final var p4 = camera.project(affinePoint);

        // check that all projected points p1, p2, p3, p4 are still the same
        assertTrue(p1.equals(p2, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p3, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p4, ABSOLUTE_ERROR));
    }

    @Test
    void testTransformCameraAndPoints() throws AlgebraException, GeometryException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // create intrinsic parameters
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create rotation parameters
            final var alphaEuler = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES3, MAX_ANGLE_DEGREES3));
            final var betaEuler = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES3, MAX_ANGLE_DEGREES3));
            final var gammaEuler = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES3, MAX_ANGLE_DEGREES3));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // create camera center
            final var cameraCenter = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize camera to improve accuracy
            camera.normalize();

            // create 6 random point correspondences
            final var point3D1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final var point3D2 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final var point3D3 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final var point3D4 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final var point3D5 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final var point3D6 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

            final var point2D1 = camera.project(point3D1);
            final var point2D2 = camera.project(point3D2);
            final var point2D3 = camera.project(point3D3);
            final var point2D4 = camera.project(point3D4);
            final var point2D5 = camera.project(point3D5);
            final var point2D6 = camera.project(point3D6);

            // create transformation
            final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotAxis = new double[Rotation3D.INHOM_COORDS];
            randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            // normalize axis
            final var norm = Utils.normF(rotAxis);
            ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

            final var rotation2 = Rotation3D.create(rotAxis, theta);

            final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
            randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var transformation = new MetricTransformation3D(rotation2, translation, scale);

            // transform camera and points
            final var outCamera = transformation.transformAndReturnNew(camera);
            final var outPoint3D1 = transformation.transformAndReturnNew(point3D1);
            final var outPoint3D2 = transformation.transformAndReturnNew(point3D2);
            final var outPoint3D3 = transformation.transformAndReturnNew(point3D3);
            final var outPoint3D4 = transformation.transformAndReturnNew(point3D4);
            final var outPoint3D5 = transformation.transformAndReturnNew(point3D5);
            final var outPoint3D6 = transformation.transformAndReturnNew(point3D6);

            final var outPoint2D1 = outCamera.project(outPoint3D1);
            final var outPoint2D2 = outCamera.project(outPoint3D2);
            final var outPoint2D3 = outCamera.project(outPoint3D3);
            final var outPoint2D4 = outCamera.project(outPoint3D4);
            final var outPoint2D5 = outCamera.project(outPoint3D5);
            final var outPoint2D6 = outCamera.project(outPoint3D6);

            outCamera.decompose();

            final var outCameraCenter = transformation.transformAndReturnNew(cameraCenter);
            final var outCameraCenter2 = outCamera.getCameraCenter();

            // check that projection of transformed points on transformed camera does
            // not change projected points
            assertTrue(outPoint2D1.equals(point2D1, ABSOLUTE_ERROR));
            assertTrue(outPoint2D2.equals(point2D2, ABSOLUTE_ERROR));
            assertTrue(outPoint2D3.equals(point2D3, ABSOLUTE_ERROR));
            assertTrue(outPoint2D4.equals(point2D4, ABSOLUTE_ERROR));
            assertTrue(outPoint2D5.equals(point2D5, ABSOLUTE_ERROR));
            assertTrue(outPoint2D6.equals(point2D6, ABSOLUTE_ERROR));

            // check that camera center has been transformed
            if (!outCameraCenter.equals(outCameraCenter2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(outCameraCenter.equals(outCameraCenter2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testInverse() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var invTransformation1 = transformation.inverseAndReturnNew();
        final var invTransformation2 = new MetricTransformation3D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final var params = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoint = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

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
    void testToMetric() {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var expectedMatrix = transformation.asMatrix();

        final var metricMatrix = transformation.toMetric().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testToAffine() {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new MetricTransformation3D(rotation, translation, scale);

        final var expectedMatrix = transformation.asMatrix();

        final var metricMatrix = transformation.toAffine().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testCombine() throws WrongSizeException, RotationException {
        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        var norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        final var rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

        final var rotation1 = Rotation3D.create(rotAxis1, theta1);
        final var rotation2 = Rotation3D.create(rotAxis2, theta2);

        final var translation1 = new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var scale1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scale2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var transformation1 = new MetricTransformation3D(rotation1, translation1, scale1);
        final var transformation2 = new MetricTransformation3D(rotation2, translation2, scale2);

        var expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(transformation2.asMatrix());

        var expectedRotation = rotation1.combineAndReturnNew(rotation2);
        var rotM1 = rotation1.asInhomogeneousMatrix();
        var t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        rotM1.multiplyByScalar(scale1);
        var expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
        var expectedScale = scale1 * scale2;

        // combine and return result as a new transformation
        var transformation3 = transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        var m3 = transformation3.asMatrix();
        var m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check also correctness of rotation and translation
        assertEquals(expectedRotation.getRotationAngle(), transformation1.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0], transformation1.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1], transformation1.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2], transformation1.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);

        assertEquals(expectedRotation.getRotationAngle(), transformation3.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0], transformation3.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1], transformation3.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2], transformation3.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);

        assertArrayEquals(expectedTranslation, transformation1.getTranslation(), ABSOLUTE_ERROR);
        assertArrayEquals(expectedTranslation, transformation3.getTranslation(), ABSOLUTE_ERROR);

        assertEquals(expectedScale, transformation1.getScale(), ABSOLUTE_ERROR);
        assertEquals(expectedScale, transformation3.getScale(), ABSOLUTE_ERROR);

        // now try combining with Euclidean transformations
        transformation1 = new MetricTransformation3D(rotation1, translation1, scale1);
        final var euclideanTransformation = new EuclideanTransformation3D(rotation2, translation2);

        expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(euclideanTransformation.asMatrix());
        expectedRotation = rotation1.combineAndReturnNew(rotation2);
        rotM1 = rotation1.asInhomogeneousMatrix();
        t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        rotM1.multiplyByScalar(scale1);
        expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
        // scale2 is assumed to be 1 because
        // transformation is Euclidean
        expectedScale = scale1;

        // combine and return result as a new transformation
        transformation3 = transformation1.combineAndReturnNew(euclideanTransformation);
        // combine into transformation1
        transformation1.combine(euclideanTransformation);

        // both matrices m1 and m3 need to be equal
        m3 = transformation3.asMatrix();
        m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check also correctness of rotation and translation
        assertEquals(expectedRotation.getRotationAngle(), transformation1.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0], transformation1.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1], transformation1.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2], transformation1.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);

        assertEquals(expectedRotation.getRotationAngle(), transformation3.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0], transformation3.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1], transformation3.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2], transformation3.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);

        assertArrayEquals(expectedTranslation, transformation1.getTranslation(), ABSOLUTE_ERROR);
        assertArrayEquals(expectedTranslation, transformation3.getTranslation(), ABSOLUTE_ERROR);

        assertEquals(expectedScale, transformation1.getScale(), ABSOLUTE_ERROR);
        assertEquals(expectedScale, transformation3.getScale(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetTransformationFromPoints() throws CoincidentPointsException {
        final var randomizer = new UniformRandomizer();
        final var roll = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var pitch = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var yaw = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final var q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        final var translation = new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
        final var scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);

        final var transformation = new MetricTransformation3D(q, translation, scale);

        // test constructor with corresponding points
        final var inputPoint1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var inputPoint4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var outputPoint1 = transformation.transformAndReturnNew(inputPoint1);
        final var outputPoint2 = transformation.transformAndReturnNew(inputPoint2);
        final var outputPoint3 = transformation.transformAndReturnNew(inputPoint3);
        final var outputPoint4 = transformation.transformAndReturnNew(inputPoint4);

        final var transformation2 = new MetricTransformation3D();
        transformation2.setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                outputPoint2, outputPoint3, outputPoint4);

        final var q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();

        final var translation2 = transformation2.getTranslation();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, transformation2.getScale(), ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation1 = new MetricTransformation3D(rotation, translation, scale);

        // check
        assertSame(rotation, transformation1.getRotation());
        assertSame(translation, transformation1.getTranslation());
        assertEquals(scale, transformation1.getScale(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(transformation1);
        final var transformation2 = SerializationHelper.<MetricTransformation3D>deserialize(bytes);

        // check
        assertEquals(transformation1.getRotation(), transformation2.getRotation());
        assertNotSame(transformation1.getRotation(), transformation2.getRotation());
        assertArrayEquals(transformation1.getTranslation(), transformation2.getTranslation(), 0.0);
        assertNotSame(transformation1.getTranslation(), transformation2.getTranslation());
        assertEquals(transformation1.getScale(), transformation2.getScale(), 0.0);
    }

    private static void transformPoint(
            final Point3D inputPoint, final Point3D outputPoint, final Rotation3D rotation, final double[] translation,
            final double scale) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(
                scale * outputPoint.getInhomX() + translation[0],
                scale * outputPoint.getInhomY() + translation[1],
                scale * outputPoint.getInhomZ() + translation[2]);
    }

    private static void transformPlane(
            final Plane inputPlane, final Plane outputPlane, final MetricTransformation3D transformation)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {
        inputPlane.normalize();
        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();
        final var p = Matrix.newFromArray(inputPlane.asArray(), true);

        outputPlane.setParameters(transInvT.multiplyAndReturnNew(p).toArray());
    }

    private static void transformQuadric(
            final Quadric inputQuadric, final Quadric outputQuadric, final MetricTransformation3D transformation)
            throws AlgebraException, NonSymmetricMatrixException {

        final var t = transformation.asMatrix();
        final var invT = Utils.inverse(t);
        var norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();

        inputQuadric.normalize();
        final var q = inputQuadric.asMatrix();

        final var transQ = transInvT.multiplyAndReturnNew(q.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transQ);
        transQ.multiplyByScalar(1.0 / norm);

        outputQuadric.setParameters(transQ);
    }

    private static void transformDualQuadric(
            final DualQuadric inputDualQuadric, final DualQuadric outputDualQuadric,
            final MetricTransformation3D transformation) throws WrongSizeException, NonSymmetricMatrixException {

        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var transT = t.transposeAndReturnNew();

        inputDualQuadric.normalize();
        final var dualQ = inputDualQuadric.asMatrix();

        final var transDualQ = t.multiplyAndReturnNew(dualQ.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualQ);
        transDualQ.multiplyByScalar(1.0 / norm);

        outputDualQuadric.setParameters(transDualQ);
    }

    private static void transformLine(
            final Line3D inputLine, final Line3D outputLine, final MetricTransformation3D transformation)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException, CoincidentPlanesException {

        inputLine.normalize();
        final var inputPlane1 = inputLine.getPlane1();
        final var inputPlane2 = inputLine.getPlane2();

        final var outputPlane1 = new Plane();
        final var outputPlane2 = new Plane();

        transformPlane(inputPlane1, outputPlane1, transformation);
        transformPlane(inputPlane2, outputPlane2, transformation);

        outputLine.setPlanes(outputPlane1, outputPlane2);
    }
}
