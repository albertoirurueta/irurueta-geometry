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
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MetricTransformation3DTest {

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
    public void testConstructor() throws RotationException, CoincidentPointsException {
        // Test empty constructor
        MetricTransformation3D transformation = new MetricTransformation3D();

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(MetricTransformation2D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // Test constructor with rotation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        transformation = new MetricTransformation3D(rotation);

        // check correctness
        double sign = Math.signum(
                transformation.getRotation().getRotationAngle() * theta);
        assertEquals(theta * sign, transformation.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[0] * sign, transformation.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[1] * sign, transformation.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[2] * sign, transformation.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(MetricTransformation2D.DEFAULT_SCALE,
                transformation.getScale(), 0.0);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation3D((Rotation3D) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);

        // Test constructor with translation
        double[] translation =
                new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new MetricTransformation3D(translation);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(MetricTransformation2D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation3D((double[]) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation = new MetricTransformation3D(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with scale
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        transformation = new MetricTransformation3D(scale);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
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
        assertEquals(theta * sign, transformation.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[0] * sign, transformation.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[1] * sign, transformation.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[2] * sign, transformation.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(scale, transformation.getScale(), 0.0);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation3D(null, translation, scale);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            transformation = new MetricTransformation3D(rotation, null, scale);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);

        // Force IllegalArgumentException
        try {
            transformation = new MetricTransformation3D(rotation, badTranslation, scale);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // test constructor with corresponding points
        final double roll = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                        MAX_ANGLE_DEGREES2));
        final double pitch = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                        MAX_ANGLE_DEGREES2));
        final double yaw = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                        MAX_ANGLE_DEGREES2));

        final Quaternion q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        translation =
                new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
        scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);

        final InhomogeneousPoint3D inputPoint1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint3D inputPoint2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint3D inputPoint3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint3D inputPoint4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        transformation = new MetricTransformation3D(q, translation,
                scale);

        final Point3D outputPoint1 = transformation.transformAndReturnNew(inputPoint1);
        final Point3D outputPoint2 = transformation.transformAndReturnNew(inputPoint2);
        final Point3D outputPoint3 = transformation.transformAndReturnNew(inputPoint3);
        final Point3D outputPoint4 = transformation.transformAndReturnNew(inputPoint4);

        final MetricTransformation3D transformation2 =
                new MetricTransformation3D(inputPoint1, inputPoint2,
                        inputPoint3, inputPoint4, outputPoint1,
                        outputPoint2, outputPoint3, outputPoint4);

        final Quaternion q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();

        final double[] translation2 = transformation2.getTranslation();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, transformation2.getScale(), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetRotation() throws RotationException {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize rotation axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        // test default values
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        final double sign = Math.signum(
                transformation.getRotation().getRotationAngle() * theta);
        assertEquals(theta * sign, transformation.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[0] * sign, transformation.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[1] * sign, transformation.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis[2] * sign, transformation.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);

        // Force NullPointerException
        try {
            transformation.setRotation(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testAddRotation() throws RotationException {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double[] rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        final double[] rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

        final Rotation3D rotation1 = Rotation3D.create(rotAxis1, theta1);
        final Rotation3D rotation2 = Rotation3D.create(rotAxis2, theta2);

        final Rotation3D combinedRotation = rotation1.combineAndReturnNew(rotation2);

        // set rotation1
        transformation.setRotation(rotation1);

        // check correctness
        final double sign = Math.signum(
                transformation.getRotation().getRotationAngle() * theta1);
        assertEquals(theta1 * sign, transformation.getRotation().getRotationAngle(),
                ABSOLUTE_ERROR);
        assertEquals(rotAxis1[0] * sign, transformation.getRotation().getRotationAxis()[0],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis1[1] * sign, transformation.getRotation().getRotationAxis()[1],
                ABSOLUTE_ERROR);
        assertEquals(rotAxis1[2] * sign, transformation.getRotation().getRotationAxis()[2],
                ABSOLUTE_ERROR);

        // add second rotation
        transformation.addRotation(rotation2);

        // check correctness
        assertEquals(combinedRotation.getRotationAngle(),
                transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(combinedRotation.getRotationAxis()[0],
                transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(combinedRotation.getRotationAxis()[1],
                transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(combinedRotation.getRotationAxis()[2],
                transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetTranslation() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];

        try {
            transformation.setTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAddTranslation() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation1 = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double[] translation2 = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final double[] translationCopy = Arrays.copyOf(translation1,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0],
                transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1],
                transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2],
                transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0],
                transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1],
                transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2],
                transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation.addTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTranslationX() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);

        // set new value
        transformation.setTranslationX(translationX);

        // check correctness
        assertEquals(translationX, transformation.getTranslationX(), 0.0);
    }

    @Test
    public void testGetSetTranslationY() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set new value
        transformation.setTranslationY(translationY);

        // check correctness
        assertEquals(translationY, transformation.getTranslationY(), 0.0);
    }


    @Test
    public void testGetSetTranslationZ() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set new value
        transformation.setTranslationZ(translationZ);

        // check correctness
        assertEquals(translationZ, transformation.getTranslationZ(), 0.0);
    }

    @Test
    public void testSetTranslationCoordinates() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY, translationZ);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetTranslationPoint() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D translation = new InhomogeneousPoint3D(
                translationX, translationY, translationZ);

        // check default value
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS,
                transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        final Point3D translation2 = transformation.getTranslationPoint();
        final Point3D translation3 = Point3D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    public void testAddTranslationX() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
    public void testAddTranslationY() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

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
    public void testAddTranslationZ() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
    public void testAddTranslationCoordinates() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
    public void testAddTranslationPoint() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1, translationZ1);

        // add translation
        final Point3D translation2 = new InhomogeneousPoint3D(translationX2, translationY2, translationZ2);
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    public void testGetSetScale() {
        final MetricTransformation3D transformation = new MetricTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(MetricTransformation3D.DEFAULT_SCALE, transformation.getScale(), 0.0);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(scale, transformation.getScale(), 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Matrix m = Matrix.identity(4, 4);
        m.setSubmatrix(0, 0, 2, 2,
                rotation.asInhomogeneousMatrix().multiplyByScalarAndReturnNew(
                        scale));
        m.setSubmatrix(0, 3, 2, 3, translation);

        final Matrix transMatrix1 = transformation.asMatrix();
        final Matrix transMatrix2 = new Matrix(MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        final Matrix t = new Matrix(MetricTransformation3D.HOM_COORDS + 1,
                MetricTransformation3D.HOM_COORDS);
        try {
            transformation.asMatrix(t);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testTransformPoint() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point3D point = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point3D expectedPoint = Point3D.create();
        transformPoint(point, expectedPoint, rotation, translation, scale);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Point3D outPoint1 = transformation.transformAndReturnNew(point);
        final Point3D outPoint2 = Point3D.create();
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
    public void testTransformPoints() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final List<Point3D> outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        final List<Point3D> outPoints2 = new ArrayList<>();
        transformation.transformPoints(inputPoints, outPoints2);

        // check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPoints.get(i);

            final Point3D outPoint1 = outPoints1.get(i);
            final Point3D outPoint2 = outPoints2.get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformAndOverwritePoints() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        transformation.transformAndOverwritePoints(inputPoints);

        // check correctness
        assertEquals(inputPoints.size(), size);
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPoints.get(i);

            final Point3D point = inputPoints.get(i);

            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformQuadric() throws NonSymmetricMatrixException,
            AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create input conic
        // Constructor with params
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quadric quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final Quadric expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // make transformation
        final Quadric outQuadric1 = transformation.transformAndReturnNew(quadric);
        final Quadric outQuadric2 = new Quadric();
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
    public void testTransformQuadricAndPoints() throws AlgebraException,
            GeometryException {

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
            final Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
                quadric = new Quadric(point1, point2, point3, point4, point5, point6,
                        point7, point8, point9);
            } catch (final GeometryException ignore) {
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
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final Quadric expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // transform quadric and points
        final Quadric outQuadric = transformation.transformAndReturnNew(quadric);
        final Point3D outPoint1 = transformation.transformAndReturnNew(point1);
        final Point3D outPoint2 = transformation.transformAndReturnNew(point2);
        final Point3D outPoint3 = transformation.transformAndReturnNew(point3);
        final Point3D outPoint4 = transformation.transformAndReturnNew(point4);
        final Point3D outPoint5 = transformation.transformAndReturnNew(point5);
        final Point3D outPoint6 = transformation.transformAndReturnNew(point6);
        final Point3D outPoint7 = transformation.transformAndReturnNew(point7);
        final Point3D outPoint8 = transformation.transformAndReturnNew(point8);
        final Point3D outPoint9 = transformation.transformAndReturnNew(point9);

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
    public void tesTransformDualQuadric() throws NonSymmetricMatrixException,
            AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create input conic
        // Constructor with params
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final DualQuadric dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final DualQuadric expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // make transformation
        final DualQuadric outDualQuadric1 = transformation.transformAndReturnNew(dualQuadric);
        final DualQuadric outDualQuadric2 = new DualQuadric();
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
    public void testTransformDualQuadricAndPlanes() throws AlgebraException,
            GeometryException {

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
            final Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
                dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5,
                        plane6, plane7, plane8, plane9);
            } catch (final GeometryException ignore) {
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
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        // compute expected value
        final DualQuadric expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // transform dual quadric and planes
        final DualQuadric outDualQuadric = transformation.transformAndReturnNew(dualQuadric);
        final Plane outPlane1 = transformation.transformAndReturnNew(plane1);
        final Plane outPlane2 = transformation.transformAndReturnNew(plane2);
        final Plane outPlane3 = transformation.transformAndReturnNew(plane3);
        final Plane outPlane4 = transformation.transformAndReturnNew(plane4);
        final Plane outPlane5 = transformation.transformAndReturnNew(plane5);
        final Plane outPlane6 = transformation.transformAndReturnNew(plane6);
        final Plane outPlane7 = transformation.transformAndReturnNew(plane7);
        final Plane outPlane8 = transformation.transformAndReturnNew(plane8);
        final Plane outPlane9 = transformation.transformAndReturnNew(plane9);

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
    public void testTransformPlane() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Plane plane = new Plane(params);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Plane expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        final Plane outPlane1 = transformation.transformAndReturnNew(plane);
        final Plane outPlane2 = new Plane();
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
    public void testTransformPlaneAndPoints() throws AlgebraException,
            GeometryException {

        // create plane from 3 points
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 3 non-linear dependent rows
        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final Point3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        final Point3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        final Point3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        final Plane plane = new Plane(point1, point2, point3);

        // check that points belong to the plane
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Plane expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        // transform plane and points
        final Plane outPlane = transformation.transformAndReturnNew(plane);
        final Point3D outPoint1 = transformation.transformAndReturnNew(point1);
        final Point3D outPoint2 = transformation.transformAndReturnNew(point1);
        final Point3D outPoint3 = transformation.transformAndReturnNew(point1);

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
    public void testTransformPlanes() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final ArrayList<Plane> inputPlanes = new ArrayList<>(size);
        final ArrayList<Plane> expectedPlanes = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Plane plane = new Plane(params);
            inputPlanes.add(plane);

            final Plane expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        final List<Plane> outPlanes1 = transformation.transformPlanesAndReturnNew(inputPlanes);
        final List<Plane> outPlanes2 = new ArrayList<>();
        transformation.transformPlanes(inputPlanes, outPlanes2);

        // check correctness
        assertEquals(outPlanes1.size(), inputPlanes.size());
        assertEquals(outPlanes2.size(), inputPlanes.size());
        for (int i = 0; i < size; i++) {
            final Plane expectedPlane = expectedPlanes.get(i);

            final Plane outPlane1 = outPlanes1.get(i);
            final Plane outPlane2 = outPlanes2.get(i);

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
    public void testTransformAndOverwritePlanes() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final ArrayList<Plane> inputPlanes = new ArrayList<>(size);
        final ArrayList<Plane> expectedPlanes = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Plane plane = new Plane(params);
            inputPlanes.add(plane);

            final Plane expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        transformation.transformAndOverwritePlanes(inputPlanes);

        // check correctness
        assertEquals(inputPlanes.size(), size);
        for (int i = 0; i < size; i++) {
            final Plane expectedPlane = expectedPlanes.get(i);

            final Plane plane = inputPlanes.get(i);

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
    public void testTransformLine() throws CoincidentPointsException,
            CoincidentPlanesException, AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point3D point1 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point3D point2 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final Line3D line = new Line3D(point1, point2);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Line3D expectedLine = new Line3D(point1, point2);
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final Line3D outLine1 = transformation.transformAndReturnNew(line);
        final Line3D outLine2 = new Line3D(point1, point2);
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
    public void testTransformLines() throws CoincidentPlanesException,
            CoincidentPointsException, AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final ArrayList<Line3D> inputLines = new ArrayList<>(size);
        final ArrayList<Line3D> expectedLines = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point1 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point2 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final Line3D line = new Line3D(point1, point2);
            inputLines.add(line);

            final Line3D expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }


        final List<Line3D> outLines1 = transformation.transformLines(inputLines);
        final List<Line3D> outLines2 = new ArrayList<>();
        transformation.transformLines(inputLines, outLines2);

        // check correctness
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
        for (int i = 0; i < size; i++) {
            final Line3D expectedLine = expectedLines.get(i);

            final Line3D outLine1 = outLines1.get(i);
            final Line3D outLine2 = outLines2.get(i);

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
    public void testTransformAndOverwriteLines() throws CoincidentPointsException,
            CoincidentPlanesException, AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final ArrayList<Line3D> inputLines = new ArrayList<>(size);
        final ArrayList<Line3D> expectedLines = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point1 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point2 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final Line3D line = new Line3D(point1, point2);
            inputLines.add(line);

            final Line3D expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        transformation.transformAndOverwriteLines(inputLines);

        // check correctness
        assertEquals(inputLines.size(), size);
        for (int i = 0; i < size; i++) {
            final Line3D expectedLine = expectedLines.get(i);

            final Line3D line = inputLines.get(i);

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
    public void testTransformPolygon() throws NotEnoughVerticesException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final Polygon3D inputPolygon = new Polygon3D(inputPoints);
        final Polygon3D expectedPolygon = new Polygon3D(expectedPoints);

        final Polygon3D outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        final Polygon3D outPolygon2 = new Polygon3D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);

        // check correctness
        assertEquals(outPolygon1.getVertices().size(),
                inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPolygon.getVertices().get(i);

            final Point3D outPoint1 = outPolygon1.getVertices().get(i);
            final Point3D outPoint2 = outPolygon2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputPolygon);

        // check correctness
        assertEquals(expectedPolygon.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPolygon.getVertices().get(i);

            final Point3D outPoint = outPolygon1.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformTriangle() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = Triangle2D.NUM_VERTICES;

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final Triangle3D inputTriangle = new Triangle3D(inputPoints.get(0),
                inputPoints.get(1), inputPoints.get(2));
        final Triangle3D expectedTriangle = new Triangle3D(expectedPoints.get(0),
                expectedPoints.get(1), expectedPoints.get(2));

        final Triangle3D outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        final Triangle3D outTriangle2 = new Triangle3D(
                new InhomogeneousPoint3D(inputPoints.get(0)),
                new InhomogeneousPoint3D(inputPoints.get(1)),
                new InhomogeneousPoint3D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);

        // check correctness
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedTriangle.getVertices().get(i);

            final Point3D outPoint1 = outTriangle1.getVertices().get(i);
            final Point3D outPoint2 = outTriangle2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputTriangle);

        // check correctness
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedTriangle.getVertices().get(i);

            final Point3D outPoint = inputTriangle.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformCamera() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // generate random metric 3D point
        final InhomogeneousPoint3D metricPoint = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // generate random camera
        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final PinholeCamera camera = new PinholeCamera(cameraMatrix);

        // project metric point
        final Point2D p1 = camera.project(metricPoint);

        // generate arbitrary transformation
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        // transform metric point and camera
        final Point3D affinePoint = transformation.transformAndReturnNew(metricPoint);
        final PinholeCamera affineCamera1 = transformation.transformAndReturnNew(
                camera);
        final PinholeCamera affineCamera2 = new PinholeCamera();
        transformation.transform(camera, affineCamera2);

        transformation.transform(camera);

        // project affine point with affine camera
        final Point2D p2 = affineCamera1.project(affinePoint);
        final Point2D p3 = affineCamera2.project(affinePoint);
        final Point2D p4 = camera.project(affinePoint);

        // check that all projected points p1, p2, p3, p4 are still the same
        assertTrue(p1.equals(p2, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p3, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p4, ABSOLUTE_ERROR));
    }

    @Test
    public void testTransformCameraAndPoints() throws AlgebraException,
            GeometryException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // create intrinsic parameters
            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                    MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                    MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // create rotation parameters
            final double alphaEuler = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES3, MAX_ANGLE_DEGREES3));
            final double betaEuler = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES3, MAX_ANGLE_DEGREES3));
            final double gammaEuler = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES3, MAX_ANGLE_DEGREES3));

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                    gammaEuler);

            // create camera center
            final Point3D cameraCenter = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

            // normalize camera to improve accuracy
            camera.normalize();

            // create 6 random point correspondences
            final Point3D point3D1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final Point3D point3D2 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final Point3D point3D3 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final Point3D point3D4 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final Point3D point3D5 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
            final Point3D point3D6 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

            final Point2D point2D1 = camera.project(point3D1);
            final Point2D point2D2 = camera.project(point3D2);
            final Point2D point2D3 = camera.project(point3D3);
            final Point2D point2D4 = camera.project(point3D4);
            final Point2D point2D5 = camera.project(point3D5);
            final Point2D point2D6 = camera.project(point3D6);

            // create transformation
            final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
            randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            // normalize axis
            final double norm = Utils.normF(rotAxis);
            ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

            final Rotation3D rotation2 = Rotation3D.create(rotAxis, theta);

            final double[] translation = new double[
                    MetricTransformation3D.NUM_TRANSLATION_COORDS];
            randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);

            final MetricTransformation3D transformation =
                    new MetricTransformation3D(rotation2, translation, scale);

            // transform camera and points
            final PinholeCamera outCamera = transformation.transformAndReturnNew(camera);
            final Point3D outPoint3D1 = transformation.transformAndReturnNew(point3D1);
            final Point3D outPoint3D2 = transformation.transformAndReturnNew(point3D2);
            final Point3D outPoint3D3 = transformation.transformAndReturnNew(point3D3);
            final Point3D outPoint3D4 = transformation.transformAndReturnNew(point3D4);
            final Point3D outPoint3D5 = transformation.transformAndReturnNew(point3D5);
            final Point3D outPoint3D6 = transformation.transformAndReturnNew(point3D6);

            final Point2D outPoint2D1 = outCamera.project(outPoint3D1);
            final Point2D outPoint2D2 = outCamera.project(outPoint3D2);
            final Point2D outPoint2D3 = outCamera.project(outPoint3D3);
            final Point2D outPoint2D4 = outCamera.project(outPoint3D4);
            final Point2D outPoint2D5 = outCamera.project(outPoint3D5);
            final Point2D outPoint2D6 = outCamera.project(outPoint3D6);

            outCamera.decompose();

            final Point3D outCameraCenter = transformation.transformAndReturnNew(cameraCenter);
            final Point3D outCameraCenter2 = outCamera.getCameraCenter();

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
    public void testInverse() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Transformation3D invTransformation1 =
                transformation.inverseAndReturnNew();
        final MetricTransformation3D invTransformation2 =
                new MetricTransformation3D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final double[] params = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point3D inputPoint = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

        final Point3D transfPoint = transformation.transformAndReturnNew(inputPoint);

        final Point3D invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        final Point3D invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);

        // check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));

        // try inverting original transformation
        transformation.inverse();
        final Point3D outPoint = transformation.transformAndReturnNew(transfPoint);

        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }

    @Test
    public void testToMetric() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Matrix expectedMatrix = transformation.asMatrix();

        final Matrix metricMatrix = transformation.toMetric().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testToAffine() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(rotation, translation, scale);

        final Matrix expectedMatrix = transformation.asMatrix();

        final Matrix metricMatrix = transformation.toAffine().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testCombine() throws WrongSizeException, RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double[] rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        double norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        final double[] rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

        final Rotation3D rotation1 = Rotation3D.create(rotAxis1, theta1);
        final Rotation3D rotation2 = Rotation3D.create(rotAxis2, theta2);

        final double[] translation1 = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double[] translation2 = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double scale1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scale2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        MetricTransformation3D transformation1 =
                new MetricTransformation3D(rotation1, translation1, scale1);
        final MetricTransformation3D transformation2 =
                new MetricTransformation3D(rotation2, translation2, scale2);

        Matrix expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                transformation2.asMatrix());

        Rotation3D expectedRotation = rotation1.combineAndReturnNew(rotation2);
        Matrix rotM1 = rotation1.asInhomogeneousMatrix();
        Matrix t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        rotM1.multiplyByScalar(scale1);
        double[] expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
        double expectedScale = scale1 * scale2;

        // combine and return result as a new transformation
        MetricTransformation3D transformation3 =
                transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        Matrix m3 = transformation3.asMatrix();
        Matrix m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check also correctness of rotation and translation
        assertEquals(expectedRotation.getRotationAngle(),
                transformation1.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0],
                transformation1.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1],
                transformation1.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2],
                transformation1.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        assertEquals(expectedRotation.getRotationAngle(),
                transformation3.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0],
                transformation3.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1],
                transformation3.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2],
                transformation3.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        assertArrayEquals(expectedTranslation, transformation1.getTranslation(), ABSOLUTE_ERROR);
        assertArrayEquals(expectedTranslation, transformation3.getTranslation(), ABSOLUTE_ERROR);

        assertEquals(expectedScale, transformation1.getScale(), ABSOLUTE_ERROR);
        assertEquals(expectedScale, transformation3.getScale(), ABSOLUTE_ERROR);

        // now try combining with Euclidean transformations
        transformation1 = new MetricTransformation3D(rotation1, translation1, scale1);
        final EuclideanTransformation3D euclideanTransformation =
                new EuclideanTransformation3D(rotation2, translation2);

        expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                euclideanTransformation.asMatrix());
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
        transformation3 =
                transformation1.combineAndReturnNew(euclideanTransformation);
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
        assertEquals(expectedRotation.getRotationAngle(),
                transformation1.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0],
                transformation1.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1],
                transformation1.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2],
                transformation1.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        assertEquals(expectedRotation.getRotationAngle(),
                transformation3.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[0],
                transformation3.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[1],
                transformation3.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(expectedRotation.getRotationAxis()[2],
                transformation3.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        assertArrayEquals(expectedTranslation, transformation1.getTranslation(), ABSOLUTE_ERROR);
        assertArrayEquals(expectedTranslation, transformation3.getTranslation(), ABSOLUTE_ERROR);

        assertEquals(expectedScale, transformation1.getScale(), ABSOLUTE_ERROR);
        assertEquals(expectedScale, transformation3.getScale(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSetTransformationFromPoints()
            throws CoincidentPointsException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final double pitch = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final double yaw = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final Quaternion q = new Quaternion(roll, pitch, yaw);
        q.normalize();

        final double[] translation =
                new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
        final double scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);

        final MetricTransformation3D transformation =
                new MetricTransformation3D(q, translation, scale);

        // test constructor with corresponding points
        final InhomogeneousPoint3D inputPoint1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint3D inputPoint2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint3D inputPoint3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint3D inputPoint4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Point3D outputPoint1 = transformation.transformAndReturnNew(inputPoint1);
        final Point3D outputPoint2 = transformation.transformAndReturnNew(inputPoint2);
        final Point3D outputPoint3 = transformation.transformAndReturnNew(inputPoint3);
        final Point3D outputPoint4 = transformation.transformAndReturnNew(inputPoint4);

        final MetricTransformation3D transformation2 = new MetricTransformation3D();
        transformation2.setTransformationFromPoints(inputPoint1,
                inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                outputPoint2, outputPoint3, outputPoint4);

        final Quaternion q2 = transformation2.getRotation().toQuaternion();
        q2.normalize();

        final double[] translation2 = transformation2.getTranslation();

        assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(scale, transformation2.getScale(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation =
                new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final MetricTransformation3D transformation1 = new MetricTransformation3D(
                rotation, translation, scale);

        // check
        assertSame(rotation, transformation1.getRotation());
        assertSame(translation, transformation1.getTranslation());
        assertEquals(scale, transformation1.getScale(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(transformation1);
        final MetricTransformation3D transformation2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertEquals(transformation1.getRotation(), transformation2.getRotation());
        assertNotSame(transformation1.getRotation(), transformation2.getRotation());
        assertArrayEquals(transformation1.getTranslation(), transformation2.getTranslation(), 0.0);
        assertNotSame(transformation1.getTranslation(), transformation2.getTranslation());
        assertEquals(transformation1.getScale(), transformation2.getScale(), 0.0);
    }

    private static void transformPoint(
            final Point3D inputPoint, final Point3D outputPoint,
            final Rotation3D rotation, final double[] translation, final double scale) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(
                scale * outputPoint.getInhomX() + translation[0],
                scale * outputPoint.getInhomY() + translation[1],
                scale * outputPoint.getInhomZ() + translation[2]);
    }

    private static void transformPlane(
            final Plane inputPlane, final Plane outputPlane,
            final MetricTransformation3D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        inputPlane.normalize();
        final Matrix t = transformation.asMatrix();
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final Matrix transInvT = invT.transposeAndReturnNew();
        final Matrix p = Matrix.newFromArray(inputPlane.asArray(), true);

        outputPlane.setParameters(transInvT.multiplyAndReturnNew(p).toArray());
    }

    private static void transformQuadric(
            final Quadric inputQuadric,
            final Quadric outputQuadric,
            final MetricTransformation3D transformation)
            throws AlgebraException, NonSymmetricMatrixException {

        final Matrix t = transformation.asMatrix();
        final Matrix invT = Utils.inverse(t);
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final Matrix transInvT = invT.transposeAndReturnNew();

        inputQuadric.normalize();
        final Matrix q = inputQuadric.asMatrix();

        final Matrix transQ = transInvT.multiplyAndReturnNew(q.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transQ);
        transQ.multiplyByScalar(1.0 / norm);

        outputQuadric.setParameters(transQ);
    }

    private static void transformDualQuadric(
            final DualQuadric inputDualQuadric,
            final DualQuadric outputDualQuadric,
            final MetricTransformation3D transformation) throws WrongSizeException,
            NonSymmetricMatrixException {

        final Matrix t = transformation.asMatrix();
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix transT = t.transposeAndReturnNew();

        inputDualQuadric.normalize();
        final Matrix dualQ = inputDualQuadric.asMatrix();

        final Matrix transDualQ = t.multiplyAndReturnNew(
                dualQ.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualQ);
        transDualQ.multiplyByScalar(1.0 / norm);

        outputDualQuadric.setParameters(transDualQ);
    }

    private static void transformLine(
            final Line3D inputLine, final Line3D outputLine,
            final MetricTransformation3D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException,
            CoincidentPlanesException {

        inputLine.normalize();
        final Plane inputPlane1 = inputLine.getPlane1();
        final Plane inputPlane2 = inputLine.getPlane2();

        final Plane outputPlane1 = new Plane();
        final Plane outputPlane2 = new Plane();

        transformPlane(inputPlane1, outputPlane1, transformation);
        transformPlane(inputPlane2, outputPlane2, transformation);

        outputLine.setPlanes(outputPlane1, outputPlane2);
    }
}
