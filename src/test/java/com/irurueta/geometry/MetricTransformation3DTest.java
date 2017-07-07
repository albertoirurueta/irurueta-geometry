/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.MetricTransformation3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 27, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class MetricTransformation3DTest {
    
    public static final int PINHOLE_CAMERA_ROWS = 3;
    public static final int PINHOLE_CAMERA_COLS = 4;
    
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final int MIN_POINTS = 3;
    public static final int MAX_POINTS = 50;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    
    public static final double MIN_ANGLE_DEGREES2 = -90.0;
    public static final double MAX_ANGLE_DEGREES2 = 90.0;
    
    public static final double MIN_TRANSLATION2 = -100.0;
    public static final double MAX_TRANSLATION2 = 100.0;
    
    public static final double MIN_SCALE2 = 0.5;
    public static final double MAX_SCALE2 = 2.0;
    
    public static final int TIMES = 50;
    
    public MetricTransformation3DTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
        
    @Test
    public void testConstructor() throws RotationException, CoincidentPointsException{
        
        //Test empty constructor
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);        

        
        //Test constructor with rotation
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        transformation = new MetricTransformation3D(rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 
                rotAxis[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 
                rotAxis[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 
                rotAxis[2], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);        
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation3D((Rotation3D)null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        assertNull(transformation);
        
        
        //Test constructor with translation
        double[] translation =
                new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        transformation = new MetricTransformation3D(translation);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);        
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation3D((double[])null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation = new MetricTransformation3D(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(transformation);
        
        
        //Test constructor with scale
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);        
        transformation = new MetricTransformation3D(scale);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getScale(), scale, 0.0);        
        
        
        //Test constructor with rotation and translation
        transformation = new MetricTransformation3D(rotation, translation, 
                scale);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 
                rotAxis[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 
                rotAxis[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 
                rotAxis[2], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getScale(), scale, 0.0);
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation3D(null, translation, 
                    scale);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        try {
            transformation = new MetricTransformation3D(rotation, null, scale);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        assertNull(transformation);
        
        //Force IllegalArgumentException
        transformation = null;
        try {
            transformation = new MetricTransformation3D(rotation, 
                    badTranslation, scale);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(transformation);
        
        
        //test constructor with corresponding points
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            double roll = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                            MAX_ANGLE_DEGREES2));
            double pitch = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                            MAX_ANGLE_DEGREES2));
            double yaw = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                            MAX_ANGLE_DEGREES2));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
            
            translation = 
                    new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
            randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
            scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);        
            
            InhomogeneousPoint3D inputPoint1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D inputPoint2 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D inputPoint3 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D inputPoint4 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
            transformation = new MetricTransformation3D(q, translation, 
                    scale);
        
            Point3D outputPoint1 = transformation.transformAndReturnNew(
                    inputPoint1);
            Point3D outputPoint2 = transformation.transformAndReturnNew(
                    inputPoint2);
            Point3D outputPoint3 = transformation.transformAndReturnNew(
                    inputPoint3);
            Point3D outputPoint4 = transformation.transformAndReturnNew(
                    inputPoint4);
        
            MetricTransformation3D transformation2 =
                    new MetricTransformation3D(inputPoint1, inputPoint2, 
                            inputPoint3, inputPoint4, outputPoint1, 
                            outputPoint2, outputPoint3, outputPoint4);
        
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(transformation2.getScale(), scale, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }
    
    @Test
    public void testGetSetRotation() throws RotationException {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize rotation axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        //test default values
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0, 
                ABSOLUTE_ERROR);
        
        //set new value
        transformation.setRotation(rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 
                rotAxis[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 
                rotAxis[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2],
                rotAxis[2], ABSOLUTE_ERROR);
        
        //Force NullPointerException
        try {
            transformation.setRotation(null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
    }
    
    @Test
    public void testAddRotation() throws RotationException {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        double[] rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);
        
        double[] rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);
        
        Rotation3D rotation1 = Rotation3D.create(rotAxis1, theta1);
        Rotation3D rotation2 = Rotation3D.create(rotAxis2, theta2);
        
        Rotation3D combinedRotation = rotation1.combineAndReturnNew(rotation2);
        
        //set rotation1
        transformation.setRotation(rotation1);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), theta1, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 
                rotAxis1[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 
                rotAxis1[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 
                rotAxis1[2], ABSOLUTE_ERROR);

        //add second rotation
        transformation.addRotation(rotation2);
        
        //check correctness
        assertEquals(transformation.getRotation().getRotationAngle(),
                combinedRotation.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0],
                combinedRotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1],
                combinedRotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2],
                combinedRotation.getRotationAxis()[2], ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetTranslation() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
        
        //set new value
        transformation.setTranslation(translation);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translation[1], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translation[2], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translation[0], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translation[1], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translation[2], 
                ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        
        try {
            transformation.setTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testAddTranslation() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation1 = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] translation2 = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        
        //set new value
        double[] translationCopy = Arrays.copyOf(translation1, 
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translation1[1], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translation1[2], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translation1[0], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translation1[1], 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translation1[2], 
                ABSOLUTE_ERROR);
        
        //add translation
        transformation.addTranslation(translation2);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0] +
                translation2[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translation1[1] +
                translation2[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translation1[2] +
                translation2[2], ABSOLUTE_ERROR);        
        assertEquals(transformation.getTranslationX(), translation1[0] +
                translation2[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translation1[1] +
                translation2[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translation1[2] +
                translation2[2], ABSOLUTE_ERROR);

        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation.addTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e){}        
    }
    
    @Test
    public void testGetSetTranslationX() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        
        //set new value
        transformation.setTranslationX(translationX);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), translationX, 0.0);
    }  
        
    @Test
    public void testGetSetTranslationY() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set new value
        transformation.setTranslationY(translationY);
        
        //check correctness
        assertEquals(transformation.getTranslationY(), translationY, 0.0);
    }
    
    
    @Test
    public void testGetSetTranslationZ() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        
        //set new value
        transformation.setTranslationZ(translationZ);
        
        //check correctness
        assertEquals(transformation.getTranslationZ(), translationZ, 0.0);
    }
    
    @Test
    public void testSetTranslationCoordinates() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
       
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
                       
        //set new value
        transformation.setTranslation(translationX, translationY, translationZ);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translationZ, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translationZ, 
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testGetSetTranslationPoint() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());        
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        InhomogeneousPoint3D translation = new InhomogeneousPoint3D(
                translationX, translationY, translationZ);
       
        //check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
                       
        //set new value
        transformation.setTranslation(translation);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translationZ, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translationZ, 
                ABSOLUTE_ERROR);   
        
        Point3D translation2 = transformation.getTranslationPoint();
        Point3D translation3 = Point3D.create();
        transformation.getTranslationPoint(translation3);
        
        //check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);        
    }
       
    @Test
    public void testAddTranslationX() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        
        //set value
        transformation.setTranslationX(translationX1);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), translationX1, 0.0);
        
        //add translation x
        transformation.addTranslationX(translationX2);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), 
                translationX1 + translationX2, 0.0);
    }
    
    @Test
    public void testAddTranslationY() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set value
        transformation.setTranslationY(translationY1);
        
        //check correctness
        assertEquals(transformation.getTranslationY(), translationY1, 0.0);
        
        //add translation y
        transformation.addTranslationY(translationY2);
        
        //check correctness
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
    }
    
    @Test
    public void testAddTranslationZ() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        
        //set value
        transformation.setTranslationZ(translationZ1);
        
        //Check correctness
        assertEquals(transformation.getTranslationZ(), translationZ1, 0.0);
        
        //add translationZ
        transformation.addTranslationZ(translationZ2);
        
        //check correctness
        assertEquals(transformation.getTranslationZ(),
                translationZ1 + translationZ2, 0.0);
    }
    
    @Test
    public void testAddTranslationCoordinates() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        
        //set values
        transformation.setTranslation(translationX1, translationY1, 
                translationZ1);
        
        //add translation
        transformation.addTranslation(translationX2, translationY2, 
                translationZ2);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), 
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
        assertEquals(transformation.getTranslationZ(),
                translationZ1 + translationZ2, 0.0);
    }
    
    @Test
    public void testAddTranslationPoint() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        
        //set values
        transformation.setTranslation(translationX1, translationY1, 
                translationZ1);
        
        //add translation
        Point3D translation2 = new InhomogeneousPoint3D(translationX2,
                translationY2, translationZ2);
        transformation.addTranslation(translation2);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), 
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
        assertEquals(transformation.getTranslationZ(),
                translationZ1 + translationZ2, 0.0);        
    }    
    
    @Test
    public void testGetSetScale() {
        MetricTransformation3D transformation = new MetricTransformation3D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getScale(), 
                MetricTransformation3D.DEFAULT_SCALE, 0.0);
        
        //set value
        transformation.setScale(scale);
        
        //check correctness
        assertEquals(transformation.getScale(), scale, 0.0);
    }    
    
    @Test
    public void testAsMatrix() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Matrix m = Matrix.identity(4, 4);
        m.setSubmatrix(0, 0, 2, 2, 
                rotation.asInhomogeneousMatrix().multiplyByScalarAndReturnNew(
                scale));
        m.setSubmatrix(0, 3, 2, 3, translation);
        
        Matrix transMatrix1 = transformation.asMatrix();
        Matrix transMatrix2 = new Matrix(MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);
        
        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));
        
        Matrix T = new Matrix(MetricTransformation3D.HOM_COORDS + 1,
                MetricTransformation3D.HOM_COORDS);
        try {
            transformation.asMatrix(T);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testTransformPoint() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Point3D point = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                        
        
        Point3D expectedPoint = Point3D.create();
        transformPoint(point, expectedPoint, rotation, translation, scale);

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Point3D outPoint1 = transformation.transformAndReturnNew(point);
        Point3D outPoint2 = Point3D.create();
        transformation.transform(point, outPoint2);
        
        //check correctness
        assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
        assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        
        //update point
        transformation.transform(point);
        
        //check correctness
        assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));        
    }
    
    @Test
    public void testTransformPoints() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                
        
        ArrayList<Point3D> inputPoints = new ArrayList<Point3D>(size);
        ArrayList<Point3D> expectedPoints = new ArrayList<Point3D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        List<Point3D> outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        List<Point3D> outPoints2 = new ArrayList<Point3D>();
        transformation.transformPoints(inputPoints, outPoints2);
                
        //check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (int i = 0; i < size; i++) {
            Point3D expectedPoint = expectedPoints.get(i);
            
            Point3D outPoint1 = outPoints1.get(i);
            Point3D outPoint2 = outPoints2.get(i);
            
            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));            
        }
    }   
    
    @Test
    public void testTransformAndOverwritePoints() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                
        
        
        ArrayList<Point3D> inputPoints = new ArrayList<Point3D>(size);
        ArrayList<Point3D> expectedPoints = new ArrayList<Point3D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        transformation.transformAndOverwritePoints(inputPoints);
                        
        //check correctness
        assertEquals(inputPoints.size(), size);
        for (int i = 0; i < size; i++) {
            Point3D expectedPoint = expectedPoints.get(i);
            
            Point3D point = inputPoints.get(i);
            
            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }        
    }
    
    @Test
    public void testTransformQuadric() throws WrongSizeException, 
            NonSymmetricMatrixException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //create input conic
        //Constructor with params
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Quadric quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);
              
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        //compute expected value
        Quadric expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();
        
        //make transformation
        Quadric outQuadric1 = transformation.transformAndReturnNew(quadric);
        Quadric outQuadric2 = new Quadric();
        transformation.transform(quadric, outQuadric2);
        
        //check correctness
        outQuadric1.normalize();
        outQuadric2.normalize();
        
        assertEquals(expectedQuadric.getA(), outQuadric1.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric1.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric1.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric1.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric1.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric1.getF(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric1.getG(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric1.getH(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric1.getI(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric1.getJ(), 
                ABSOLUTE_ERROR);
        
        assertEquals(expectedQuadric.getA(), outQuadric2.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric2.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric2.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric2.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric2.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric2.getF(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getG(), outQuadric2.getG(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getH(), outQuadric2.getH(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getI(), outQuadric2.getI(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getJ(), outQuadric2.getJ(), 
                ABSOLUTE_ERROR);    
        
        transformation.transform(quadric);
        
        //check correctness
        quadric.normalize();
        
        assertEquals(expectedQuadric.getA(), quadric.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), quadric.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), quadric.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), quadric.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), quadric.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), quadric.getF(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getG(), quadric.getG(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getH(), quadric.getH(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getI(), quadric.getI(), 
                ABSOLUTE_ERROR);        
        assertEquals(expectedQuadric.getJ(), quadric.getJ(), 
                ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void tesTransformDualQuadric() throws WrongSizeException, 
        NonSymmetricMatrixException, RankDeficientMatrixException, 
        DecomposerException, AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //create input conic
        //Constructor with params
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        
        DualQuadric dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);
              
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        //compute expected value
        DualQuadric expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();
        
        //make transformation
        DualQuadric outDualQuadric1 = transformation.transformAndReturnNew(dualQuadric);
        DualQuadric outDualQuadric2 = new DualQuadric();
        transformation.transform(dualQuadric, outDualQuadric2);
        
        //check correctness
        outDualQuadric1.normalize();
        outDualQuadric2.normalize();
        
        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(), 
                ABSOLUTE_ERROR);

        
        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(), 
                ABSOLUTE_ERROR);    
        
        transformation.transform(dualQuadric);
        
        //check correctness
        dualQuadric.normalize();
        
        assertEquals(expectedDualQuadric.getA(), dualQuadric.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), dualQuadric.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), dualQuadric.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), dualQuadric.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), dualQuadric.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), dualQuadric.getF(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), dualQuadric.getG(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), dualQuadric.getH(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), dualQuadric.getI(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), dualQuadric.getJ(), 
                ABSOLUTE_ERROR);        
    }  
    
    @Test
    public void testTransformPlane() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane = new Plane(params);
                
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Plane expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();
        
        Plane outPlane1 = transformation.transformAndReturnNew(plane);
        Plane outPlane2 = new Plane();
        transformation.transform(plane, outPlane2);
        
        outPlane1.normalize();
        outPlane2.normalize();
        
        //check correctness
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
        
        //check correctness
        assertEquals(expectedPlane.getA(), plane.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), plane.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), plane.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), plane.getD(), ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testTransformPlanes() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        ArrayList<Plane> inputPlanes = new ArrayList<Plane>(size);
        ArrayList<Plane> expectedPlanes = new ArrayList<Plane>(size);
        for (int i = 0; i < size; i++) {
            double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Plane plane = new Plane(params);
            inputPlanes.add(plane);
            
            Plane expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);
        
            expectedPlanes.add(expectedPlane);
        }                

        
        List<Plane> outPlanes1 = transformation.transformPlanesAndReturnNew(inputPlanes);
        List<Plane> outPlanes2 = new ArrayList<Plane>();
        transformation.transformPlanes(inputPlanes, outPlanes2);
                
        //check correctness
        assertEquals(outPlanes1.size(), inputPlanes.size());
        assertEquals(outPlanes2.size(), inputPlanes.size());
        for (int i = 0; i < size; i++) {
            Plane expectedPlane = expectedPlanes.get(i);
            
            Plane outPlane1 = outPlanes1.get(i);
            Plane outPlane2 = outPlanes2.get(i);
            
            expectedPlane.normalize();
            outPlane1.normalize();
            outPlane2.normalize();
            
            //check correctness
            assertEquals(expectedPlane.getA(), outPlane1.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), outPlane1.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), outPlane1.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), outPlane1.getD(), 
                    ABSOLUTE_ERROR);

            assertEquals(expectedPlane.getA(), outPlane2.getA(), 
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), outPlane2.getB(), 
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), outPlane2.getC(), 
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), outPlane2.getD(), 
                    ABSOLUTE_ERROR);
        }
    }  
    
    @Test
    public void testTransformAndOverwritePlanes() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        ArrayList<Plane> inputPlanes = new ArrayList<Plane>(size);
        ArrayList<Plane> expectedPlanes = new ArrayList<Plane>(size);
        for (int i = 0; i < size; i++) {
            double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Plane plane = new Plane(params);
            inputPlanes.add(plane);
            
            Plane expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);
        
            expectedPlanes.add(expectedPlane);
        }                
        
        transformation.transformAndOverwritePlanes(inputPlanes);
                        
        //check correctness
        assertEquals(inputPlanes.size(), size);
        for (int i = 0; i < size; i++) {
            Plane expectedPlane = expectedPlanes.get(i);
            
            Plane plane = inputPlanes.get(i);

            expectedPlane.normalize();
            plane.normalize();
            
            //check correctness
            assertEquals(expectedPlane.getA(), plane.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), plane.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), plane.getC(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), plane.getD(), ABSOLUTE_ERROR);
        }        
    }
    
    @Test
    public void testTransformLine() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            CoincidentPointsException, CoincidentPlanesException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point3D point1 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point3D point2 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
        Line3D line = new Line3D(point1, point2);
                
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Line3D expectedLine = new Line3D(point1, point2);
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();
        
        Line3D outLine1 = transformation.transformAndReturnNew(line);
        Line3D outLine2 = new Line3D(point1, point2);
        transformation.transform(line, outLine2);
        
        outLine1.normalize();
        outLine2.normalize();
        
        //check correctness
        assertEquals(expectedLine.getPlane1().getA(), 
                outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), 
                outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), 
                outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), 
                outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), 
                outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), 
                outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), 
                outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), 
                outLine1.getPlane2().getD(), ABSOLUTE_ERROR);
        
        
        assertEquals(expectedLine.getPlane1().getA(), 
                outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), 
                outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), 
                outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), 
                outLine2.getPlane1().getD(), ABSOLUTE_ERROR);
        
        assertEquals(expectedLine.getPlane2().getA(), 
                outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), 
                outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), 
                outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), 
                outLine2.getPlane2().getD(), ABSOLUTE_ERROR);  
        
        transformation.transform(line);
        
        line.normalize();
        
        //check correctness
        assertEquals(expectedLine.getPlane1().getA(),
                line.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), 
                line.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(),
                line.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(),
                line.getPlane1().getD(), ABSOLUTE_ERROR);
        
        assertEquals(expectedLine.getPlane2().getA(),
                line.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(),
                line.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(),
                line.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(),
                line.getPlane2().getD(), ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testTransformLines() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            CoincidentPlanesException, CoincidentPointsException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        ArrayList<Line3D> inputLines = new ArrayList<Line3D>(size);
        ArrayList<Line3D> expectedLines = new ArrayList<Line3D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            Point3D point1 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            Point3D point2 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
            Line3D line = new Line3D(point1, point2);
            inputLines.add(line);
            
            Line3D expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);
        
            expectedLines.add(expectedLine);
        }                

        
        List<Line3D> outLines1 = transformation.transformLines(inputLines);
        List<Line3D> outLines2 = new ArrayList<Line3D>();
        transformation.transformLines(inputLines, outLines2);
                
        //check correctness
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
        for (int i = 0; i < size; i++) {
            Line3D expectedLine = expectedLines.get(i);
            
            Line3D outLine1 = outLines1.get(i);
            Line3D outLine2 = outLines2.get(i);
            
            expectedLine.normalize();
            outLine1.normalize();
            outLine2.normalize();
            
            //check correctness
            assertEquals(expectedLine.getPlane1().getA(), 
                outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), 
                outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), 
                outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), 
                outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), 
                outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), 
                outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), 
                outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), 
                outLine1.getPlane2().getD(), ABSOLUTE_ERROR);
            
            
            assertEquals(expectedLine.getPlane1().getA(), 
                outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), 
                outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), 
                outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), 
                outLine2.getPlane1().getD(), ABSOLUTE_ERROR);
            
            assertEquals(expectedLine.getPlane2().getA(), 
                outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), 
                outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), 
                outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), 
                outLine2.getPlane2().getD(), ABSOLUTE_ERROR);            
        }
    }  
    
    @Test
    public void testTransformAndOverwriteLines() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            CoincidentPointsException, CoincidentPlanesException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        ArrayList<Line3D> inputLines = new ArrayList<Line3D>(size);
        ArrayList<Line3D> expectedLines = new ArrayList<Line3D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            Point3D point1 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            Point3D point2 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
            Line3D line = new Line3D(point1, point2);
            inputLines.add(line);
            
            Line3D expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);
        
            expectedLines.add(expectedLine);
        }                
        
        transformation.transformAndOverwriteLines(inputLines);
                        
        //check correctness
        assertEquals(inputLines.size(), size);
        for (int i = 0; i < size; i++) {
            Line3D expectedLine = expectedLines.get(i);
            
            Line3D line = inputLines.get(i);

            expectedLine.normalize();
            line.normalize();
            
            //check correctness
            assertEquals(expectedLine.getPlane1().getA(), 
                    line.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), 
                    line.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), 
                    line.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), 
                    line.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), 
                    line.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), 
                    line.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), 
                    line.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), 
                    line.getPlane2().getD(), ABSOLUTE_ERROR);
        }        
    }
    
    @Test
    public void testTransformPolygon() throws NotEnoughVerticesException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        ArrayList<Point3D> inputPoints = new ArrayList<Point3D>(size);
        ArrayList<Point3D> expectedPoints = new ArrayList<Point3D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                
        
        Polygon3D inputPolygon = new Polygon3D(inputPoints);
        Polygon3D expectedPolygon = new Polygon3D(expectedPoints);

        
        Polygon3D outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        Polygon3D outPolygon2 = new Polygon3D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);
                        
        //check correctness
        assertEquals(outPolygon1.getVertices().size(), 
                inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(), 
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            Point3D expectedPoint = expectedPolygon.getVertices().get(i);
            
            Point3D outPoint1 = outPolygon1.getVertices().get(i);
            Point3D outPoint2 = outPolygon2.getVertices().get(i);
            
            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));            
        }
        
        transformation.transform(inputPolygon);
        
        //check correctness
        assertEquals(expectedPolygon.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            Point3D expectedPoint = expectedPolygon.getVertices().get(i);
            
            Point3D outPoint = outPolygon1.getVertices().get(i);
            
            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }        
    }
    
    @Test
    public void testTransformTriangle() throws NotEnoughVerticesException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = Triangle2D.NUM_VERTICES;
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        ArrayList<Point3D> inputPoints = new ArrayList<Point3D>(size);
        ArrayList<Point3D> expectedPoints = new ArrayList<Point3D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                
        
        Triangle3D inputTriangle = new Triangle3D(inputPoints.get(0),
                inputPoints.get(1), inputPoints.get(2));
        Triangle3D expectedTriangle = new Triangle3D(expectedPoints.get(0),
                expectedPoints.get(1), expectedPoints.get(2));
        
        Triangle3D outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        Triangle3D outTriangle2 = new Triangle3D(
                new InhomogeneousPoint3D(inputPoints.get(0)),
                new InhomogeneousPoint3D(inputPoints.get(1)),
                new InhomogeneousPoint3D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);
                        
        //check correctness
        for (int i = 0; i < size; i++) {
            Point3D expectedPoint = expectedTriangle.getVertices().get(i);
            
            Point3D outPoint1 = outTriangle1.getVertices().get(i);
            Point3D outPoint2 = outTriangle2.getVertices().get(i);
            
            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));            
        }
        
        transformation.transform(inputTriangle);
        
        //check correctness
        for (int i = 0; i < size; i++) {
            Point3D expectedPoint = expectedTriangle.getVertices().get(i);
            
            Point3D outPoint = inputTriangle.getVertices().get(i);
            
            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }        
    }
    
    @Test
    public void testTransformCamera() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //generate random metric 3D point
        InhomogeneousPoint3D metricPoint = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        //generate random camera
        Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        PinholeCamera camera = new PinholeCamera(cameraMatrix);
        
        //project metric point
        Point2D p1 = camera.project(metricPoint);
        
        //generate arbitrary transformation
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);

        //transform metric point and camera
        Point3D affinePoint = transformation.transformAndReturnNew(metricPoint);
        PinholeCamera affineCamera1 = transformation.transformAndReturnNew(
                camera);
        PinholeCamera affineCamera2 = new PinholeCamera();
        transformation.transform(camera, affineCamera2);
        
        transformation.transform(camera);
        
        //project affine point with affine camera
        Point2D p2 = affineCamera1.project(affinePoint);
        Point2D p3 = affineCamera2.project(affinePoint);
        Point2D p4 = camera.project(affinePoint);
        
        //check that all projected points p1, p2, p3, p4 are still the same
        assertTrue(p1.equals(p2, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p3, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p4, ABSOLUTE_ERROR));
    }    
    
    @Test
    public void testInverse() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);                                

        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Transformation3D invTransformation1 = 
                transformation.inverseAndReturnNew();
        MetricTransformation3D invTransformation2 =
                new MetricTransformation3D();
        transformation.inverse(invTransformation2);
        
        //check that inverse transformation matrix is the inverse matrix of
        //current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS, 
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));
        
        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS, 
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));
        
        //test transforming a random point by transformation and then by its
        //inverse to ensure it remains the same
        double[] params = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Point3D inputPoint = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, params);
        
        Point3D transfPoint = transformation.transformAndReturnNew(inputPoint);
        
        Point3D invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        Point3D invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);
        
        //check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));
        
        //try inversing original transformation
        transformation.inverse();
        Point3D outPoint = transformation.transformAndReturnNew(transfPoint);
        
        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }   

    @Test
    public void testToMetric() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
                
        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Matrix expectedMatrix = transformation.asMatrix();
        
        Matrix metricMatrix = transformation.toMetric().asMatrix();
        
        //check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }        
    
    @Test
    public void testToAffine() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        
        Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        
        double[] translation = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
                
        MetricTransformation3D transformation = 
                new MetricTransformation3D(rotation, translation, scale);
        
        Matrix expectedMatrix = transformation.asMatrix();
        
        Matrix metricMatrix = transformation.toAffine().asMatrix();
        
        //check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }        
    
    @Test
    public void testCombine() throws WrongSizeException, RotationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        double[] rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        double norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        double[] rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        //normalize axis
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);
        
        
        Rotation3D rotation1 = Rotation3D.create(rotAxis1, theta1);
        Rotation3D rotation2 = Rotation3D.create(rotAxis2, theta2);
        
        double[] translation1 = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] translation2 = new double[
                EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double scale1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double scale2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        
        MetricTransformation3D transformation1 =
                new MetricTransformation3D(rotation1, translation1, scale1);
        MetricTransformation3D transformation2 =
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
                
        //combine and return result as a new transformation
        MetricTransformation3D transformation3 = 
                transformation1.combineAndReturnNew(transformation2);
        //combine into transformation1
        transformation1.combine(transformation2);
        
        //both matrices m1 and m3 need to be equal
        Matrix m3 = transformation3.asMatrix();
        Matrix m1 = transformation1.asMatrix();
        
        //check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));
        
        //besides, resulgint transformation matrices need to be equal to 
        //expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));
        
        //check also correctness of rotation and translation
        //check also correctness of rotation and translation
        assertEquals(transformation1.getRotation().getRotationAngle(),
                expectedRotation.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(transformation1.getRotation().getRotationAxis()[0],
                expectedRotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(transformation1.getRotation().getRotationAxis()[1],
                expectedRotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(transformation1.getRotation().getRotationAxis()[2],
                expectedRotation.getRotationAxis()[2], ABSOLUTE_ERROR);

        assertEquals(transformation3.getRotation().getRotationAngle(),
                expectedRotation.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getRotationAxis()[0],
                expectedRotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getRotationAxis()[1],
                expectedRotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getRotationAxis()[2],
                expectedRotation.getRotationAxis()[2], ABSOLUTE_ERROR);
        
        
        assertArrayEquals(transformation1.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);        
        
        assertEquals(transformation1.getScale(), expectedScale, ABSOLUTE_ERROR);
        assertEquals(transformation3.getScale(), expectedScale, ABSOLUTE_ERROR);

        
        //now try combining with euclidean transformations
        transformation1 =
                new MetricTransformation3D(rotation1, translation1, scale1);
        EuclideanTransformation3D euclideanTransformation = 
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
        expectedScale = scale1;  //scale2 is assumed to be 1 because 
                                //transformation is euclidean
        
        
        //combine and return result as a new transformation
        transformation3 = 
                transformation1.combineAndReturnNew(euclideanTransformation);
        //combine into transformation1
        transformation1.combine(euclideanTransformation);
        
        //both matrices m1 and m3 need to be equal
        m3 = transformation3.asMatrix();
        m1 = transformation1.asMatrix();
        
        //check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));
        
        //besides, resulgint transformation matrices need to be equal to 
        //expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));
        
        //check also correctness of rotation and translation
        //check also correctness of rotation and translation
        assertEquals(transformation1.getRotation().getRotationAngle(),
                expectedRotation.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(transformation1.getRotation().getRotationAxis()[0],
                expectedRotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(transformation1.getRotation().getRotationAxis()[1],
                expectedRotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(transformation1.getRotation().getRotationAxis()[2],
                expectedRotation.getRotationAxis()[2], ABSOLUTE_ERROR);

        assertEquals(transformation3.getRotation().getRotationAngle(),
                expectedRotation.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getRotationAxis()[0],
                expectedRotation.getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getRotationAxis()[1],
                expectedRotation.getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getRotationAxis()[2],
                expectedRotation.getRotationAxis()[2], ABSOLUTE_ERROR);
        
        
        assertArrayEquals(transformation1.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);        
        
        assertEquals(transformation1.getScale(), expectedScale, ABSOLUTE_ERROR);
        assertEquals(transformation3.getScale(), expectedScale, ABSOLUTE_ERROR);
        
    }    
    
    @Test
    public void testSetTransformationFromPoints() 
            throws CoincidentPointsException, RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double roll = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                            MAX_ANGLE_DEGREES2));
            double pitch = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                            MAX_ANGLE_DEGREES2));
            double yaw = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                            MAX_ANGLE_DEGREES2));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation =
                    new double[EuclideanTransformation3D.NUM_TRANSLATION_COORDS];
            randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
            double scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);        
        
            MetricTransformation3D transformation = 
                    new MetricTransformation3D(q, translation, scale);
        
            //test constructor with corresponding points
            InhomogeneousPoint3D inputPoint1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D inputPoint2 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D inputPoint3 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D inputPoint4 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
            Point3D outputPoint1 = transformation.transformAndReturnNew(
                    inputPoint1);
            Point3D outputPoint2 = transformation.transformAndReturnNew(
                    inputPoint2);
            Point3D outputPoint3 = transformation.transformAndReturnNew(
                    inputPoint3);
            Point3D outputPoint4 = transformation.transformAndReturnNew(
                    inputPoint4);
        
            MetricTransformation3D transformation2 =
                    new MetricTransformation3D();
            transformation2.setTransformationFromPoints(inputPoint1, 
                    inputPoint2, inputPoint3, inputPoint4, outputPoint1, 
                    outputPoint2, outputPoint3, outputPoint4);
        
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();

            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(transformation2.getScale(), scale, ABSOLUTE_ERROR);
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }
    
    private static void transformPoint(Point3D inputPoint, Point3D outputPoint,
            Rotation3D rotation, double[] translation, double scale) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(
                scale * outputPoint.getInhomX() + translation[0], 
                scale * outputPoint.getInhomY() + translation[1], 
                scale * outputPoint.getInhomZ() + translation[2]);
    }
    
    private static void transformPlane(Plane inputPlane, Plane outputPlane,
            MetricTransformation3D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        inputPlane.normalize();
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        
        Matrix invT = Utils.inverse(T);
        Matrix P = Matrix.newFromArray(inputPlane.asArray(), true);
        
        outputPlane.setParameters(invT.multiplyAndReturnNew(P).toArray());
    }
    
    private static void transformQuadric(Quadric inputQuadric, 
            Quadric outputQuadric, MetricTransformation3D transformation) 
            throws WrongSizeException, NonSymmetricMatrixException {
        
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        Matrix transT = T.transposeAndReturnNew();
        
        inputQuadric.normalize();
        Matrix Q = inputQuadric.asMatrix();
        
        Matrix transQ = transT.multiplyAndReturnNew(Q.multiplyAndReturnNew(T));
        //normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transQ);
        transQ.multiplyByScalar(1.0 / norm);
        
        outputQuadric.setParameters(transQ);
    }
    
    private static void transformDualQuadric(DualQuadric inputDualQuadric, 
            DualQuadric outputDualQuadric, 
            MetricTransformation3D transformation) throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            NonSymmetricMatrixException {
        
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        
        Matrix invT = Utils.inverse(T);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        Matrix transInvT = invT.transposeAndReturnNew();
        
        inputDualQuadric.normalize();
        Matrix dualQ = inputDualQuadric.asMatrix();
        
        Matrix transDualQ = transInvT.multiplyAndReturnNew(
                dualQ.multiplyAndReturnNew(invT));
        //normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualQ);
        transDualQ.multiplyByScalar(1.0 / norm);
        
        outputDualQuadric.setParameters(transDualQ);
    }
    
    private static void transformLine(Line3D inputLine, Line3D outputLine,
            MetricTransformation3D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException, 
            CoincidentPlanesException {
        
        inputLine.normalize();
        Plane inputPlane1 = inputLine.getPlane1();
        Plane inputPlane2 = inputLine.getPlane2();
        
        Plane outputPlane1 = new Plane();
        Plane outputPlane2 = new Plane();
        
        transformPlane(inputPlane1, outputPlane1, transformation);
        transformPlane(inputPlane2, outputPlane2, transformation);
        
        outputLine.setPlanes(outputPlane1, outputPlane2);
    }    
}
