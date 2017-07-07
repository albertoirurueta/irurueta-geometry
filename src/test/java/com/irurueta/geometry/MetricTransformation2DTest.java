/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.MetricTransformation2D
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

public class MetricTransformation2DTest {
    
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final int MIN_POINTS = 3;
    public static final int MAX_POINTS = 50;        
    
    public static final double ABSOLUTE_ERROR = 1e-8;

    public static final double MIN_ANGLE_DEGREES2 = -90.0;
    public static final double MAX_ANGLE_DEGREES2 = 90.0;
    
    public static final double MIN_TRANSLATION2 = -100.0;
    public static final double MAX_TRANSLATION2 = 100.0;
    
    public static final double MIN_SCALE2 = 0.5;
    public static final double MAX_SCALE2 = 2.0;
    
    public static final int TIMES = 50;
    
    public MetricTransformation2DTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }       
    
    @Test
    public void testConstructor() throws CoincidentPointsException {
        
        //Test empty constructor
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        assertEquals(transformation.getTranslation().length, 
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);
        
        //Test constructor with rotation
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        transformation = new MetricTransformation2D(rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 0.0);
        assertEquals(transformation.getTranslation().length, 
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation2D((Rotation2D)null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        assertNull(transformation);

        
        //Test constructor with translation
        double[] translation = 
                new double[MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        transformation = new MetricTransformation2D(translation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        assertEquals(transformation.getTranslation().length, translation.length);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation2D((double[])null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation = new MetricTransformation2D(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(transformation);
        
        
        //Test constructor with scale
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        transformation = new MetricTransformation2D(scale);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        assertEquals(transformation.getTranslation().length, 
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getScale(), scale, 0.0);
        
        
        //Test constructor with rotation, translation and scale
        transformation = new MetricTransformation2D(rotation, translation, 
                scale);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 0.0);
        assertEquals(transformation.getTranslation().length, translation.length);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getScale(), scale, 0.0);

        //Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation2D(null, translation, 
                    scale);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        try {
            transformation = new MetricTransformation2D(rotation, null, scale);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
        assertNull(transformation);
        
        //Force IllegalArgumentException
        transformation = null;
        try {
            transformation = new MetricTransformation2D(rotation, 
                    badTranslation, scale);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(transformation);
        
        //test constructor with corresponding points
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            theta = randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
            rotation = new Rotation2D(theta);
            translation = 
                    new double[MetricTransformation2D.NUM_TRANSLATION_COORDS];
            randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
            scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);
            
            InhomogeneousPoint2D inputPoint1 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D inputPoint2 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D inputPoint3 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
            transformation = new MetricTransformation2D(rotation, translation, 
                    scale);
        
            Point2D outputPoint1 = transformation.transformAndReturnNew(
                    inputPoint1);
            Point2D outputPoint2 = transformation.transformAndReturnNew(
                    inputPoint2);
            Point2D outputPoint3 = transformation.transformAndReturnNew(
                    inputPoint3);

            MetricTransformation2D transformation2 = 
                    new MetricTransformation2D(inputPoint1, inputPoint2, 
                            inputPoint3, outputPoint1, outputPoint2, 
                            outputPoint3);
        
            //check correctness
            Rotation2D rotation2 = transformation2.getRotation();
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(rotation2.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);        
            assertEquals(transformation2.getScale(), scale, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }
   
    @Test
    public void testGetSetRotation() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Rotation2D rotation = new Rotation2D(theta);
        
        //test default values
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        
        //set new value
        transformation.setRotation(rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 0.0);
        
        //Force NullPointerException
        try {
            transformation.setRotation(null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException e) { }
    }
    
    @Test
    public void testAddRotation() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double combinedTheta = theta1 + theta2;
        
        Rotation2D rotation1 = new Rotation2D(theta1);
        Rotation2D rotation2 = new Rotation2D(theta2);
        
        //set rotation1
        transformation.setRotation(rotation1);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta1, 0.0);
        
        //add second rotation
        transformation.addRotation(rotation2);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), combinedTheta, 
                0.0);        
    }
    
    @Test
    public void testGetSetTranslation() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set new value
        transformation.setTranslation(translation);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS + 1];
        
        try {
            transformation.setTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testAddTranslation() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation1 = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] translation2 = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set new value
        double[] translationCopy = Arrays.copyOf(translation1, 
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation1[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation1[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation1[1], 0.0);        
        
        //add translation
        transformation.addTranslation(translation2);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0] +
                translation2[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation1[1] +
                translation2[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation1[0] +
                translation2[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation1[1] +
                translation2[1], 0.0);
        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation.addTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetTranslationX() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
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
        MetricTransformation2D transformation = new MetricTransformation2D();
        
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
    public void testSetTranslationCoordinates() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);  
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
                       
        //set new value
        transformation.setTranslation(translationX, translationY);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY, 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetTranslationPoint() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);  
        InhomogeneousPoint2D translation = new InhomogeneousPoint2D(
                translationX, translationY);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
                       
        //set new value
        transformation.setTranslation(translation);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY, 
                ABSOLUTE_ERROR);
        
        Point2D translation2 = transformation.getTranslationPoint();
        Point2D translation3 = Point2D.create();
        transformation.getTranslationPoint(translation3);
        
        //check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);                
    }
    
    @Test
    public void testAddTranslationX() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
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
        MetricTransformation2D transformation = new MetricTransformation2D();
        
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
    public void testAddTranslationCoordinates() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set values
        transformation.setTranslation(translationX1, translationY1);
        
        //add translation
        transformation.addTranslation(translationX2, translationY2);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), 
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);        
    }
    
    @Test
    public void testAddTranslationPoint() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set values
        transformation.setTranslation(translationX1, translationY1);
        
        //add translation
        Point2D translation2 = new InhomogeneousPoint2D(translationX2,
                translationY2);
        transformation.addTranslation(translation2);
        
        //check correctness
        assertEquals(transformation.getTranslationX(), 
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);        
    }
            
    @Test
    public void testGetSetScale() {
        MetricTransformation2D transformation = new MetricTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getScale(), 
                MetricTransformation2D.DEFAULT_SCALE, 0.0);
        
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
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);        
        
        Rotation2D rotation = new Rotation2D(theta);
        
        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        Matrix m = Matrix.identity(3, 3);
        m.setSubmatrix(0, 0, 1, 1, 
                rotation.asInhomogeneousMatrix().multiplyByScalarAndReturnNew(
                scale));
        m.setSubmatrix(0, 2, 1, 2, translation);
        
        Matrix transMatrix1 = transformation.asMatrix();
        Matrix transMatrix2 = new Matrix(MetricTransformation2D.HOM_COORDS,
                MetricTransformation2D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);
        
        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));
        
        Matrix T = new Matrix(MetricTransformation2D.HOM_COORDS + 1,
                MetricTransformation2D.HOM_COORDS + 1);
        try {
            transformation.asMatrix(T);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testTransformPoint() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] coords = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);        
        
        Point2D point = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Point2D expectedPoint = Point2D.create();
        transformPoint(point, expectedPoint, rotation, translation, scale);

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        Point2D outPoint1 = transformation.transformAndReturnNew(point);
        Point2D outPoint2 = Point2D.create();
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
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);        
        
        
        ArrayList<Point2D> inputPoints = new ArrayList<Point2D>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<Point2D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        List<Point2D> outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        List<Point2D> outPoints2 = new ArrayList<Point2D>();
        transformation.transformPoints(inputPoints, outPoints2);
                
        //check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (int i = 0; i < size; i++) {
            Point2D expectedPoint = expectedPoints.get(i);
            
            Point2D outPoint1 = outPoints1.get(i);
            Point2D outPoint2 = outPoints2.get(i);
            
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
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);        
        
        
        ArrayList<Point2D> inputPoints = new ArrayList<Point2D>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<Point2D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        transformation.transformAndOverwritePoints(inputPoints);
                        
        //check correctness
        assertEquals(inputPoints.size(), size);
        for (int i = 0; i < size; i++) {
            Point2D expectedPoint = expectedPoints.get(i);
            
            Point2D point = inputPoints.get(i);
            
            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }        
    }                
    
    @Test
    public void testTransformConic() throws WrongSizeException, 
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
        
        Conic conic = new Conic(a, b, c, d, e, f);
              
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);        

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        //compute expected value
        Conic expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();
        
        //make transformation
        Conic outConic1 = transformation.transformAndReturnNew(conic);
        Conic outConic2 = new Conic();
        transformation.transform(conic, outConic2);
        
        //check correctness
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
        
        //check correctness
        conic.normalize();
        
        assertEquals(expectedConic.getA(), conic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), conic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), conic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), conic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), conic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), conic.getF(), ABSOLUTE_ERROR);        
    }
    
    @Test
    public void tesTransformDualConic() throws WrongSizeException, 
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
        
        DualConic dualConic = new DualConic(a, b, c, d, e, f);
              
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE); 

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        //compute expected value
        DualConic expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();
        
        //make transformation
        DualConic outDualConic1 = transformation.transformAndReturnNew(dualConic);
        DualConic outDualConic2 = new DualConic();
        transformation.transform(dualConic, outDualConic2);
        
        //check correctness
        outDualConic1.normalize();
        outDualConic2.normalize();
        
        assertEquals(expectedDualConic.getA(), outDualConic1.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic1.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic1.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic1.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic1.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic1.getF(), 
                ABSOLUTE_ERROR);
        
        assertEquals(expectedDualConic.getA(), outDualConic2.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic2.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic2.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic2.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic2.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic2.getF(), 
                ABSOLUTE_ERROR);
        
        transformation.transform(dualConic);
        
        //check correctness
        dualConic.normalize();
        
        assertEquals(expectedDualConic.getA(), dualConic.getA(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), dualConic.getB(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), dualConic.getC(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), dualConic.getD(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), dualConic.getE(), 
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), dualConic.getF(), 
                ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testTransformLine() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] params = new double[
                Line2D.LINE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line = new Line2D(params);
                
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);         

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        Line2D expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();
        
        Line2D outLine1 = transformation.transformAndReturnNew(line);
        Line2D outLine2 = new Line2D();
        transformation.transform(line, outLine2);
        
        outLine1.normalize();
        outLine2.normalize();
        
        //check correctness
        assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);
        
        transformation.transform(line);
        
        line.normalize();
        
        //check correctness
        assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testTransformLines() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);         
        
        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        ArrayList<Line2D> inputLines = new ArrayList<Line2D>(size);
        ArrayList<Line2D> expectedLines = new ArrayList<Line2D>(size);
        for (int i = 0; i < size; i++) {
            double[] params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Line2D line = new Line2D(params);
            inputLines.add(line);
            
            Line2D expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);
        
            expectedLines.add(expectedLine);
        }                

        
        List<Line2D> outLines1 = transformation.transformLinesAndReturnNew(inputLines);
        List<Line2D> outLines2 = new ArrayList<Line2D>();
        transformation.transformLines(inputLines, outLines2);
                
        //check correctness
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
        for (int i = 0; i < size; i++) {
            Line2D expectedLine = expectedLines.get(i);
            
            Line2D outLine1 = outLines1.get(i);
            Line2D outLine2 = outLines2.get(i);
            
            expectedLine.normalize();
            outLine1.normalize();
            outLine2.normalize();
            
            //check correctness
            assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);
        }
    }
    
    @Test
    public void testTransformAndOverwriteLines() throws WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);         
                
        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        ArrayList<Line2D> inputLines = new ArrayList<Line2D>(size);
        ArrayList<Line2D> expectedLines = new ArrayList<Line2D>(size);
        for (int i = 0; i < size; i++) {
            double[] params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Line2D line = new Line2D(params);
            inputLines.add(line);
            
            Line2D expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);
        
            expectedLines.add(expectedLine);
        }                
        
        transformation.transformAndOverwriteLines(inputLines);
                        
        //check correctness
        assertEquals(inputLines.size(), size);
        for (int i = 0; i < size; i++) {
            Line2D expectedLine = expectedLines.get(i);
            
            Line2D line = inputLines.get(i);

            expectedLine.normalize();
            line.normalize();
            
            //check correctness
            assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);
        }        
    }
    
    @Test
    public void testTransformPolygon() throws NotEnoughVerticesException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);         

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        ArrayList<Point2D> inputPoints = new ArrayList<Point2D>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<Point2D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                
        
        Polygon2D inputPolygon = new Polygon2D(inputPoints);
        Polygon2D expectedPolygon = new Polygon2D(expectedPoints);

        
        Polygon2D outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        Polygon2D outPolygon2 = new Polygon2D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);
                        
        //check correctness
        assertEquals(outPolygon1.getVertices().size(), 
                inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(), 
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            Point2D expectedPoint = expectedPolygon.getVertices().get(i);
            
            Point2D outPoint1 = outPolygon1.getVertices().get(i);
            Point2D outPoint2 = outPolygon2.getVertices().get(i);
            
            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));            
        }
        
        transformation.transform(inputPolygon);
        
        //check correctness
        assertEquals(expectedPolygon.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            Point2D expectedPoint = expectedPolygon.getVertices().get(i);
            
            Point2D outPoint = outPolygon1.getVertices().get(i);
            
            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }                
    }
    
    @Test
    public void testTransformTriangle() throws NotEnoughVerticesException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = Triangle2D.NUM_VERTICES;
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);         

        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        ArrayList<Point2D> inputPoints = new ArrayList<Point2D>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<Point2D>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);
        
            expectedPoints.add(expectedPoint);
        }                
        
        Triangle2D inputTriangle = new Triangle2D(inputPoints.get(0),
                inputPoints.get(1), inputPoints.get(2));
        Triangle2D expectedTriangle = new Triangle2D(expectedPoints.get(0),
                expectedPoints.get(1), expectedPoints.get(2));
        
        Triangle2D outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        Triangle2D outTriangle2 = new Triangle2D(
                new InhomogeneousPoint2D(inputPoints.get(0)),
                new InhomogeneousPoint2D(inputPoints.get(1)),
                new InhomogeneousPoint2D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);
                        
        //check correctness
        for (int i = 0; i < size; i++) {
            Point2D expectedPoint = expectedTriangle.getVertices().get(i);
            
            Point2D outPoint1 = outTriangle1.getVertices().get(i);
            Point2D outPoint2 = outTriangle2.getVertices().get(i);
            
            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));            
        }
        
        transformation.transform(inputTriangle);
        
        //check correctness
        for (int i = 0; i < size; i++) {
            Point2D expectedPoint = expectedTriangle.getVertices().get(i);
            
            Point2D outPoint = inputTriangle.getVertices().get(i);
            
            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }                
    }
    
    @Test
    public void testInverse() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);         
        
        MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);
        
        Transformation2D invTransformation1 = 
                transformation.inverseAndReturnNew();
        MetricTransformation2D invTransformation2 = 
                new MetricTransformation2D();
        transformation.inverse(invTransformation2);
        
        //check that inverse transformation matrix is the inverse matrix of
        //current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation2D.HOM_COORDS, 
                MetricTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));
        
        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation2D.HOM_COORDS, 
                MetricTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));
        
        //test transforming a random point by transformation and then by its
        //inverse to ensure it remains the same
        double[] params = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Point2D inputPoint = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, params);
        
        Point2D transfPoint = transformation.transformAndReturnNew(inputPoint);
        
        Point2D invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        Point2D invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);
        
        //check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));
        
        //try inversing original transformation
        transformation.inverse();
        Point2D outPoint = transformation.transformAndReturnNew(transfPoint);
        
        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testToMetric() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
                
        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
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
        
        Rotation2D rotation = new Rotation2D(theta);
        
        double[] translation = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
                
        MetricTransformation2D transformation = 
                new MetricTransformation2D(rotation, translation, scale);
        
        Matrix expectedMatrix = transformation.asMatrix();
        
        Matrix metricMatrix = transformation.toAffine().asMatrix();
        
        //check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }    
    
    @Test
    public void testCombine() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        Rotation2D rotation1 = new Rotation2D(theta1);
        Rotation2D rotation2 = new Rotation2D(theta2);
        
        double[] translation1 = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] translation2 = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double scale1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double scale2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        
        MetricTransformation2D transformation1 =
                new MetricTransformation2D(rotation1, translation1, scale1);
        MetricTransformation2D transformation2 =
                new MetricTransformation2D(rotation2, translation2, scale2);
        

        Matrix expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                transformation2.asMatrix());
        
        Rotation2D expectedRotation = rotation1.combineAndReturnNew(rotation2);
        Matrix rotM1 = rotation1.asInhomogeneousMatrix();
        Matrix t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        rotM1.multiplyByScalar(scale1);
        double[] expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
        double expectedScale = scale1 * scale2;
                
        //combine and return result as a new transformation
        MetricTransformation2D transformation3 = 
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
        assertEquals(transformation1.getRotation().getTheta(), 
                expectedRotation.getTheta(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getTheta(), 
                expectedRotation.getTheta(), ABSOLUTE_ERROR);
        
        
        assertArrayEquals(transformation1.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);        
        
        assertEquals(transformation1.getScale(), expectedScale, ABSOLUTE_ERROR);
        assertEquals(transformation3.getScale(), expectedScale, ABSOLUTE_ERROR);

        
        //now try combining with euclidean transformations
        transformation1 =
                new MetricTransformation2D(rotation1, translation1, scale1);
        EuclideanTransformation2D euclideanTransformation = 
                new EuclideanTransformation2D(rotation2, translation2);
        
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
        assertEquals(transformation1.getRotation().getTheta(), 
                expectedRotation.getTheta(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getTheta(), 
                expectedRotation.getTheta(), ABSOLUTE_ERROR);
        
        
        assertArrayEquals(transformation1.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);        
        
        assertEquals(transformation1.getScale(), expectedScale, ABSOLUTE_ERROR);
        assertEquals(transformation3.getScale(), expectedScale, ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testSetTransformationFromPoints() 
            throws CoincidentPointsException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES2, 
                    MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
            Rotation2D rotation = new Rotation2D(theta);     
        
            double[] translation = 
                    new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
            randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
            double scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);
        
        
            MetricTransformation2D transformation = 
                    new MetricTransformation2D(rotation, translation, scale);
        
            InhomogeneousPoint2D inputPoint1 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D inputPoint2 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D inputPoint3 = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
            Point2D outputPoint1 = transformation.transformAndReturnNew(
                    inputPoint1);
            Point2D outputPoint2 = transformation.transformAndReturnNew(
                    inputPoint2);
            Point2D outputPoint3 = transformation.transformAndReturnNew(
                    inputPoint3);

            MetricTransformation2D transformation2 = 
                    new MetricTransformation2D();
            transformation2.setTransformationFromPoints(inputPoint1, 
                    inputPoint2, inputPoint3, outputPoint1, outputPoint2, 
                    outputPoint3);
        
            //check correctness
            Rotation2D rotation2 = transformation2.getRotation();
            double[] translation2 = transformation2.getTranslation();

            assertEquals(rotation2.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);        
            assertEquals(transformation2.getScale(), scale, ABSOLUTE_ERROR);
            numValid++;
        }
        
        assertEquals(numValid, TIMES);         
    }
    
    private static void transformPoint(Point2D inputPoint, Point2D outputPoint, 
            Rotation2D rotation, double[] translation, double scale) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(
                scale * outputPoint.getInhomX() + translation[0], 
                scale * outputPoint.getInhomY() + translation[1]);        
    }    
    
    private static void transformLine(Line2D inputLine, Line2D outputLine,
            MetricTransformation2D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        inputLine.normalize();
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        
        Matrix invT = Utils.inverse(T);        
        Matrix l = Matrix.newFromArray(inputLine.asArray(), true);
        
        outputLine.setParameters(invT.multiplyAndReturnNew(l).toArray());
    }
    
    private static void transformConic(Conic inputConic, Conic outputConic,
            MetricTransformation2D transformation) throws WrongSizeException,
            NonSymmetricMatrixException {
        
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        Matrix transT = T.transposeAndReturnNew();
        
        inputConic.normalize();
        Matrix C = inputConic.asMatrix();
        
        Matrix transC = transT.multiplyAndReturnNew(C.multiplyAndReturnNew(T));
        //normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transC);
        transC.multiplyByScalar(1.0 / norm);
        
        outputConic.setParameters(transC);
    }

    private static void transformDualConic(DualConic inputDualConic, 
            DualConic outputDualConic, MetricTransformation2D transformation) 
            throws WrongSizeException, NonSymmetricMatrixException,
            RankDeficientMatrixException, DecomposerException {
        
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        
        Matrix invT = Utils.inverse(T);        
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        Matrix transInvT = invT.transposeAndReturnNew();
        
        inputDualConic.normalize();
        Matrix dualC = inputDualConic.asMatrix();
        
        Matrix transDualC = transInvT.multiplyAndReturnNew(
                dualC.multiplyAndReturnNew(invT));
        //normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualC);
        transDualC.multiplyByScalar(1.0 / norm);        
        
        outputDualConic.setParameters(transDualC);
    }    
}
