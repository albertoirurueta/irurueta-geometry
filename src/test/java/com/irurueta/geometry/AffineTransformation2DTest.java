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
import com.irurueta.algebra.Utils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class AffineTransformation2DTest {

    private static final int HOM_COORDS = 3;
    
    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_SCALE = 0.2;
    private static final double MAX_SCALE = 5.0;
    
    private static final int MIN_POINTS = 3;
    private static final int MAX_POINTS = 50;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int TIMES = 100;
    
    public AffineTransformation2DTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
        
    @Test
    public void testConstructor() throws AlgebraException {
        
        //Test empty constructor
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR); 
        assertTrue(transformation.getA().equals(
                transformation.getParameters().asMatrix().multiplyAndReturnNew(
                transformation.getRotation().asInhomogeneousMatrix()), 
                ABSOLUTE_ERROR));
        assertNotNull(transformation.getA());
        
        
        //Test constructor with A matrix
        Matrix A = Matrix.createWithUniformRandomValues(
                AffineTransformation2D.INHOM_COORDS, 
                AffineTransformation2D.INHOM_COORDS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        transformation = new AffineTransformation2D(A);
        
        //check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertNotNull(transformation.getParameters());
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation2D((Matrix)null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Force IllegalArgumentException
        Matrix badA = new Matrix(AffineTransformation2D.INHOM_COORDS + 1,
                AffineTransformation2D.INHOM_COORDS + 1);
        try {
            transformation = new AffineTransformation2D(badA);
            fail("IllegalArgumetnException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with scale
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);        
        transformation = new AffineTransformation2D(scale);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(), 
                scale, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), 
                scale, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR); 
        assertNotNull(transformation.getA());  
        
        
        //Test constructor with rotation
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);
        transformation = new AffineTransformation2D(rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR); 
        assertNotNull(transformation.getA());
        
        //Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation2D((Rotation2D)null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with scale and rotation
        transformation = new AffineTransformation2D(scale, rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scale, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scale, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR); 
        assertNotNull(transformation.getA());
        
        //Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation2D(scale, 
                    (Rotation2D)null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with affine parameters and rotation
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        transformation = new AffineTransformation2D(params, rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scaleX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scaleY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(), skewness, 
                ABSOLUTE_ERROR); 
        assertNotNull(transformation.getA());
        
        //Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation2D(null, rotation);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        try {
            //noinspection all
            transformation = new AffineTransformation2D(params, null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with translation
        double[] translation = new double[AffineParameters2D.INHOM_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        transformation = new AffineTransformation2D(translation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), 
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), 
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(), 
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR); 
        assertNotNull(transformation.getA());
        
        //Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation2D((double[])null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        double[] badTranslation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation = new AffineTransformation2D(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
        
        //Test constructor with matrix A and translation
        transformation = new AffineTransformation2D(A, translation);
        
        //check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertNotNull(transformation.getParameters());
        assertTrue(A.equals(transformation.getA(), ABSOLUTE_ERROR));
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation2D((Matrix)null, 
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        try {
            //noinspection all
            transformation = new AffineTransformation2D(A, null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Force IllegalArgumentException
        try {
            transformation = new AffineTransformation2D(badA, translation);
            fail("IllegalArgumetnException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            transformation = new AffineTransformation2D(A, badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with scale and translation
        transformation = new AffineTransformation2D(scale, translation);
        
        //Check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(), 
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        
        //Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation2D(scale, (double[])null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Force IllegalArgumentException
        try {
            transformation = new AffineTransformation2D(scale, badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with rotation and translation
        transformation = new AffineTransformation2D(rotation, translation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), 
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), 
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation2D((Rotation2D)null,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        try {
            //noinspection all
            transformation = new AffineTransformation2D(rotation, null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Force IllegalArgumentException
        try {
            transformation = new AffineTransformation2D(rotation, 
                    badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with scale, rotation and translation
        transformation = new AffineTransformation2D(scale, rotation, 
                translation);
        
        //check correcntess
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scale, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scale, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation2D(scale, null,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        try {
            //noinspection all
            transformation = new AffineTransformation2D(scale, rotation,
                    null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Foce IllegalArgumentException
        try {
            transformation = new AffineTransformation2D(scale, rotation, 
                    badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
        
        
        //Test constructor with affine parameters, rotation and translation
        transformation = new AffineTransformation2D(params, rotation, 
                translation);

        //check correcntess
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scaleX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scaleY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(), skewness, 
                ABSOLUTE_ERROR);
        
        //Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation2D(null, rotation, 
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        try {
            transformation = new AffineTransformation2D(params, null, 
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        try {
            //noinspection all
            transformation = new AffineTransformation2D(params, rotation,
                    null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Force IllegalARgumentException
        try {
            transformation = new AffineTransformation2D(params, rotation, 
                    badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(transformation);
    }
    
    @Test
    public void testGetSetA() throws WrongSizeException {
        Matrix A = Matrix.createWithUniformRandomValues(
                AffineTransformation2D.INHOM_COORDS, 
                AffineTransformation2D.INHOM_COORDS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        //check default value
        assertTrue(transformation.getA().equals(Matrix.identity(
                AffineParameters2D.INHOM_COORDS, 
                AffineParameters2D.INHOM_COORDS), ABSOLUTE_ERROR));
        
        //set matrix A
        transformation.setA(A);
        
        //check correctness
        assertTrue(transformation.getA().equals(A, ABSOLUTE_ERROR));

        //Force NullPointerException
        try {
            transformation.setA(null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
        
        //Force IllegalArgumentException
        Matrix badA = new Matrix(AffineTransformation2D.INHOM_COORDS + 1,
                AffineTransformation2D.INHOM_COORDS + 1);
        
        try {
            transformation.setA(badA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetRotation() throws AlgebraException {
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Rotation2D rotation = new Rotation2D(theta);
        
        //test default values
        assertEquals(transformation.getRotation().getTheta(), 0.0, 
                ABSOLUTE_ERROR);
        
        //set new value
        transformation.setRotation(rotation);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 
                ABSOLUTE_ERROR);
        
        //Force NullPointerException
        try {
            transformation.setRotation(null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
    }    
   
    @Test
    public void testAddRotation() throws AlgebraException {
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double combinedTheta = theta1 + theta2;
        combinedTheta += Math.PI;
        int times = (int)Math.floor(combinedTheta / (2.0 * Math.PI));
        combinedTheta -= times * 2.0 * Math.PI;
        combinedTheta -= Math.PI;
        
        Rotation2D rotation1 = new Rotation2D(theta1);
        Rotation2D rotation2 = new Rotation2D(theta2);
        
        //set rotation1
        transformation.setRotation(rotation1);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), theta1, 
                ABSOLUTE_ERROR);
        
        //add second rotation
        transformation.addRotation(rotation2);
        
        //check correctness
        assertEquals(transformation.getRotation().getTheta(), combinedTheta, 
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testGetSetParameters() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        //instantiate transformation
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        //check default values
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        
        AffineParameters2D defaultParams = new AffineParameters2D();
        transformation.getParameters(defaultParams);
        
        assertEquals(defaultParams.getScaleX(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getScaleY(),
                AffineParameters2D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getSkewness(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        
        //set parameters
        transformation.setParameters(params);
        
        //check correctness
        assertEquals(transformation.getParameters().getScaleX(), scaleX, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scaleY, 
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewness(), skewness, 
                ABSOLUTE_ERROR);        
        
        AffineParameters2D params2 = new AffineParameters2D();
        transformation.getParameters(params2);
        
        assertEquals(params2.getScaleX(), scaleX, ABSOLUTE_ERROR);
        assertEquals(params2.getScaleY(), scaleY, ABSOLUTE_ERROR);
        assertEquals(params2.getSkewness(), skewness, ABSOLUTE_ERROR);                
    }
    
    @Test
    public void testGetSetTranslation() {
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set new value
        transformation.setTranslation(translation);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        
        //Force IllegalArgumentException
        double[] badTranslation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS + 1];
        
        try {
            transformation.setTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
        
    @Test
    public void testAddTranslation() {
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation1 = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] translation2 = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        
        //set new value
        double[] translationCopy = Arrays.copyOf(translation1, 
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation1[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation1[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation1[1], 0.0);        
        
        //add translation
        transformation.addTranslation(translation2);
        
        //check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
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
                AffineTransformation2D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation.addTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
        
    @Test
    public void testGetSetTranslationX() {
        AffineTransformation2D transformation = new AffineTransformation2D();
        
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        InhomogeneousPoint2D translation = new InhomogeneousPoint2D(
                translationX, translationY);
        
        //check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
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
        AffineTransformation2D transformation = new AffineTransformation2D();
        
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
    public void testGetSetScale() throws AlgebraException {
        AffineTransformation2D transformation = new AffineTransformation2D();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        
        //check default value
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters2D.DEFAULT_SCALE, 0.0);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters2D.DEFAULT_SCALE, 0.0);
        
        //set value
        transformation.setScale(scale);
        
        //check correctness
        assertEquals(transformation.getParameters().getScaleX(), scale, 0.0);
        assertEquals(transformation.getParameters().getScaleY(), scale, 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(params, rotation, translation);
        
        Matrix m = Matrix.identity(AffineTransformation2D.HOM_COORDS, 
                AffineTransformation2D.HOM_COORDS);
        m.setSubmatrix(0, 0, 1, 1, 
                params.asMatrix().multiplyAndReturnNew(
                rotation.asInhomogeneousMatrix()));
        m.setSubmatrix(0, 2, 1, 2, translation);
        
        Matrix transMatrix1 = transformation.asMatrix();
        Matrix transMatrix2 = new Matrix(AffineTransformation2D.HOM_COORDS,
                AffineTransformation2D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);
        
        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));
        
        //test again by providing A matrix        
        Matrix A = Matrix.createWithUniformRandomValues(
                AffineTransformation2D.INHOM_COORDS, 
                AffineTransformation2D.INHOM_COORDS, MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        transformation = new AffineTransformation2D(A, translation);        

        m.setSubmatrix(0, 0, 1, 1, A);
        
        transMatrix1 = transformation.asMatrix();
        transMatrix2 = new Matrix(AffineTransformation2D.HOM_COORDS,
                AffineTransformation2D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);
        
        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));        
        
        Matrix T = new Matrix(AffineTransformation2D.HOM_COORDS + 1,
                AffineTransformation2D.HOM_COORDS + 1);
        try {
            transformation.asMatrix(T);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testTransformPoint() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] coords = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(params, rotation, translation);
        
        
        Point2D point = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);        
        
        Point2D expectedPoint = Point2D.create();
        transformPoint(point, expectedPoint, transformation);

        
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
    public void testTransformPoints() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(params, rotation, translation);
        
        
        ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);
        
            expectedPoints.add(expectedPoint);
        }                

        
        List<Point2D> outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        List<Point2D> outPoints2 = new ArrayList<>();
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
    public void testTransformAndOverwritePoints() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(params, rotation, translation);
        
        
        ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);
        
            expectedPoints.add(expectedPoint);
        }                
        
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
    public void testTransformConic() throws AlgebraException,
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
              
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(params, rotation, translation);
        
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
    public void testTransformConicAndPoints() throws AlgebraException,
            GeometryException {
        //create Conic from 5 points
        Conic conic = null;
        Point2D point1, point2, point3, point4, point5;
        do {
            Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
            } catch (GeometryException ignore) { }
        } while (conic == null);

        //check that points belong to conic
        assertTrue(conic.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point5, ABSOLUTE_ERROR));

        //create transformation
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY,
                skewness);

        AffineTransformation2D transformation =
                new AffineTransformation2D(affineParams, rotation, translation);

        //compute expected value
        Conic expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        //transform conic and points
        Conic outConic = transformation.transformAndReturnNew(conic);
        Point2D outPoint1 = transformation.transformAndReturnNew(point1);
        Point2D outPoint2 = transformation.transformAndReturnNew(point2);
        Point2D outPoint3 = transformation.transformAndReturnNew(point3);
        Point2D outPoint4 = transformation.transformAndReturnNew(point4);
        Point2D outPoint5 = transformation.transformAndReturnNew(point5);

        //check that transformed points still belong to transformed conic
        assertTrue(outConic.isLocus(outPoint1, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint2, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint3, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint4, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint5, ABSOLUTE_ERROR));

        //check conic correctness
        outConic.normalize();

        assertEquals(expectedConic.getA(), outConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    public void tesTransformDualConic() throws NonSymmetricMatrixException,
            AlgebraException {
        
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
              
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(params, rotation, translation);
        
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
    public void testTransformDualConicAndLines() throws AlgebraException,
            GeometryException {
        //create dual conic from 5 lines
        DualConic dualConic = null;
        Line2D line1, line2, line3, line4, line5;
        do {
            Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
            } catch (GeometryException ignore) { }
        } while (dualConic == null);

        //check that lines belong to dual conic
        assertTrue(dualConic.isLocus(line1, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line2, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line3, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line4, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line5, ABSOLUTE_ERROR));

        //create transformation
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY,
                skewness);

        AffineTransformation2D transformation =
                new AffineTransformation2D(affineParams, rotation, translation);

        //compute expected value
        DualConic expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        //transform dual conic and lines
        DualConic outDualConic = transformation.transformAndReturnNew(dualConic);
        Line2D outLine1 = transformation.transformAndReturnNew(line1);
        Line2D outLine2 = transformation.transformAndReturnNew(line2);
        Line2D outLine3 = transformation.transformAndReturnNew(line3);
        Line2D outLine4 = transformation.transformAndReturnNew(line4);
        Line2D outLine5 = transformation.transformAndReturnNew(line5);

        //check that transformed lines still belong to transformed dual conic
        assertTrue(outDualConic.isLocus(outLine1, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine2, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine3, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine4, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine5, ABSOLUTE_ERROR));

        //check dual conic correctness
        outDualConic.normalize();

        assertEquals(expectedDualConic.getA(), outDualConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformLine() throws AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] params = new double[
                Line2D.LINE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line = new Line2D(params);
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);
        
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
    public void testTransformLineAndPoints() throws AlgebraException {

        //create line from 2 points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        //ensure we create a matrix with 2 non linear dependent rows
        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        Point2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        Point2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        Line2D line = new Line2D(point1, point2);

        //check that points belong to the line
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));

        //create transformation
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY,
                skewness);

        AffineTransformation2D transformation =
                new AffineTransformation2D(affineParams, rotation, translation);

        Line2D expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        //transform line and points
        Line2D outLine = transformation.transformAndReturnNew(line);
        Point2D outPoint1 = transformation.transformAndReturnNew(point1);
        Point2D outPoint2 = transformation.transformAndReturnNew(point2);

        //check that transformed points still belong to transformed line
        assertTrue(outLine.isLocus(outPoint1));
        assertTrue(outLine.isLocus(outPoint2));

        //check line correctness
        outLine.normalize();

        assertEquals(expectedLine.getA(), outLine.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine.getC(), ABSOLUTE_ERROR);
    }
    
    @Test
    public void testTransformLines() throws AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);
        
        ArrayList<Line2D> inputLines = new ArrayList<>(size);
        ArrayList<Line2D> expectedLines = new ArrayList<>(size);
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
        List<Line2D> outLines2 = new ArrayList<>();
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
    public void testTransformAndOverwriteLines() throws AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);
        
        ArrayList<Line2D> inputLines = new ArrayList<>(size);
        ArrayList<Line2D> expectedLines = new ArrayList<>(size);
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
    public void testTransformPolygon() throws NotEnoughVerticesException, 
        AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);
        
        ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);
        
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
    public void testTransformTriangle() throws AlgebraException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int size = Triangle2D.NUM_VERTICES;

        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);
        
        ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
            Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);
            
            Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);
        
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
    public void testInverse() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);
        
        Transformation2D invTransformation1 = 
                transformation.inverseAndReturnNew();
        AffineTransformation2D invTransformation2 = 
                new AffineTransformation2D();
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
    public void testToProjective() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double[] translation = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
        Rotation2D rotation = new Rotation2D(theta);
        AffineParameters2D affineParams = new AffineParameters2D(scaleX, scaleY, 
                skewness);
        
        AffineTransformation2D transformation = 
                new AffineTransformation2D(affineParams, rotation, translation);

        Matrix expectedMatrix = transformation.asMatrix();
        double norm = Utils.normF(expectedMatrix);
        expectedMatrix.multiplyByScalar(1.0 / norm);
        
        Matrix projectiveMatrix = transformation.toProjective().asMatrix();
        norm = Utils.normF(projectiveMatrix);
        projectiveMatrix.multiplyByScalar(1.0 / norm);
        
        //check equalness up to scale        
        assertTrue(expectedMatrix.equals(projectiveMatrix, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testCombine() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] translation1 = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness1 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        double[] translation2 = new double[
                AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double scaleX2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double scaleY2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        double skewness2 = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Rotation2D rotation1 = new Rotation2D(theta1);
        AffineParameters2D affineParams1 = new AffineParameters2D(scaleX1, 
                scaleY1, skewness1);
        
        AffineTransformation2D transformation1 = 
                new AffineTransformation2D(affineParams1, rotation1, 
                translation1);

        Rotation2D rotation2 = new Rotation2D(theta2);
        AffineParameters2D affineParams2 = new AffineParameters2D(scaleX2, 
                scaleY2, skewness2);
        
        AffineTransformation2D transformation2 = 
                new AffineTransformation2D(affineParams2, rotation2, 
                translation2);
        
        Matrix expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                transformation2.asMatrix());
        Matrix expectedA = transformation1.getA().multiplyAndReturnNew(
                transformation2.getA());
        
        Matrix A1 = transformation1.getA().clone();
        Matrix t2 = Matrix.newFromArray(translation2, true);
        A1.multiply(t2);        
        double[] expectedTranslation = A1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
                
        //combine and return result as a new transformation
        AffineTransformation2D transformation3 = 
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
        
        //check correctness of A and translation
        assertTrue(transformation1.getA().equals(expectedA, ABSOLUTE_ERROR));
        assertTrue(transformation3.getA().equals(expectedA, ABSOLUTE_ERROR));
                
        assertArrayEquals(transformation1.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation, 
                ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testSetTransformationFromPoints() throws WrongSizeException, 
        DecomposerException, CoincidentPointsException {
        
        for (int t = 0; t < TIMES; t++) {
        
            Matrix A;
            do {
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation2D.INHOM_COORDS, 
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            } while(Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation2D transformation1 = 
                    new AffineTransformation2D(A, translation);

            
            //generate 3 non coincident random points
            Point2D inputPoint1, inputPoint2, inputPoint3;
            Point2D outputPoint1, outputPoint2, outputPoint3;
            Matrix m = new Matrix(6, 7); //build matrix initialized to zero
            do {
                Matrix coordsMatrix = Matrix.createWithUniformRandomValues(3, 2, 
                            -1.0, 1.0);
        
                double[] coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0, 
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint1 = Point2D.create(
                        CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
                coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1, 
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint2 = Point2D.create(
                        CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2, 
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint3 = Point2D.create(
                        CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
                        
                //Transform points using transformation
                outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
                outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
                outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
        
                //1st pair of points
                double iX = inputPoint1.getHomX();
                double iY = inputPoint1.getHomY();
                double iW = inputPoint1.getHomW();
        
                double oX = outputPoint1.getHomX();
                double oY = outputPoint1.getHomY();
                double oW = outputPoint1.getHomW();
        
                double oWiX = oW * iX;
                double oWiY = oW * iY;
                double oWiW = oW * iW;
        
                double oXiW = oX * iW;
                double oYiW = oY * iW;
                
                double norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + 
                        oWiW * oWiW + oXiW * oXiW);
        
                m.setElementAt(0, 0, oWiX / norm);
                m.setElementAt(0, 1, oWiY / norm);        
                m.setElementAt(0, 4, oWiW / norm);        
                m.setElementAt(0, 6, -oXiW / norm);
        
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oYiW * oYiW);
        
                m.setElementAt(1, 2, oWiX / norm);
                m.setElementAt(1, 3, oWiY / norm);        
                m.setElementAt(1, 5, oWiW / norm);        
                m.setElementAt(1, 6, -oYiW / norm);
        
                //2nd pair of points
                iX = inputPoint2.getHomX();
                iY = inputPoint2.getHomY();
                iW = inputPoint2.getHomW();
        
                oX = outputPoint2.getHomX();
                oY = outputPoint2.getHomY();
                oW = outputPoint2.getHomW();
        
                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;
        
                oXiW = oX * iW;
                oYiW = oY * iW;
                
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oXiW * oXiW);
        
                m.setElementAt(2, 0, oWiX / norm);
                m.setElementAt(2, 1, oWiY / norm);        
                m.setElementAt(2, 4, oWiW / norm);        
                m.setElementAt(2, 6, -oXiW / norm);
        
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oYiW * oYiW);
        
                m.setElementAt(3, 2, oWiX / norm);
                m.setElementAt(3, 3, oWiY / norm);        
                m.setElementAt(3, 5, oWiW / norm);        
                m.setElementAt(3, 6, -oYiW / norm);
               
                //3rd pair of points
                iX = inputPoint3.getHomX();
                iY = inputPoint3.getHomY();
                iW = inputPoint3.getHomW();
        
                oX = outputPoint3.getHomX();
                oY = outputPoint3.getHomY();
                oW = outputPoint3.getHomW();
        
                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;
        
                oXiW = oX * iW;
                oYiW = oY * iW;
                
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oXiW * oXiW);
        
                m.setElementAt(4, 0, oWiX / norm);
                m.setElementAt(4, 1, oWiY / norm);        
                m.setElementAt(4, 4, oWiW / norm);        
                m.setElementAt(4, 6, -oXiW / norm);
        
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oYiW * oYiW);
        
                m.setElementAt(5, 2, oWiX / norm);
                m.setElementAt(5, 3, oWiY / norm);        
                m.setElementAt(5, 5, oWiW / norm);        
                m.setElementAt(5, 6, -oYiW / norm);
            } while(Utils.rank(m) < 6);
                
        
            //Now build another transformation
            AffineTransformation2D transformation2 = 
                    new AffineTransformation2D();
        
            //estimate transformation from corresponding points
            transformation2.setTransformationFromPoints(inputPoint1, 
                    inputPoint2, inputPoint3, outputPoint1, outputPoint2, 
                    outputPoint3);
        
            //check correctness of transformation by checking transformed points
            assertTrue(outputPoint1.equals(new InhomogeneousPoint2D(
                    transformation2.transformAndReturnNew(inputPoint1)), 
                    ABSOLUTE_ERROR));
            assertTrue(outputPoint2.equals(new InhomogeneousPoint2D(
                    transformation2.transformAndReturnNew(inputPoint2)), 
                    ABSOLUTE_ERROR));
            assertTrue(outputPoint3.equals(new InhomogeneousPoint2D(
                    transformation2.transformAndReturnNew(inputPoint3)), 
                    ABSOLUTE_ERROR));
        
            //Force CoincidentPointsException
            try {
                transformation2.setTransformationFromPoints(inputPoint1, 
                        inputPoint1, inputPoint3, outputPoint1, outputPoint1, 
                        outputPoint3);
                fail("CoincidentPointsException expected but not thrown");
            } catch (CoincidentPointsException ignore) { }
        }
    }    
    
    @Test
    public void testConstructorFromPoints() throws WrongSizeException, 
        DecomposerException, CoincidentPointsException {
        
        for (int t = 0; t < TIMES; t++) {
        
            Matrix A;
            do {
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation2D.INHOM_COORDS, 
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            } while(Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation2D transformation1 = 
                    new AffineTransformation2D(A, translation);

            
            //generate 3 non coincident random points
            Point2D inputPoint1, inputPoint2, inputPoint3;
            Point2D outputPoint1, outputPoint2, outputPoint3;
            Matrix m = new Matrix(6, 7); //build matrix initialized to zero
            do {
                Matrix coordsMatrix = Matrix.createWithUniformRandomValues(3, 2, 
                            -1.0, 1.0);
        
                double[] coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0, 
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint1 = Point2D.create(
                        CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
        
                coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1, 
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint2 = Point2D.create(
                        CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2, 
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint3 = Point2D.create(
                        CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
                        
                //Transform points using transformation
                outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
                outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
                outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
        
                //1st pair of points
                double iX = inputPoint1.getHomX();
                double iY = inputPoint1.getHomY();
                double iW = inputPoint1.getHomW();
        
                double oX = outputPoint1.getHomX();
                double oY = outputPoint1.getHomY();
                double oW = outputPoint1.getHomW();
        
                double oWiX = oW * iX;
                double oWiY = oW * iY;
                double oWiW = oW * iW;
        
                double oXiW = oX * iW;
                double oYiW = oY * iW;
                
                double norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + 
                        oWiW * oWiW + oXiW * oXiW);
        
                m.setElementAt(0, 0, oWiX / norm);
                m.setElementAt(0, 1, oWiY / norm);        
                m.setElementAt(0, 4, oWiW / norm);        
                m.setElementAt(0, 6, -oXiW / norm);
        
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oYiW * oYiW);
        
                m.setElementAt(1, 2, oWiX / norm);
                m.setElementAt(1, 3, oWiY / norm);        
                m.setElementAt(1, 5, oWiW / norm);        
                m.setElementAt(1, 6, -oYiW / norm);
        
                //2nd pair of points
                iX = inputPoint2.getHomX();
                iY = inputPoint2.getHomY();
                iW = inputPoint2.getHomW();
        
                oX = outputPoint2.getHomX();
                oY = outputPoint2.getHomY();
                oW = outputPoint2.getHomW();
        
                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;
        
                oXiW = oX * iW;
                oYiW = oY * iW;
                
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oXiW * oXiW);
        
                m.setElementAt(2, 0, oWiX / norm);
                m.setElementAt(2, 1, oWiY / norm);        
                m.setElementAt(2, 4, oWiW / norm);        
                m.setElementAt(2, 6, -oXiW / norm);
        
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oYiW * oYiW);
        
                m.setElementAt(3, 2, oWiX / norm);
                m.setElementAt(3, 3, oWiY / norm);        
                m.setElementAt(3, 5, oWiW / norm);        
                m.setElementAt(3, 6, -oYiW / norm);
               
                //3rd pair of points
                iX = inputPoint3.getHomX();
                iY = inputPoint3.getHomY();
                iW = inputPoint3.getHomW();
        
                oX = outputPoint3.getHomX();
                oY = outputPoint3.getHomY();
                oW = outputPoint3.getHomW();
        
                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;
        
                oXiW = oX * iW;
                oYiW = oY * iW;
                
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oXiW * oXiW);
        
                m.setElementAt(4, 0, oWiX / norm);
                m.setElementAt(4, 1, oWiY / norm);        
                m.setElementAt(4, 4, oWiW / norm);        
                m.setElementAt(4, 6, -oXiW / norm);
        
                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                        oYiW * oYiW);
        
                m.setElementAt(5, 2, oWiX / norm);
                m.setElementAt(5, 3, oWiY / norm);        
                m.setElementAt(5, 5, oWiW / norm);        
                m.setElementAt(5, 6, -oYiW / norm);
            } while (Utils.rank(m) < 6);
                
        
            //Now build another transformation
            AffineTransformation2D transformation2 = 
                    new AffineTransformation2D(inputPoint1, 
                    inputPoint2, inputPoint3, outputPoint1, outputPoint2, 
                    outputPoint3);
        
            //check correctness of transformation by checking transformed points
            assertTrue(outputPoint1.equals(new InhomogeneousPoint2D(
                    transformation2.transformAndReturnNew(inputPoint1)), 
                    ABSOLUTE_ERROR));
            assertTrue(outputPoint2.equals(new InhomogeneousPoint2D(
                    transformation2.transformAndReturnNew(inputPoint2)), 
                    ABSOLUTE_ERROR));
            assertTrue(outputPoint3.equals(new InhomogeneousPoint2D(
                    transformation2.transformAndReturnNew(inputPoint3)), 
                    ABSOLUTE_ERROR));
        
            //Force CoincidentPointsException
            transformation2 = null;
            try {
                transformation2 = new AffineTransformation2D(inputPoint1, 
                        inputPoint1, inputPoint3, outputPoint1, outputPoint1, 
                        outputPoint3);
                fail("CoincidentPointsException expected but not thrown");
            } catch (CoincidentPointsException ignore) { }
            assertNull(transformation2);
        }
    }        
    
    @Test
    public void testSetTransformationFromLines() throws CoincidentLinesException,
            AlgebraException {
        
        for (int t = 0; t < TIMES; t++) {
        
            Matrix A;
            do {
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation2D.INHOM_COORDS, 
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation2D transformation1 = 
                    new AffineTransformation2D(A, translation);

            
            //generate 3 non coincident random lines
            Line2D inputLine1, inputLine2, inputLine3;
            Line2D outputLine1, outputLine2, outputLine3;
            Matrix m = new Matrix(6, 7); //build matrix initialized to zero
            do {
                Matrix coordsMatrix = Matrix.createWithUniformRandomValues(3, 3, 
                            -1.0, 1.0);
        
                double[] params = coordsMatrix.getSubmatrixAsArray(0, 0, 0, 
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine1 = new Line2D(params);
        
                params = coordsMatrix.getSubmatrixAsArray(1, 0, 1, 
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine2 = new Line2D(params);

                params = coordsMatrix.getSubmatrixAsArray(2, 0, 2, 
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine3 = new Line2D(params);
                        
                //Transform lines using transformation
                outputLine1 = transformation1.transformAndReturnNew(inputLine1);
                outputLine2 = transformation1.transformAndReturnNew(inputLine2);
                outputLine3 = transformation1.transformAndReturnNew(inputLine3);
        
                //1st pair of points
                double iA = inputLine1.getA();
                double iB = inputLine1.getB();
                double iC = inputLine1.getC();
        
                double oA = outputLine1.getA();
                double oB = outputLine1.getB();
                double oC = outputLine1.getC();
        
                double oCiA = oC * iA;
                double oCiB = oC * iB;
                double oCiC = oC * iC;
        
                double oAiC = oA * iC;
                double oBiC = oB * iC;
                
                double norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + 
                        oCiC * oCiC + oAiC * oAiC);
        
                m.setElementAt(0, 0, oCiA / norm);
                m.setElementAt(0, 1, oCiB / norm);        
                m.setElementAt(0, 4, oCiC / norm);        
                m.setElementAt(0, 6, -oAiC / norm);
        
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oBiC * oBiC);
        
                m.setElementAt(1, 2, oCiA / norm);
                m.setElementAt(1, 3, oCiB / norm);        
                m.setElementAt(1, 5, oCiC / norm);        
                m.setElementAt(1, 6, -oBiC / norm);
        
                //2nd pair of lines
                iA = inputLine2.getA();
                iB = inputLine2.getB();
                iC = inputLine2.getC();
        
                oA = outputLine2.getA();
                oB = outputLine2.getB();
                oC = outputLine2.getC();
        
                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;
        
                oAiC = oA * iC;
                oBiC = oB * iC;
                
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oAiC * oAiC);
        
                m.setElementAt(2, 0, oCiA / norm);
                m.setElementAt(2, 1, oCiB / norm);        
                m.setElementAt(2, 4, oCiC / norm);        
                m.setElementAt(2, 6, -oAiC / norm);
        
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oBiC * oBiC);
        
                m.setElementAt(3, 2, oCiA / norm);
                m.setElementAt(3, 3, oCiB / norm);        
                m.setElementAt(3, 5, oCiC / norm);        
                m.setElementAt(3, 6, -oBiC / norm);
               
                //3rd pair of lines
                iA = inputLine3.getA();
                iB = inputLine3.getB();
                iC = inputLine3.getC();
        
                oA = outputLine3.getA();
                oB = outputLine3.getB();
                oC = outputLine3.getC();
        
                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;
        
                oAiC = oA * iC;
                oBiC = oB * iC;
                
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oAiC * oAiC);
        
                m.setElementAt(4, 0, oCiA / norm);
                m.setElementAt(4, 1, oCiB / norm);        
                m.setElementAt(4, 4, oCiC / norm);        
                m.setElementAt(4, 6, -oAiC / norm);
        
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oBiC * oBiC);
        
                m.setElementAt(5, 2, oCiA / norm);
                m.setElementAt(5, 3, oCiB / norm);        
                m.setElementAt(5, 5, oCiC / norm);        
                m.setElementAt(5, 6, -oBiC / norm);
            } while(Utils.rank(m) < 6);
                
        
            //Now build another transformation
            AffineTransformation2D transformation2 = 
                    new AffineTransformation2D();
        
            //estimate transformation from corresponding lines
            transformation2.setTransformationFromLines(inputLine1, 
                    inputLine2, inputLine3, outputLine1, outputLine2, 
                    outputLine3);
        
            //check correctness of transformation by checking transformed points
            Line2D l = transformation2.transformAndReturnNew(inputLine1);
            l.normalize();
            
            assertEquals(Math.abs(outputLine1.getA()), Math.abs(l.getA()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getB()), Math.abs(l.getB()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getC()), Math.abs(l.getC()), 
                    ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine2);
            l.normalize();
            
            assertEquals(Math.abs(outputLine2.getA()), Math.abs(l.getA()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getB()), Math.abs(l.getB()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getC()), Math.abs(l.getC()), 
                    ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine3);
            l.normalize();
            
            assertEquals(Math.abs(outputLine3.getA()), Math.abs(l.getA()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine3.getB()), Math.abs(l.getB()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine3.getC()), Math.abs(l.getC()), 
                    ABSOLUTE_ERROR);
        
            //Force CoincidentLinesException
            try {
                transformation2.setTransformationFromLines(inputLine1, 
                        inputLine1, inputLine3, outputLine1, outputLine1, 
                        outputLine3);
                fail("CoincidentLinesException expected but not thrown");
            } catch (CoincidentLinesException ignore) { }
        }
    }        
    
    @Test
    public void testConstructorFromLines() throws CoincidentLinesException,
            AlgebraException {
        
        for (int t = 0; t < TIMES; t++) {
        
            Matrix A;
            do {
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation2D.INHOM_COORDS, 
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            } while(Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation2D transformation1 = 
                    new AffineTransformation2D(A, translation);

            
            //generate 3 non coincident random lines
            Line2D inputLine1, inputLine2, inputLine3;
            Line2D outputLine1, outputLine2, outputLine3;
            Matrix m = new Matrix(6, 7); //build matrix initialized to zero
            do {
                Matrix coordsMatrix = Matrix.createWithUniformRandomValues(3, 3, 
                            -1.0, 1.0);
        
                double[] params = coordsMatrix.getSubmatrixAsArray(0, 0, 0, 
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine1 = new Line2D(params);
        
                params = coordsMatrix.getSubmatrixAsArray(1, 0, 1, 
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine2 = new Line2D(params);

                params = coordsMatrix.getSubmatrixAsArray(2, 0, 2, 
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine3 = new Line2D(params);
                        
                //Transform lines using transformation
                outputLine1 = transformation1.transformAndReturnNew(inputLine1);
                outputLine2 = transformation1.transformAndReturnNew(inputLine2);
                outputLine3 = transformation1.transformAndReturnNew(inputLine3);
        
                //1st pair of points
                double iA = inputLine1.getA();
                double iB = inputLine1.getB();
                double iC = inputLine1.getC();
        
                double oA = outputLine1.getA();
                double oB = outputLine1.getB();
                double oC = outputLine1.getC();
        
                double oCiA = oC * iA;
                double oCiB = oC * iB;
                double oCiC = oC * iC;
        
                double oAiC = oA * iC;
                double oBiC = oB * iC;
                
                double norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + 
                        oCiC * oCiC + oAiC * oAiC);
        
                m.setElementAt(0, 0, oCiA / norm);
                m.setElementAt(0, 1, oCiB / norm);        
                m.setElementAt(0, 4, oCiC / norm);        
                m.setElementAt(0, 6, -oAiC / norm);
        
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oBiC * oBiC);
        
                m.setElementAt(1, 2, oCiA / norm);
                m.setElementAt(1, 3, oCiB / norm);        
                m.setElementAt(1, 5, oCiC / norm);        
                m.setElementAt(1, 6, -oBiC / norm);
        
                //2nd pair of lines
                iA = inputLine2.getA();
                iB = inputLine2.getB();
                iC = inputLine2.getC();
        
                oA = outputLine2.getA();
                oB = outputLine2.getB();
                oC = outputLine2.getC();
        
                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;
        
                oAiC = oA * iC;
                oBiC = oB * iC;
                
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oAiC * oAiC);
        
                m.setElementAt(2, 0, oCiA / norm);
                m.setElementAt(2, 1, oCiB / norm);        
                m.setElementAt(2, 4, oCiC / norm);        
                m.setElementAt(2, 6, -oAiC / norm);
        
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oBiC * oBiC);
        
                m.setElementAt(3, 2, oCiA / norm);
                m.setElementAt(3, 3, oCiB / norm);        
                m.setElementAt(3, 5, oCiC / norm);        
                m.setElementAt(3, 6, -oBiC / norm);
               
                //3rd pair of lines
                iA = inputLine3.getA();
                iB = inputLine3.getB();
                iC = inputLine3.getC();
        
                oA = outputLine3.getA();
                oB = outputLine3.getB();
                oC = outputLine3.getC();
        
                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;
        
                oAiC = oA * iC;
                oBiC = oB * iC;
                
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oAiC * oAiC);
        
                m.setElementAt(4, 0, oCiA / norm);
                m.setElementAt(4, 1, oCiB / norm);        
                m.setElementAt(4, 4, oCiC / norm);        
                m.setElementAt(4, 6, -oAiC / norm);
        
                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                        oBiC * oBiC);
        
                m.setElementAt(5, 2, oCiA / norm);
                m.setElementAt(5, 3, oCiB / norm);        
                m.setElementAt(5, 5, oCiC / norm);        
                m.setElementAt(5, 6, -oBiC / norm);
            } while(Utils.rank(m) < 6);
                
        
            //Now build another transformation
            AffineTransformation2D transformation2 = 
                    new AffineTransformation2D(inputLine1, 
                    inputLine2, inputLine3, outputLine1, outputLine2, 
                    outputLine3);
        
            //check correctness of transformation by checking transformed lines
            Line2D l = transformation2.transformAndReturnNew(inputLine1);
            l.normalize();
            
            assertEquals(Math.abs(outputLine1.getA()), Math.abs(l.getA()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getB()), Math.abs(l.getB()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getC()), Math.abs(l.getC()), 
                    ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine2);
            l.normalize();
            
            assertEquals(Math.abs(outputLine2.getA()), Math.abs(l.getA()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getB()), Math.abs(l.getB()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getC()), Math.abs(l.getC()), 
                    ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine3);
            l.normalize();
            
            assertEquals(Math.abs(outputLine3.getA()), Math.abs(l.getA()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine3.getB()), Math.abs(l.getB()), 
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine3.getC()), Math.abs(l.getC()), 
                    ABSOLUTE_ERROR);
        
            //Force CoincidentLinesException
            transformation2 = null;
            try {
                transformation2 = new AffineTransformation2D(inputLine1, 
                        inputLine1, inputLine3, outputLine1, outputLine1, 
                        outputLine3);
                fail("CoincidentLinesException expected but not thrown");
            } catch (CoincidentLinesException ignore) { }
            assertNull(transformation2);
        }
    }  
    
    private static void transformPoint(Point2D inputPoint, Point2D outputPoint, 
            AffineTransformation2D transformation) throws AlgebraException {
        inputPoint.normalize();
        
        Matrix A = transformation.getA().clone();
        
        double[] coords = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        coords[0] = inputPoint.getInhomX();
        coords[1] = inputPoint.getInhomY();
        
        Matrix p = Matrix.newFromArray(coords, true);
        
        A.multiply(p);
        
        double[] translation = transformation.getTranslation();
        
        outputPoint.setInhomogeneousCoordinates(
                A.getElementAtIndex(0) + translation[0], 
                A.getElementAtIndex(1) + translation[1]);        
    }    
    
    private static void transformLine(Line2D inputLine, Line2D outputLine,
            AffineTransformation2D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        inputLine.normalize();
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        
        Matrix invT = Utils.inverse(T);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        Matrix transInvT = invT.transposeAndReturnNew();
        Matrix l = Matrix.newFromArray(inputLine.asArray(), true);
        
        outputLine.setParameters(transInvT.multiplyAndReturnNew(l).toArray());
    }
    
    private static void transformConic(Conic inputConic, Conic outputConic,
            AffineTransformation2D transformation) throws AlgebraException,
            NonSymmetricMatrixException {
        
        Matrix T = transformation.asMatrix();
        Matrix invT = Utils.inverse(T);
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        Matrix transInvT = invT.transposeAndReturnNew();
        
        inputConic.normalize();
        Matrix C = inputConic.asMatrix();
        
        Matrix transC = transInvT.multiplyAndReturnNew(C.multiplyAndReturnNew(invT));
        //normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transC);
        transC.multiplyByScalar(1.0 / norm);
        
        outputConic.setParameters(transC);
    }

    private static void transformDualConic(DualConic inputDualConic, 
            DualConic outputDualConic, AffineTransformation2D transformation) 
            throws WrongSizeException, NonSymmetricMatrixException {
        
        Matrix T = transformation.asMatrix();
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);

        Matrix transT = T.transposeAndReturnNew();

        inputDualConic.normalize();
        Matrix dualC = inputDualConic.asMatrix();
        
        Matrix transDualC = T.multiplyAndReturnNew(
                dualC.multiplyAndReturnNew(transT));
        //normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualC);
        transDualC.multiplyByScalar(1.0 / norm);        
        
        outputDualConic.setParameters(transDualC);
    }    
}
