/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.EuclideanTransformation3DEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 23, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.EuclideanTransformation3D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Utils;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class EuclideanTransformation3DEstimatorTest implements 
        EuclideanTransformation3DEstimatorListener {
    
    public static final double MIN_ANGLE_DEGREES = -90.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;
    
    public static final double MIN_TRANSLATION = -100.0;
    public static final double MAX_TRANSLATION = 100.0;
    
    public static final double MIN_RANDOM_VALUE = 50.0;
    public static final double MAX_RANDOM_VALUE = 100.0;        
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 50;
    
    private int estimateStart;
    private int estimateEnd;
    
    public EuclideanTransformation3DEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() {
        //empty constructor
        EuclideanTransformation3DEstimator estimator = 
                new EuclideanTransformation3DEstimator();
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.MINIMUM_SIZE);
        
        
        //constructor with points
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
                
        estimator = new EuclideanTransformation3DEstimator(inputPoints, 
                outputPoints);
        
        //check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.MINIMUM_SIZE);
        
        //Force IllegalArgumentException
        List<Point3D> wrong = new ArrayList<Point3D>();
        wrong.add(Point3D.create());

        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong, 
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new EuclideanTransformation3DEstimator(inputPoints,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //constructor with listener
        estimator = new EuclideanTransformation3DEstimator(this);
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.MINIMUM_SIZE);
        
        
        //constructor with listener and points
        estimator = new EuclideanTransformation3DEstimator(this, inputPoints, 
                outputPoints);
        
        //check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.MINIMUM_SIZE);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong, 
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong, 
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new EuclideanTransformation3DEstimator(this, 
                    inputPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);    
        
        
        //empty constructor with weak minimum points allowed
        estimator = new EuclideanTransformation3DEstimator(true);
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE);
        
        
        //constructor with points
        inputPoints = new ArrayList<Point3D>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        
        outputPoints = new ArrayList<Point3D>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
                
        estimator = new EuclideanTransformation3DEstimator(inputPoints, 
                outputPoints, true);
        
        //check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong, wrong, 
                    true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new EuclideanTransformation3DEstimator(wrong, 
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new EuclideanTransformation3DEstimator(inputPoints,
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //constructor with listener
        estimator = new EuclideanTransformation3DEstimator(this, true);
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE);
        
        
        //constructor with listener and points
        estimator = new EuclideanTransformation3DEstimator(this, inputPoints, 
                outputPoints, true);
        
        //check default values
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong, 
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new EuclideanTransformation3DEstimator(this, wrong, 
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new EuclideanTransformation3DEstimator(this, 
                    inputPoints, wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);            
    }
    
    @Test
    public void testGetSetPoints() throws LockedException {
        EuclideanTransformation3DEstimator estimator = 
                new EuclideanTransformation3DEstimator();
        
        //initial values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        
        //set values
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        inputPoints.add(Point3D.create());
        
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        outputPoints.add(Point3D.create());
        
        estimator.setPoints(inputPoints, outputPoints);
        
        //check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        
        //Force IllegalArgumentException
        List<Point3D> wrong = new ArrayList<Point3D>();
        try {
            estimator.setPoints(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setPoints(wrong, outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setPoints(inputPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        EuclideanTransformation3DEstimator estimator = 
                new EuclideanTransformation3DEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }
    
    @Test
    public void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        EuclideanTransformation3DEstimator estimator =
                new EuclideanTransformation3DEstimator();
        
        //check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.MINIMUM_SIZE);
        
        //set new value
        estimator.setWeakMinimumSizeAllowed(true);
        
        //check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DEstimator.WEAK_MINIMUM_SIZE);        
    }    
    
    @Test
    public void testEstimateNoLMSE() throws NotReadyException, LockedException, 
            CoincidentPointsException {
                
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
        
            EuclideanTransformation3D transformation = 
                    new EuclideanTransformation3D(q, translation);
        
            //generate random list of input points and transform them
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            InhomogeneousPoint3D inputPoint;
            for (int i = 0; i < EuclideanTransformation3DEstimator.MINIMUM_SIZE; i++) {
                double x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                double y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                double z = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint3D(x, y, z);
                inputPoints.add(inputPoint);
            }
        
            //transform points
            List<Point3D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);
        
            EuclideanTransformation3DEstimator estimator = 
                    new EuclideanTransformation3DEstimator(this, inputPoints, 
                            outputPoints);
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());
        
            EuclideanTransformation3D transformation2 = estimator.estimate();
            EuclideanTransformation3D transformation3 = 
                    new EuclideanTransformation3D();
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());
        
            estimator.estimate(transformation3);
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
        
            //check correctness of estimated transformations
        
            //transform points using transformation2
            List<Point3D> outputPoints2 = 
                    transformation2.transformPointsAndReturnNew(inputPoints);
        
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), 
                        ABSOLUTE_ERROR));
            }            
        
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            
            
            //transform points using transformation3
            List<Point3D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);
            
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Quaternion q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();
            
            double[] translation3 = transformation3.getTranslation();
            
            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }
        
    @Test
    public void testEstimateLMSE() throws NotReadyException, LockedException, 
            CoincidentPointsException {
                
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
        
            EuclideanTransformation3D transformation = 
                    new EuclideanTransformation3D(q, translation);
        
            //generate random list of input points and transform them
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            InhomogeneousPoint3D inputPoint;
            for (int i = 0; i < EuclideanTransformation3DEstimator.MINIMUM_SIZE + 1; i++) {
                double x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                double y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                double z = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint3D(x, y, z);
                inputPoints.add(inputPoint);
            }
        
            //transform points
            List<Point3D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);
        
            EuclideanTransformation3DEstimator estimator = 
                    new EuclideanTransformation3DEstimator(this, inputPoints, 
                            outputPoints);
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());
        
            EuclideanTransformation3D transformation2 = estimator.estimate();
            EuclideanTransformation3D transformation3 = 
                    new EuclideanTransformation3D();
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());
        
            estimator.estimate(transformation3);
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
        
            //check correctness of estimated transformations
        
            //transform points using transformation2
            List<Point3D> outputPoints2 = 
                    transformation2.transformPointsAndReturnNew(inputPoints);
        
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            boolean valid = true;
            for (int i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints2.get(i), ABSOLUTE_ERROR)) {
                    valid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            if(!valid) continue;
        
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            
            //transform points using transformation3
            List<Point3D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);
            
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Quaternion q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();
            
            double[] translation3 = transformation3.getTranslation();
            
            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }    
    
    @Test
    public void testEstimatePlanar() throws NotReadyException, LockedException, 
            CoincidentPointsException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
            
            EuclideanTransformation3D transformation = 
                    new EuclideanTransformation3D(q, translation);
        
            //generate random list of input points and transform them
            //generate random plane
            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double d = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Plane plane = new Plane(a, b, c, d);
            
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            HomogeneousPoint3D inputPoint;
            for (int i = 0; i < MetricTransformation3DEstimator.WEAK_MINIMUM_SIZE; i++) {
                
                double homX, homY;
                double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(inputPoint));
                
                inputPoints.add(inputPoint);                
            }
            
            //transform points
            List<Point3D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);
        
            EuclideanTransformation3DEstimator estimator = 
                    new EuclideanTransformation3DEstimator(this, inputPoints, 
                            outputPoints, true);
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            EuclideanTransformation3D transformation2 = estimator.estimate();
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            EuclideanTransformation3D transformation3 = 
                    new EuclideanTransformation3D();            
            estimator.estimate(transformation3);
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
        
            //check correctness of estimated transformations
        
            //transform points using transformation2
            List<Point3D> outputPoints2 = 
                    transformation2.transformPointsAndReturnNew(inputPoints);
        
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            boolean isValid = true;
            for (int i = 0; i < outputPoints.size(); i++) {
                if(!outputPoints.get(i).equals(outputPoints2.get(i), 
                        ABSOLUTE_ERROR)) {
                    isValid = false;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            if(!isValid) continue;
            
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();
        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            
            //transform points using transformation3
            List<Point3D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);
            
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Quaternion q3 = transformation3.getRotation().toQuaternion();
            q3.normalize();
            
            double[] translation3 = transformation3.getTranslation();
            
            assertEquals(q.getA(), q3.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q3.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q3.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q3.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            
            numValid++;            
        }
        
        assertTrue(numValid > 0);
    }
    
    
    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    @Override
    public void onEstimateStart(EuclideanTransformation3DEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(EuclideanTransformation3DEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }
    
    private void checkLocked(EuclideanTransformation3DEstimator estimator) {
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.estimate(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setWeakMinimumSizeAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }        
    }    
}
