/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.MetricTransformation2DEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 23, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MetricTransformation2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Rotation2D;
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

public class MetricTransformation2DEstimatorTest implements 
        MetricTransformation2DEstimatorListener {
    
    public static final double MIN_ANGLE_DEGREES = -90.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;
    
    public static final double MIN_TRANSLATION = -100.0;
    public static final double MAX_TRANSLATION = 100.0;
    
    public static final double MIN_RANDOM_VALUE = 50.0;
    public static final double MAX_RANDOM_VALUE = 100.0;                
    
    public static final double MIN_SCALE = 0.5;
    public static final double MAX_SCALE = 2.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 50;
    
    private int estimateStart;
    private int estimateEnd;
    
    public MetricTransformation2DEstimatorTest() { }
    
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
        MetricTransformation2DEstimator estimator = 
                new MetricTransformation2DEstimator();
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation2DEstimator.MINIMUM_SIZE);
        
        //constructor with points
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        
        
        //constructor with points
        estimator = new MetricTransformation2DEstimator(inputPoints, 
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
                MetricTransformation2DEstimator.MINIMUM_SIZE);
        
        //Force IllegalArgumentException
        List<Point2D> wrong = new ArrayList<Point2D>();
        wrong.add(Point2D.create());

        estimator = null;
        try {
            estimator = new MetricTransformation2DEstimator(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new MetricTransformation2DEstimator(wrong, 
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new MetricTransformation2DEstimator(inputPoints,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //constructor with listener
        estimator = new MetricTransformation2DEstimator(this);
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation2DEstimator.MINIMUM_SIZE);        
        
        //constructor with listener and points
        estimator = new MetricTransformation2DEstimator(this, inputPoints, 
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
                MetricTransformation2DEstimator.MINIMUM_SIZE);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MetricTransformation2DEstimator(this, wrong, 
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new MetricTransformation2DEstimator(this, wrong, 
                    outputPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new MetricTransformation2DEstimator(this, 
                    inputPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
        

        //empty constructor with weak minimum points allowed
        estimator = new MetricTransformation2DEstimator(true);
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE);
        
        //constructor with points
        inputPoints = new ArrayList<Point2D>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        
        outputPoints = new ArrayList<Point2D>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        
        
        //constructor with points
        estimator = new MetricTransformation2DEstimator(inputPoints, 
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
                MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MetricTransformation2DEstimator(wrong, wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new MetricTransformation2DEstimator(wrong, 
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new MetricTransformation2DEstimator(inputPoints,
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //constructor with listener
        estimator = new MetricTransformation2DEstimator(this, true);
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE);                
        
        
        //constructor with listener and points
        estimator = new MetricTransformation2DEstimator(this, inputPoints, 
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
                MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE);                
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MetricTransformation2DEstimator(this, wrong, 
                    wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
        try {
            estimator = new MetricTransformation2DEstimator(this, wrong, 
                    outputPoints, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new MetricTransformation2DEstimator(this, 
                    inputPoints, wrong, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetPoints() throws LockedException {
        MetricTransformation2DEstimator estimator = 
                new MetricTransformation2DEstimator();
        
        //initial values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        
        //set values
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        inputPoints.add(Point2D.create());
        
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        outputPoints.add(Point2D.create());
        
        estimator.setPoints(inputPoints, outputPoints);
        
        //check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        
        //Force IllegalArgumentException
        List<Point2D> wrong = new ArrayList<Point2D>();
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
        MetricTransformation2DEstimator estimator = 
                new MetricTransformation2DEstimator();
        
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
        MetricTransformation2DEstimator estimator =
                new MetricTransformation2DEstimator();
        
        //check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation2DEstimator.MINIMUM_SIZE);        
        
        //set new value
        estimator.setWeakMinimumSizeAllowed(true);
        
        //check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE);        
    }        
    
    @Test
    public void testEstimateNoLMSE() throws NotReadyException, LockedException, 
            CoincidentPointsException {
                
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
                    
            double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            Rotation2D rotation = new Rotation2D(theta);
        
            double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
            
            double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        
            MetricTransformation2D transformation = 
                    new MetricTransformation2D(rotation, translation, scale);
        
            //generate random list of input points and transform them
            List<Point2D> inputPoints = new ArrayList<Point2D>();
            InhomogeneousPoint2D inputPoint;
            for (int i = 0; i < MetricTransformation2DEstimator.MINIMUM_SIZE; i++) {
                double x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                double y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint2D(x, y);
                inputPoints.add(inputPoint);
            }
        
            //transform points
            List<Point2D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);
        
            MetricTransformation2DEstimator estimator = 
                    new MetricTransformation2DEstimator(this, inputPoints, 
                            outputPoints);
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            MetricTransformation2D transformation2 = estimator.estimate();
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            MetricTransformation2D transformation3 = 
                    new MetricTransformation2D();            
            estimator.estimate(transformation3);
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
        
            //check correctness of estimated transformations
        
            //transform points using transformation2
            List<Point2D> outputPoints2 = 
                    transformation2.transformPointsAndReturnNew(inputPoints);
        
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Rotation2D rotation2 = transformation2.getRotation();
        
            double[] translation2 = transformation2.getTranslation();
            double scale2 = transformation2.getScale();
        
            assertEquals(rotation2.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);
            
            //transform points using transformation3
            List<Point2D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);
            
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Rotation2D rotation3 = transformation3.getRotation();
            
            double[] translation3 = transformation3.getTranslation();
            double scale3 = transformation3.getScale();
            
            assertEquals(rotation3.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            assertEquals(scale, scale3, ABSOLUTE_ERROR);
            
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
                    
            double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            Rotation2D rotation = new Rotation2D(theta);
        
            double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
            
            double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        
            MetricTransformation2D transformation = 
                    new MetricTransformation2D(rotation, translation, scale);
        
            //generate random list of input points and transform them
            List<Point2D> inputPoints = new ArrayList<Point2D>();
            InhomogeneousPoint2D inputPoint;
            for (int i = 0; i < MetricTransformation2DEstimator.MINIMUM_SIZE + 1; i++) {
                double x = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                double y = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION);
                inputPoint = new InhomogeneousPoint2D(x, y);
                inputPoints.add(inputPoint);
            }
        
            //transform points
            List<Point2D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);
        
            MetricTransformation2DEstimator estimator = 
                    new MetricTransformation2DEstimator(this, inputPoints, 
                            outputPoints);
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            MetricTransformation2D transformation2 = estimator.estimate();
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            MetricTransformation2D transformation3 = 
                    new MetricTransformation2D();            
            estimator.estimate(transformation3);
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
        
            //check correctness of estimated transformations
        
            //transform points using transformation2
            List<Point2D> outputPoints2 = 
                    transformation2.transformPointsAndReturnNew(inputPoints);
        
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints2.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Rotation2D rotation2 = transformation2.getRotation();
        
            double[] translation2 = transformation2.getTranslation();
            double scale2 = transformation2.getScale();
        
            assertEquals(rotation2.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            assertEquals(scale, scale2, ABSOLUTE_ERROR);
            
            //transform points using transformation3
            List<Point2D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);
            
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), 
                        ABSOLUTE_ERROR));
            }
            
            Rotation2D rotation3 = transformation3.getRotation();
            
            double[] translation3 = transformation3.getTranslation();
            double scale3 = transformation3.getScale();
            
            assertEquals(rotation3.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            assertEquals(scale, scale3, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateColinear() throws NotReadyException, LockedException, 
            CoincidentPointsException {
                
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
                    
            double theta = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            Rotation2D rotation = new Rotation2D(theta);
        
            double[] translation = new double[2];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
            
            double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        
            MetricTransformation2D transformation = 
                    new MetricTransformation2D(rotation, translation, scale);
        
            //generate random list of input points and transform them
            //generate random line
            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Line2D line = new Line2D(a, b, c);
            
            List<Point2D> inputPoints = new ArrayList<Point2D>();
            HomogeneousPoint2D inputPoint;
            for (int i = 0; i < MetricTransformation2DEstimator.WEAK_MINIMUM_SIZE; i++) {
                double homX, homY;
                double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homW) / a;
                }
                
                inputPoint = new HomogeneousPoint2D(homX, homY, homW);
                
                assertTrue(line.isLocus(inputPoint));
                
                inputPoints.add(inputPoint);
            }
        
            //transform points
            List<Point2D> outputPoints = transformation.
                    transformPointsAndReturnNew(inputPoints);
        
            MetricTransformation2DEstimator estimator = 
                    new MetricTransformation2DEstimator(this, inputPoints, 
                            outputPoints, true);
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            MetricTransformation2D transformation2 = estimator.estimate();
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertFalse(estimator.isLocked());

            MetricTransformation2D transformation3 = 
                    new MetricTransformation2D();            
            estimator.estimate(transformation3);
        
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertFalse(estimator.isLocked());
        
        
            //check correctness of estimated transformations
        
            //transform points using transformation2
            List<Point2D> outputPoints2 = 
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
            
            Rotation2D rotation2 = transformation2.getRotation();
        
            double[] translation2 = transformation2.getTranslation();
            double scale2 = transformation2.getScale();

            if (Math.abs(rotation2.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation2.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            boolean failed = false;
            for (int i = 0; i < translation.length; i++) {
                if (Math.abs(translation[i] - translation2[i]) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
            }
            if (failed) {
                continue;
            }
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            if (Math.abs(scale - scale2) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scale, scale2, ABSOLUTE_ERROR);
            
            //transform points using transformation3
            List<Point2D> outputPoints3 =
                    transformation3.transformPointsAndReturnNew(inputPoints);
            
            //check correctness
            assertEquals(outputPoints.size(), outputPoints2.size());
            for (int i = 0; i < outputPoints.size(); i++) {
                if (!outputPoints.get(i).equals(outputPoints3.get(i), ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(outputPoints.get(i).equals(outputPoints3.get(i), 
                        ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }
            
            Rotation2D rotation3 = transformation3.getRotation();
            
            double[] translation3 = transformation3.getTranslation();
            double scale3 = transformation3.getScale();

            if (Math.abs(rotation3.getTheta() - rotation.getTheta()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation3.getTheta(), rotation.getTheta(), 
                    ABSOLUTE_ERROR);
            for (int i = 0; i < translation.length; i++) {
                if (Math.abs(translation[i] - translation3[i]) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
            }
            if (failed) {
                continue;
            }
            assertArrayEquals(translation, translation3, ABSOLUTE_ERROR);
            if (Math.abs(scale - scale3) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scale, scale3, ABSOLUTE_ERROR);
            
            numValid++;
            break;
        }
        
        assertTrue(numValid > 0);
    }
    
    private void reset() {
        estimateStart = estimateEnd = 0;
    }    
    
    @Override
    public void onEstimateStart(MetricTransformation2DEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(MetricTransformation2DEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }
    
    private void checkLocked(MetricTransformation2DEstimator estimator) {
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
