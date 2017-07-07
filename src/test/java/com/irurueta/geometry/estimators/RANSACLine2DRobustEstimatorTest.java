/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.RANSACLine2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 1, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
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

public class RANSACLine2DRobustEstimatorTest implements 
        Line2DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -1.0;
    public static final double MAX_RANDOM_VALUE = 1.0;
    
    public static final double ABSOLUTE_ERROR = 1e-5;
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final double THRESHOLD = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public RANSACLine2DRobustEstimatorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        RANSACLine2DRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new RANSACLine2DRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < Line2DRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point2D.create());
        }
        
        estimator = new RANSACLine2DRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        estimator = null;
        try{
            estimator = new RANSACLine2DRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        Line2DRobustEstimatorListener listener =
                new Line2DRobustEstimatorListener(){

            @Override
            public void onEstimateStart(Line2DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(Line2DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    Line2DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    Line2DRobustEstimator estimator, float progress) {}
        };
        
        estimator = new RANSACLine2DRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with listener and points
        estimator = new RANSACLine2DRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new RANSACLine2DRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                RANSACLine2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetListener() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //set listener
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                Line2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                Line2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5f);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                Line2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(1);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 1);
        
        //Fail IllegalArgumentException
        try{
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetPoints() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < Line2DRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point2D.create());
        }
        estimator.setPoints(points);
        
        //check correctness
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        points.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        try{
            estimator.setPoints(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException{
        RANSACLine2DRobustEstimator estimator =
                new RANSACLine2DRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[
                Line2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Line2D line = new Line2D(a, b, c);
            
            //compute random points passing through the line
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Point2D> points = new ArrayList<Point2D>();
            List<Point2D> pointsWithError = new ArrayList<Point2D>();
            Point2D point, pointWithError;
            for(int i = 0; i < nPoints; i++){
                //get a random point belonging to the line (a*x + b*y + c*w = 0)
                //y = -(a*x + c*w)/b or x = -(b*y + c*w)/a
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
                point = new HomogeneousPoint2D(homX, homY, homW);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint2D(
                            point.getHomX() + errorX * point.getHomW(),
                            point.getHomY() + errorY * point.getHomW(), 
                            point.getHomW());
                }else{
                    //inlier point
                    pointWithError = point;
                }
                
                points.add(point);
                pointsWithError.add(pointWithError);
                
                //check that point without error is locus of line
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }
            
            RANSACLine2DRobustEstimator estimator =
                    new RANSACLine2DRobustEstimator(this, pointsWithError);
            
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Line2D line2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points without
            //error have estimated line as locus
            for(Point2D p : points){
                assertTrue(line2.isLocus(p, ABSOLUTE_ERROR));
            }
            
            //check that both lines are equal
            line.normalize();
            line2.normalize();
            assertTrue(line.equals(line2, ABSOLUTE_ERROR));
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    @Override
    public void onEstimateStart(Line2DRobustEstimator estimator) {
        estimateStart++;
        testLocked((RANSACLine2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(Line2DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((RANSACLine2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(Line2DRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((RANSACLine2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(Line2DRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((RANSACLine2DRobustEstimator)estimator);
    }
    
    private void testLocked(RANSACLine2DRobustEstimator estimator){
        try{
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setPoints(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}    
        try{
            estimator.estimate();
            fail("LockedException expected but not thrown");
        }catch(LockedException e){            
        }catch(Exception e){
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }    
}
