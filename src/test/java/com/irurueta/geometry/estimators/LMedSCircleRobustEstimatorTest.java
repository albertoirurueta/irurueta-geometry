/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.LMedSCircleRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.HomogeneousPoint2D;
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

public class LMedSCircleRobustEstimatorTest implements 
        CircleRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -1000.0;
    public static final double MAX_RANDOM_VALUE = 1000.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final double STOP_THRESHOLD = 1e-3;
    
    public static final double STD_ERROR = 100.0;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public LMedSCircleRobustEstimatorTest() {}
    
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
        LMedSCircleRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new LMedSCircleRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSCircleRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < CircleRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point2D.create());
        }
        
        estimator = new LMedSCircleRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSCircleRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        estimator = null;
        try{
            estimator = new LMedSCircleRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        CircleRobustEstimatorListener listener = 
                new CircleRobustEstimatorListener(){

            @Override
            public void onEstimateStart(CircleRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(CircleRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(CircleRobustEstimator estimator, 
                    int iteration) {}

            @Override
            public void onEstimateProgressChange(CircleRobustEstimator estimator, 
                    float progress) {}
        };
        
        estimator = new LMedSCircleRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSCircleRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        //test constructor with listener and points
        estimator = new LMedSCircleRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSCircleRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new LMedSCircleRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(), 
                LMedSCircleRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        
        //set new value
        estimator.setStopThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetListener() throws LockedException{
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
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
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                CircleRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                CircleRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
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
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                CircleRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(1);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 1);
        
        //Force IllegalArgumentException
        try{
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetPoints() throws LockedException{
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
        //check default value
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < CircleRobustEstimator.MINIMUM_SIZE; i++){
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
        LMedSCircleRobustEstimator estimator = 
                new LMedSCircleRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[CircleRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException, 
            RobustEstimatorException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            //instantiate a random circle
            Point2D center = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));

            Circle circle = new Circle(center, radius);

            //compute points in the circle locus
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            double theta = (double)nPoints / 360.0 * Math.PI / 180.0;
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);        
            List<Point2D> points = new ArrayList<Point2D>();
            List<Point2D> pointsWithError = new ArrayList<Point2D>();
            Point2D point, pointWithError;
            for(int i = 0; i < nPoints; i++){
                double angle = theta * (double)i;
                point = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);            

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint2D(
                            point.getInhomX() + errorX,
                            point.getInhomY() + errorY, 1.0);
                }else{
                    //inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);
                
                //check that point without error is within circle locus
                assertTrue(circle.isLocus(point, ABSOLUTE_ERROR));
            }

            LMedSCircleRobustEstimator estimator = 
                    new LMedSCircleRobustEstimator(this, pointsWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Circle circle2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points
            //are within the estimated circlelocus
            for(Point2D p : points){
                assertTrue(circle2.isLocus(p, ABSOLUTE_ERROR));
            }
            
            //check that both circles are equal
            assertEquals(circle.getCenter().distanceTo(circle2.getCenter()), 
                    0.0, ABSOLUTE_ERROR);
            assertEquals(circle.getRadius(), circle2.getRadius(), 
                    ABSOLUTE_ERROR);            
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(CircleRobustEstimator estimator) {
        estimateStart++;
        testLocked((LMedSCircleRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(CircleRobustEstimator estimator) {
        estimateEnd++;
        testLocked((LMedSCircleRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(CircleRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((LMedSCircleRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(CircleRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((LMedSCircleRobustEstimator)estimator);
    }
    
    private void testLocked(LMedSCircleRobustEstimator estimator){
        try{
            estimator.setStopThreshold(0.5);
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
