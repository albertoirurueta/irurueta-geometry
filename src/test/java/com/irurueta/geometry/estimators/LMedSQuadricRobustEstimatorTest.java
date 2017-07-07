/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.LMedSQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 19, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quadric;
import com.irurueta.geometry.Sphere;
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

public class LMedSQuadricRobustEstimatorTest implements 
        QuadricRobustEstimatorListener{
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double STD_ERROR = 100.0;
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double STOP_THRESHOLD = 1e-9;
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 100;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange; 
    
    public LMedSQuadricRobustEstimatorTest() {}
    
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
        LMedSQuadricRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new LMedSQuadricRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                QuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                QuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                QuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point3D> points = new ArrayList<Point3D>();
        for(int i = 0; i < QuadricRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point3D.create());
        }
        
        estimator = new LMedSQuadricRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                QuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                QuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                QuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        estimator = null;
        try{
            estimator = new LMedSQuadricRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        QuadricRobustEstimatorListener listener = 
                new QuadricRobustEstimatorListener(){

            @Override
            public void onEstimateStart(QuadricRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(QuadricRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    QuadricRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    QuadricRobustEstimator estimator, float progress) {}
        };
        
        estimator = new LMedSQuadricRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                QuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                QuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                QuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        //test constructor with listener and points
        estimator = new LMedSQuadricRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                LMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                QuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                QuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                QuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new LMedSQuadricRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(), 
                LMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        
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
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
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
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                QuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                QuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                QuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
        //check default value
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point3D> points = new ArrayList<Point3D>();
        for(int i = 0; i < QuadricRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point3D.create());
        }
        estimator.setPoints(points);
        
        //check correctness
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        points.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        try{
            estimator.setPoints(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException{
        LMedSQuadricRobustEstimator estimator = 
                new LMedSQuadricRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = 
                new double[QuadricRobustEstimator.MINIMUM_SIZE];
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
            Point3D center = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
            double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE));

            Sphere sphere = new Sphere(center, radius);
            Quadric quadric = sphere.toQuadric();

            //compute points in the quadric (i.e. sphere) locus
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            int halfPoints = (int)Math.ceil((double)nPoints / 2.0);
            double theta = (double)halfPoints / 360.0 * Math.PI / 180.0;
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);        
            List<Point3D> points = new ArrayList<Point3D>();
            List<Point3D> pointsWithError = new ArrayList<Point3D>();
            Point3D point, pointWithError;
            for(int i = 0; i < nPoints; i++){
                double angle1 = 0.0, angle2 = 0.0;
                if(i < halfPoints){
                    angle1 = theta * (double)i;
                }else{
                    angle2 = theta * (double)(i - halfPoints);
                }
                point = new HomogeneousPoint3D(
                        center.getInhomX() + radius * Math.cos(angle1) * 
                        Math.cos(angle2),
                        center.getInhomY() + radius * Math.sin(angle1) * 
                        Math.cos(angle2),
                        center.getInhomZ() + radius * Math.sin(angle2), 
                        1.0);            

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint3D(
                            point.getInhomX() + errorX,
                            point.getInhomY() + errorY,
                            point.getInhomZ() + errorZ, 1.0);
                }else{
                    //inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);
                
                //check that point without error is within conic locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(quadric.isLocus(point, ABSOLUTE_ERROR));
            }

            LMedSQuadricRobustEstimator estimator = 
                    new LMedSQuadricRobustEstimator(this, pointsWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Quadric quadric2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points
            //are within the estimated quadric locus
            for(Point3D p : points){
                assertTrue(quadric2.isLocus(p, ABSOLUTE_ERROR));
            }
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(QuadricRobustEstimator estimator) {
        estimateStart++;
        testLocked((LMedSQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(QuadricRobustEstimator estimator) {
        estimateEnd++;
        testLocked((LMedSQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(QuadricRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((LMedSQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(QuadricRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((LMedSQuadricRobustEstimator)estimator);
    }
    
    private void testLocked(LMedSQuadricRobustEstimator estimator){
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
