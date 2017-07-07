/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACConicRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 17, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.Conic;
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

public class PROSACConicRobustEstimatorTest 
implements ConicRobustEstimatorListener{
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final double MIN_RANDOM_VALUE = -1.0;
    public static final double MAX_RANDOM_VALUE = 1.0;
    
    public static final double STD_ERROR = 1.0;

    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double THRESHOLD = 1e-6;
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 100;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;  
    
    public PROSACConicRobustEstimatorTest() {}
    
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
        PROSACConicRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROSACConicRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < ConicRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point2D.create());
        }
        
        estimator = new PROSACConicRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<Point2D>();
        estimator = null;
        try{
            estimator = new PROSACConicRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        ConicRobustEstimatorListener listener = 
                new ConicRobustEstimatorListener(){

            @Override
            public void onEstimateStart(ConicRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(ConicRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(ConicRobustEstimator estimator, 
                    int iteration) {}

            @Override
            public void onEstimateProgressChange(ConicRobustEstimator estimator, 
                    float progress) {}
        };
        
        estimator = new PROSACConicRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        //test constructor with listener and points
        estimator = new PROSACConicRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACConicRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);      
        
        
        //test constructor with quality scores
        double[] qualityScores = new double[ConicRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACConicRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try{
            estimator = new PROSACConicRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with points and quality scores
        estimator = new PROSACConicRobustEstimator(points, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACConicRobustEstimator(emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACConicRobustEstimator(points, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROSACConicRobustEstimator(listener, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACConicRobustEstimator(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener, points and quality scores
        estimator = new PROSACConicRobustEstimator(listener, points, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACConicRobustEstimator(listener, emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACConicRobustEstimator(listener, points, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);          
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                PROSACConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetListener() throws LockedException{
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
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
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                ConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                ConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                ConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
        //check default value
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point2D> points = new ArrayList<Point2D>();
        for(int i = 0; i < ConicRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point2D.create());
        }
        estimator.setPoints(points);
        
        //check correctness
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
        double[] qualityScores = new double[points.size()];
        estimator.setQualityScores(qualityScores);
        
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
        PROSACConicRobustEstimator estimator = new PROSACConicRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[ConicRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        qualityScores = new double[1];
        try{
            estimator.setQualityScores(qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException, 
            RobustEstimatorException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            //instantiate a random circle
            Point2D center = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));

            Circle circle = new Circle(center, radius);
            Conic conic = circle.toConic();

            //compute points in the conic (i.e. circle) locus
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            double theta = (double)nPoints / 360.0 * Math.PI / 180.0;
            double[] qualityScores = new double[nPoints];
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
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint2D(
                            point.getInhomX() + errorX,
                            point.getInhomY() + errorY, 1.0);
                    
                    double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier point
                    pointWithError = point;
                }

                points.add(point);
                pointsWithError.add(pointWithError);
                
                //check that point without error is within conic locus
                assertTrue(circle.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(conic.isLocus(point, ABSOLUTE_ERROR));
            }

            PROSACConicRobustEstimator estimator = 
                    new PROSACConicRobustEstimator(this, pointsWithError, 
                    qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Conic conic2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points
            //are within the estimated conic locus
            for(Point2D p : points){
                assertTrue(conic2.isLocus(p, ABSOLUTE_ERROR));
            }
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(ConicRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(ConicRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(ConicRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROSACConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(ConicRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROSACConicRobustEstimator)estimator);
    }
    
    private void testLocked(PROSACConicRobustEstimator estimator){
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
    }
}
