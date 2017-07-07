/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROMedSQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 21, 2015
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

public class PROMedSQuadricRobustEstimatorTest implements 
        QuadricRobustEstimatorListener{
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double STD_ERROR = 100.0;

    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double THRESHOLD = 1e-6;
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int TIMES = 10;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange; 
    
    public PROMedSQuadricRobustEstimatorTest() {}
    
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
        PROMedSQuadricRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROMedSQuadricRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        
        estimator = new PROMedSQuadricRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        estimator = null;
        try{
            estimator = new PROMedSQuadricRobustEstimator(emptyPoints);
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
        
        estimator = new PROMedSQuadricRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        estimator = new PROMedSQuadricRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROMedSQuadricRobustEstimator(listener, 
                    emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);  
        
        //test constructor with quality scores
        double[] qualityScores = new double[QuadricRobustEstimator.MINIMUM_SIZE];
        estimator = new PROMedSQuadricRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try{
            estimator = new PROMedSQuadricRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with points and quality scores
        estimator = new PROMedSQuadricRobustEstimator(points, qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROMedSQuadricRobustEstimator(emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROMedSQuadricRobustEstimator(points, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROMedSQuadricRobustEstimator(listener, qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROMedSQuadricRobustEstimator(listener, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener, points and quality scores
        estimator = new PROMedSQuadricRobustEstimator(listener, points, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROMedSQuadricRobustEstimator(listener, emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROMedSQuadricRobustEstimator(listener, points, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);                  
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(), 
                PROMedSQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        
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
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
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
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
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
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
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
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
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
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
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
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
        double[] qualityScores = new double[points.size()];
        estimator.setQualityScores(qualityScores);
        
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
        PROMedSQuadricRobustEstimator estimator = 
                new PROMedSQuadricRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = 
                new double[QuadricRobustEstimator.MINIMUM_SIZE];
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
        
        int numValid = 0;
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
            double[] qualityScores = new double[nPoints];
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
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint3D(
                            point.getInhomX() + errorX,
                            point.getInhomY() + errorY,
                            point.getInhomZ() + errorZ, 1.0);
                    
                    double error = Math.sqrt(errorX * errorX + errorY * errorY +
                            errorZ * errorZ);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;                    
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

            PROMedSQuadricRobustEstimator estimator = 
                    new PROMedSQuadricRobustEstimator(this, pointsWithError,
                    qualityScores);
            
            estimator.setStopThreshold(THRESHOLD);
            
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
            boolean validPoints = true;
            for(Point3D p : points){
                if (!quadric2.isLocus(p, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(quadric2.isLocus(p, ABSOLUTE_ERROR));
            }
            
            if (!validPoints) {
                continue;
            }
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(QuadricRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROMedSQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(QuadricRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROMedSQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(QuadricRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROMedSQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(QuadricRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROMedSQuadricRobustEstimator)estimator);
    }
    
    private void testLocked(PROMedSQuadricRobustEstimator estimator){
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
    }            
}
