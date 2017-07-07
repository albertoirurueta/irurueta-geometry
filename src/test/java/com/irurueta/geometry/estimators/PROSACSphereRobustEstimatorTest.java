/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACSphereRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
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

public class PROSACSphereRobustEstimatorTest implements 
        SphereRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 1e-5;
    public static final double LARGE_ABSOLUTE_ERROR = 5e-4;
    
    public static final int MIN_POINTS = 100;
    public static final int MAX_POINTS = 500;
    
    public static final double THRESHOLD = 1e-6;
    
    public static final double MIN_SCORE_ERROR = -0.03;
    public static final double MAX_SCORE_ERROR = 0.03;     
    
    public static final double STD_ERROR = 100.0;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 10;
    
    public static final int TIMES = 10; //PROSAC seems to be problematic with 
                                        //spheres so we reduce number of times
                                        //we repeat random tests

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public PROSACSphereRobustEstimatorTest() {}
    
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
        PROSACSphereRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROSACSphereRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point3D> points = new ArrayList<Point3D>();
        for(int i = 0; i < SphereRobustEstimator.MINIMUM_SIZE; i++){
            points.add(Point3D.create());
        }
        
        estimator = new PROSACSphereRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<Point3D>();
        estimator = null;
        try{
            estimator = new PROSACSphereRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        SphereRobustEstimatorListener listener = 
                new SphereRobustEstimatorListener(){

            @Override
            public void onEstimateStart(SphereRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(SphereRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(SphereRobustEstimator estimator, 
                    int iteration) {}

            @Override
            public void onEstimateProgressChange(SphereRobustEstimator estimator, 
                    float progress) {}
        };
        
        estimator = new PROSACSphereRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());   
        
        //test constructor with listener and points
        estimator = new PROSACSphereRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACSphereRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with quality scores
        double[] qualityScores = new double[SphereRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACSphereRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try{
            estimator = new PROSACSphereRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with points and scores
        estimator = new PROSACSphereRobustEstimator(points, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACSphereRobustEstimator(emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACSphereRobustEstimator(points, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROSACSphereRobustEstimator(listener, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);   
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACSphereRobustEstimator(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
                
        
        //test constructor with listener, points and quality scores
        estimator = new PROSACSphereRobustEstimator(listener, points, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACSphereRobustEstimator(listener, emptyPoints, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACSphereRobustEstimator(listener, points, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);                 
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                PROSACSphereRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
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
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
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
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                SphereRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                SphereRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                SphereRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
        //check default value
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point3D> points = new ArrayList<Point3D>();
        for(int i = 0; i < SphereRobustEstimator.MINIMUM_SIZE; i++){
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
        PROSACSphereRobustEstimator estimator = 
                new PROSACSphereRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[SphereRobustEstimator.MINIMUM_SIZE];
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
        
        int failedCount = 0;
        boolean failed;
        for(int t = 0; t < TIMES; t++){
            //instantiate a random circle
            Point3D center = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    1.0);
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));

            Sphere sphere = new Sphere(center, radius);

            //compute points in the circle locus
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            int halfPoints = (int)Math.ceil((double)nPoints / 2.0);
            double theta = (double)nPoints / 360.0 * Math.PI / 180.0;
            double[] qualityScores = new double[nPoints];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);        
            List<Point3D> points = new ArrayList<Point3D>();
            List<Point3D> pointsWithError = new ArrayList<Point3D>();
            Point3D point, pointWithError;
            for(int i = 0; i < nPoints; i++){
                double angle1 = 0.0, angle2 = 0.0;
                if(i < halfPoints){
                    angle1 = 2.0 * theta * (double)i;
                }else{
                    angle2 = 2.0 * theta * (double)(i - halfPoints);
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
                
                //check that point without error is within circle locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
            }

            PROSACSphereRobustEstimator estimator = 
                    new PROSACSphereRobustEstimator(this, pointsWithError,
                    qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Sphere sphere2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points
            //are within the estimated sphere locus    
            failed = false;
            for(Point3D p : points){
                if(!sphere2.isLocus(p, LARGE_ABSOLUTE_ERROR)) failed = true;
            }
            
            if(!failed){            
                //check that both spheres are equal
                assertEquals(sphere.getCenter().distanceTo(sphere2.getCenter()),
                        0.0, ABSOLUTE_ERROR);
                assertEquals(sphere.getRadius(), sphere2.getRadius(), 
                        ABSOLUTE_ERROR);
            }
            
            if(failed) failedCount++;
        }
        
        assertTrue(failedCount < TIMES / 2);
    }    
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(SphereRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACSphereRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(SphereRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACSphereRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(SphereRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROSACSphereRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(SphereRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROSACSphereRobustEstimator)estimator);
    }
    
    private void testLocked(PROSACSphereRobustEstimator estimator){
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
