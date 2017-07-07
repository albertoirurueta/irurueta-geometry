/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACDualQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 24, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.DualQuadricNotAvailableException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
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

public class PROSACDualQuadricRobustEstimatorTest implements 
        DualQuadricRobustEstimatorListener{
    
    public static final int MIN_PLANES = 500;
    public static final int MAX_PLANES = 1000;
    
    public static final double MIN_RANDOM_POINT_VALUE = -1.0;
    public static final double MAX_RANDOM_POINT_VALUE = 1.0;
    
    public static final double STD_ERROR = 1.0;
    
    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double THRESHOLD = 1e-7;
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int TIMES = 10;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public PROSACDualQuadricRobustEstimatorTest() {}
    
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
        PROSACDualQuadricRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROSACDualQuadricRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with planes
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++){
            planes.add(new Plane());
        }
        
        estimator = new PROSACDualQuadricRobustEstimator(planes);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        estimator = null;
        try{
            estimator = new PROSACDualQuadricRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        DualQuadricRobustEstimatorListener listener =
                new DualQuadricRobustEstimatorListener(){

            @Override
            public void onEstimateStart(DualQuadricRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(DualQuadricRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    DualQuadricRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    DualQuadricRobustEstimator estimator, float progress) {}
        };
        
        estimator = new PROSACDualQuadricRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with listener and points
        estimator = new PROSACDualQuadricRobustEstimator(listener, planes);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualQuadricRobustEstimator(listener, 
                    emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with quality scores
        double[] qualityScores = new double[
                DualQuadricRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACDualQuadricRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try{
            estimator = new PROSACDualQuadricRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with planes and quality scores
        estimator = new PROSACDualQuadricRobustEstimator(planes, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PROSACDualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACDualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACDualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualQuadricRobustEstimator(emptyPlanes, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACDualQuadricRobustEstimator(planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROSACDualQuadricRobustEstimator(listener, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualQuadricRobustEstimator(listener, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener, points and quality scores
        estimator = new PROSACDualQuadricRobustEstimator(listener, planes, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualQuadricRobustEstimator(listener, 
                    emptyPlanes, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACDualQuadricRobustEstimator(listener, planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);                          
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                PROSACDualQuadricRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
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
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
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
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
    public void testGetSetPlanes() throws LockedException{
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
        //check default value
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++){
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);
        
        //check correctness
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        
        //if we set qualitys cores, then estimator becomes ready
        double[] qualityScores = new double[planes.size()];
        estimator.setQualityScores(qualityScores);
        
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        planes.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        try{
            estimator.setPlanes(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException{
        PROSACDualQuadricRobustEstimator estimator = 
                new PROSACDualQuadricRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = 
                new double[DualQuadricRobustEstimator.MINIMUM_SIZE];
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
            RobustEstimatorException, DualQuadricNotAvailableException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0;  t < TIMES; t++){
            //instantiate a random circle
            Point3D center = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0,
                    MAX_RANDOM_POINT_VALUE));
            
            Sphere sphere = new Sphere(center, radius);
            Quadric quadric = sphere.toQuadric();
            DualQuadric dualQuadric = quadric.getDualQuadric();
            
            //compute planes in the dual quadric locus
            int nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            int halfPoints = (int)Math.ceil((double)nPlanes / 2.0);
            double theta = (double)nPlanes / 360.0 * Math.PI / 180.0;
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<Plane>();
            List<Plane> planesWithError = new ArrayList<Plane>();
            Point3D point;
            Plane plane, planeWithError;
            double[] directorVector = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for(int i = 0; i < nPlanes; i++){
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
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                directorVector[2] = point.getInhomZ() - center.getInhomZ();
                
                plane = new Plane(point, directorVector);
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                        plane.getB() + errorB, plane.getC() + errorC, 
                        plane.getD());
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier plane
                    planeWithError = plane;
                }

                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point without error is within quadric locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(quadric.isLocus(point, ABSOLUTE_ERROR));                
                assertTrue(dualQuadric.isLocus(plane, ABSOLUTE_ERROR));
            }
    
            PROSACDualQuadricRobustEstimator estimator = 
                    new PROSACDualQuadricRobustEstimator(this, planesWithError,
                    qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            DualQuadric dualQuadric2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes
            //are within the estimated conic locus
            for(Plane p : planes){
                assertTrue(dualQuadric2.isLocus(p, ABSOLUTE_ERROR));
            }    
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(DualQuadricRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACDualQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(DualQuadricRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACDualQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(DualQuadricRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROSACDualQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(DualQuadricRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROSACDualQuadricRobustEstimator)estimator);
    }
    
    private void testLocked(PROSACDualQuadricRobustEstimator estimator){
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
            estimator.setPlanes(null);
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
