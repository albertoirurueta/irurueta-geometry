/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACPoint3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 2, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
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

public class PROSACPoint3DRobustEstimatorTest implements 
        Point3DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int MIN_LINES = 500;
    public static final int MAX_LINES = 1000;
    
    public static final double THRESHOLD = 1.0;
    
    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;    
    
    public static final double STD_ERROR = 100.0;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public PROSACPoint3DRobustEstimatorTest() {}
    
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
        PROSACPoint3DRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROSACPoint3DRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //test constructor with planes
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++){
            planes.add(new Plane());
        }
        
        estimator = new PROSACPoint3DRobustEstimator(planes);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<Plane>();
        estimator = null;
        try{
            estimator = new PROSACPoint3DRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        Point3DRobustEstimatorListener listener =
                new Point3DRobustEstimatorListener(){

            @Override
            public void onEstimateStart(Point3DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(Point3DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    Point3DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    Point3DRobustEstimator estimator, float progress) {}
        };
        
        estimator = new PROSACPoint3DRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //test constructor with listener and lines
        estimator = new PROSACPoint3DRobustEstimator(listener, planes);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPoint3DRobustEstimator(listener, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with quality scores
        double[] qualityScores = new double[
                Point3DRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACPoint3DRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try{
            estimator = new PROSACPoint3DRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with lines and scores
        estimator = new PROSACPoint3DRobustEstimator(planes, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPoint3DRobustEstimator(emptyPlanes,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACPoint3DRobustEstimator(planes, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROSACPoint3DRobustEstimator(listener, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPoint3DRobustEstimator(listener, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener, planes and quality scores
        estimator = new PROSACPoint3DRobustEstimator(listener, planes, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPoint3DRobustEstimator(listener, emptyPlanes,
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACPoint3DRobustEstimator(listener, planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                PROSACPoint3DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
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
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
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
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
    public void testGetSetPlanes() throws LockedException{
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> planes = new ArrayList<Plane>();
        for(int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++){
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);
        
        //check correctness
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
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
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[
                Point3DRobustEstimator.MINIMUM_SIZE];
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
    public void testIsSetResultRefined() throws LockedException {
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testGetSetRefinementCoordinatesType() throws LockedException {
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        
        //set new value
        estimator.setRefinementCoordinatesType(
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        
        //check correctness
        assertEquals(estimator.getRefinementCoordinatesType(),
                CoordinatesType.HOMOGENEOUS_COORDINATES);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        
        //set new value
        estimator.setComputeAndKeepInliersEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        PROSACPoint3DRobustEstimator estimator =
                new PROSACPoint3DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //set new value
        estimator.setComputeAndKeepResidualsEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }
    
    @Test
    public void testEstimateWithoutRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            ColinearPointsException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for(int t = 0; t < TIMES; t++){
            Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random lines passing through the point
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<Plane>();
            List<Plane> planesWithError = new ArrayList<Plane>();
            Plane plane, planeWithError;
            for(int i = 0; i < nPlanes; i++){
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                
                //get two more points(far enough to compute a plane)
                Point3D point2, point3;
                do{
                    point2 = new HomogeneousPoint3D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),                            
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(point2.distanceTo(point) < STD_ERROR);
                do{
                    point3 = new HomogeneousPoint3D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),                            
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(point3.distanceTo(point) < STD_ERROR ||
                        point3.distanceTo(point2) < STD_ERROR);
                
                plane = new Plane(point, point2, point3);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier plane
                    planeWithError = plane;
                }
                
                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            PROSACPoint3DRobustEstimator estimator =
                    new PROSACPoint3DRobustEstimator(this, planesWithError,
                    qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point3D point2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes without
            //error have estimated point as locus
            for(Plane p : planes){
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            if (point.distanceTo(point2) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithInhomogeneousRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            ColinearPointsException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random lines passing through the point
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<Plane>();
            List<Plane> planesWithError = new ArrayList<Plane>();
            Plane plane, planeWithError;
            for(int i = 0; i < nPlanes; i++){
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                
                //get two more points(far enough to compute a plane)
                Point3D point2, point3;
                do{
                    point2 = new HomogeneousPoint3D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),                            
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(point2.distanceTo(point) < STD_ERROR);
                do{
                    point3 = new HomogeneousPoint3D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),                            
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(point3.distanceTo(point) < STD_ERROR ||
                        point3.distanceTo(point2) < STD_ERROR);
                
                plane = new Plane(point, point2, point3);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier plane
                    planeWithError = plane;
                }
                
                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            PROSACPoint3DRobustEstimator estimator =
                    new PROSACPoint3DRobustEstimator(this, planesWithError,
                    qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point3D point2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes without
            //error have estimated point as locus
            for(Plane p : planes){
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithHomogeneousRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            ColinearPointsException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random lines passing through the point
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<Plane>();
            List<Plane> planesWithError = new ArrayList<Plane>();
            Plane plane, planeWithError;
            for(int i = 0; i < nPlanes; i++){
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                
                //get two more points(far enough to compute a plane)
                Point3D point2, point3;
                do{
                    point2 = new HomogeneousPoint3D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),                            
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(point2.distanceTo(point) < STD_ERROR);
                do{
                    point3 = new HomogeneousPoint3D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),                            
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(point3.distanceTo(point) < STD_ERROR ||
                        point3.distanceTo(point2) < STD_ERROR);
                
                plane = new Plane(point, point2, point3);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                            plane.getB() + errorB, plane.getC() + errorC,
                            plane.getD() + errorD);
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier plane
                    planeWithError = plane;
                }
                
                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            PROSACPoint3DRobustEstimator estimator =
                    new PROSACPoint3DRobustEstimator(this, planesWithError,
                    qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point3D point2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes without
            //error have estimated point as locus
            for(Plane p : planes){
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    @Override
    public void onEstimateStart(Point3DRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACPoint3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(Point3DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACPoint3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(Point3DRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROSACPoint3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(Point3DRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROSACPoint3DRobustEstimator)estimator);
    }
    
    private void testLocked(PROSACPoint3DRobustEstimator estimator){
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setPlanes(null);
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
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setComputeAndKeepInliersEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setComputeAndKeepResidualsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        assertTrue(estimator.isLocked());
    }
}
