/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 7, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
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

public class PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest 
        implements ProjectiveTransformation3DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -1000.0;
    public static final double MAX_RANDOM_VALUE = 1000.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int MIN_LINES = 500;
    public static final int MAX_LINES = 1000;
    
    public static final double THRESHOLD = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;    
    
    public static final double MIN_CONFIDENCE = 0.95;
    public static final double MAX_CONFIDENCE = 0.99;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest() {}
    
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
        //test constructor without arguments
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        

        
        //test constructor with points
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                inputPlanes, outputPlanes);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<Plane>();
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        
        //test constructor with listener
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        

        
        //test constructor with listener and points
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes);        
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);    
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);   
        
        
        //test constructor with quality scores
        double[] qualityScores = new double[
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        double[] shortQualityScores = new double[1];
        
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                qualityScores);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with planes and quality scores
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                inputPlanes, outputPlanes, qualityScores);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough planes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //not enough scores
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, outputPlanes, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        
        //test constructor with listener and quality scores
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, qualityScores);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test constructor with listener and planes
        estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes, qualityScores);        
        
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);    
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //not enough scores
            estimator = new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, outputPlanes, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);           
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        
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
    public void testGetSetQualityScores() throws LockedException{
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
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
    public void testGetSetConfidence() throws LockedException{
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        
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
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try{
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetPlanesAndIsReady() throws LockedException{
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator.setPlanes(inputPlanes, outputPlanes);
        
        //check correctness
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
        double[] qualityScores = new double[
                PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        assertTrue(estimator.isReady());

        //Force IllegalArgumentException
        List<Plane> pointsEmpty = new ArrayList<Plane>();
        try{
            //not enough lines
            estimator.setPlanes(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator.setPlanes(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }    
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException{
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException{
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
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
    public void testIsSetResultRefined() throws LockedException {
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        
        //set new value
        estimator.setComputeAndKeepInliersEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //set new value
        estimator.setComputeAndKeepResidualsEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }    
    
    @Test
    public void testEstimateWithoutRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for(int t = 0; t < TIMES; t++){
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation3D.INHOM_COORDS, 
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < ProjectiveTransformation3D.INHOM_COORDS);
            
            double[] translation = new double[
                    ProjectiveTransformation3D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            ProjectiveTransformation3D transformation1 =
                    new ProjectiveTransformation3D(A, translation);
            
            //generate random planes
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            List<Plane> inputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanesWithError = new ArrayList<Plane>();
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nPlanes; i++){
                Plane inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Plane outputPlane = transformation1.transformAndReturnNew(inputPlane);
                Plane outputPlaneWithError;
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //plane is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                        outputPlane.getD() + errorD);
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }
                
                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }
            
            PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanesWithError, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            ProjectiveTransformation3D transformation2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input planes
            //using estimated transformation (transformation2) and checking
            //that output planes are equal to the original output planes without
            //error
            boolean failed = false;
            Plane plane1, plane2;
            for(int i = 0; i < nPlanes; i++){
                plane1 = outputPlanes.get(i);
                plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2), 0.0, ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for(int t = 0; t < TIMES; t++){
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation3D.INHOM_COORDS, 
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < ProjectiveTransformation3D.INHOM_COORDS);
            
            double[] translation = new double[
                    ProjectiveTransformation3D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            ProjectiveTransformation3D transformation1 =
                    new ProjectiveTransformation3D(A, translation);
            
            //generate random planes
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            List<Plane> inputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanesWithError = new ArrayList<Plane>();
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nPlanes; i++){
                Plane inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Plane outputPlane = transformation1.transformAndReturnNew(inputPlane);
                Plane outputPlaneWithError;
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //plane is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                        outputPlane.getD() + errorD);
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC + errorD * errorD);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }
                
                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }
            
            PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanesWithError, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            ProjectiveTransformation3D transformation2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        ProjectiveTransformation3D.HOM_COORDS *
                        ProjectiveTransformation3D.HOM_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        ProjectiveTransformation3D.HOM_COORDS *
                        ProjectiveTransformation3D.HOM_COORDS);
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input planes
            //using estimated transformation (transformation2) and checking
            //that output planes are equal to the original output planes without
            //error
            Plane plane1, plane2;
            boolean failed = false;
            for(int i = 0; i < nPlanes; i++){
                plane1 = outputPlanes.get(i);
                plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (Math.abs(PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2)) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2), 0.0, ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration = 
                estimateProgressChange = 0;
    }    
    
    @Override
    public void onEstimateStart(ProjectiveTransformation3DRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(
            ProjectiveTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            ProjectiveTransformation3DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            ProjectiveTransformation3DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }
    
    private void testLocked(
            PROSACPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator){
        List<Plane> planes = new ArrayList<Plane>();
        try{
            estimator.setPlanes(planes, planes);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setConfidence(0.5);            
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setMaxIterations(10);
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
