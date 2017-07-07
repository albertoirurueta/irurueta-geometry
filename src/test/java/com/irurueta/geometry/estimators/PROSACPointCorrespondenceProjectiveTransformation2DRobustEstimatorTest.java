/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 4, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
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

public class PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimatorTest 
        implements ProjectiveTransformation2DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 5e-5;
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
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
    
    public PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimatorTest() {}
    
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
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        

        
        //test constructor with points
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                inputPoints, outputPoints);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        List<Point2D> pointsEmpty = new ArrayList<Point2D>();
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        
        //test constructor with listener
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        

        
        //test constructor with listener and points
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, inputPoints, outputPoints);        
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertNull(estimator.getQualityScores());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);    
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);  
        
        
        //test constructor with quality scores
        double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        double[] shortQualityScores = new double[1];
        
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                qualityScores);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);   
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test constructor with points and quality scores
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                inputPoints, outputPoints, qualityScores);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    pointsEmpty, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    inputPoints, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //not enough scores
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    inputPoints, outputPoints, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        
        //test constructor with listener and quality scores
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, qualityScores);
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);  
        
        
        //test constructor with listener, points and quality scores
        estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, inputPoints, outputPoints, qualityScores);        
        
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);    
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                ProjectiveTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, inputPoints, pointsEmpty, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //not enough scores
            estimator = new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                    this, inputPoints, outputPoints, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
        assertNull(estimator);          
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
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
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
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
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
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
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator.
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
    public void testGetSetPointsAndIsReady() throws LockedException{
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point2D> inputPoints = new ArrayList<Point2D>();
        List<Point2D> outputPoints = new ArrayList<Point2D>();
        for(int i = 0; i < PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point2D.create());
            outputPoints.add(Point2D.create());
        }
        
        estimator.setPoints(inputPoints, outputPoints);
        
        //check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
        double[] qualityScores = new double[
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        assertTrue(estimator.isReady());

        //Force IllegalArgumentException
        List<Point2D> pointsEmpty = new ArrayList<Point2D>();
        try{
            //not enough points
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException{
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

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
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
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
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        
        //set new value
        estimator.setComputeAndKeepInliersEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //set new value
        estimator.setComputeAndKeepResidualsEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }    
    
    @Test
    public void testEstimateWithoutRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException {
        for(int t = 0; t < TIMES; t++){
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation2D.INHOM_COORDS, 
                        ProjectiveTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < ProjectiveTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    ProjectiveTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            ProjectiveTransformation2D transformation1 =
                    new ProjectiveTransformation2D(A, translation);
            
            //generate random points
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point2D> inputPoints = new ArrayList<Point2D>();
            List<Point2D> outputPoints = new ArrayList<Point2D>();
            List<Point2D> outputPointsWithError = new ArrayList<Point2D>();
            double[] qualityScores = new double[nPoints];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nPoints; i++){
                Point2D inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Point2D outputPoint = transformation1.transformAndReturnNew(inputPoint);
                Point2D outputPointWithError;
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY);
                    
                    double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, inputPoints, outputPointsWithError, qualityScores);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            ProjectiveTransformation2D transformation2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input points
            //using estimated transformation (transformation2) and checking
            //that output points are equal to the original output points without
            //error
            Point2D p1, p2;
            for(int i = 0; i < nPoints; i++){
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
        }
    }

    @Test
    public void testEstimateWithRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException {
        for(int t = 0; t < TIMES; t++){
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation2D.INHOM_COORDS, 
                        ProjectiveTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < ProjectiveTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    ProjectiveTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            ProjectiveTransformation2D transformation1 =
                    new ProjectiveTransformation2D(A, translation);
            
            //generate random points
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point2D> inputPoints = new ArrayList<Point2D>();
            List<Point2D> outputPoints = new ArrayList<Point2D>();
            List<Point2D> outputPointsWithError = new ArrayList<Point2D>();
            double[] qualityScores = new double[nPoints];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nPoints; i++){
                Point2D inputPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Point2D outputPoint = transformation1.transformAndReturnNew(inputPoint);
                Point2D outputPointWithError;
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR,
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint2D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY);
                    
                    double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                new PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                this, inputPoints, outputPointsWithError, qualityScores);
            
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
            
            ProjectiveTransformation2D transformation2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        ProjectiveTransformation2D.HOM_COORDS *
                        ProjectiveTransformation2D.HOM_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        ProjectiveTransformation2D.HOM_COORDS *
                        ProjectiveTransformation2D.HOM_COORDS);
            }
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input points
            //using estimated transformation (transformation2) and checking
            //that output points are equal to the original output points without
            //error
            Point2D p1, p2;
            for(int i = 0; i < nPoints; i++){
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration = 
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(ProjectiveTransformation2DRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(
            ProjectiveTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            ProjectiveTransformation2DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            ProjectiveTransformation2DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)estimator);
    }
    
    private void testLocked(
            PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator estimator){
        List<Point2D> points = new ArrayList<Point2D>();
        try{
            estimator.setPoints(points, points);
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
            double[] qualityScores = new double[
                    PointCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE];
            estimator.setQualityScores(qualityScores);
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
