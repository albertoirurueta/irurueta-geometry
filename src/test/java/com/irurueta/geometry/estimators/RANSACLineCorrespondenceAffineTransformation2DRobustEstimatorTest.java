/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.RANSACPointCorrespondenceAffineTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 12, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.Line2D;
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

public class RANSACLineCorrespondenceAffineTransformation2DRobustEstimatorTest 
        implements AffineTransformation2DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -1000.0;
    public static final double MAX_RANDOM_VALUE = 1000.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int MIN_LINES = 500;
    public static final int MAX_LINES = 1000;
    
    public static final double THRESHOLD = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
    public static final double MIN_CONFIDENCE = 0.95;
    public static final double MAX_CONFIDENCE = 0.99;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public RANSACLineCorrespondenceAffineTransformation2DRobustEstimatorTest() {}
    
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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        
        assertEquals(estimator.getThreshold(), 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //test constructor with points
        List<Line2D> inputLines = new ArrayList<Line2D>();
        List<Line2D> outputLines = new ArrayList<Line2D>();
        for(int i = 0; i < LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }
        
        estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                inputLines, outputLines);
        
        assertEquals(estimator.getThreshold(), 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        List<Line2D> linesEmpty = new ArrayList<Line2D>();
        estimator = null;
        try{
            //not enough points
            estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    inputLines, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        //test constructor with listener
        estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                this);
        
        assertEquals(estimator.getThreshold(), 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        

        
        //test constructor with listener and points
        estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputLines, outputLines);        
        
        assertEquals(estimator.getThreshold(), 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);    
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                    this, inputLines, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);                
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
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
    public void testGetSetConfidence() throws LockedException{
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                RANSACLineCorrespondenceAffineTransformation2DRobustEstimator.
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
    public void testGetSetLinesAndIsReady() throws LockedException{
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputLines());
        assertNull(estimator.getOutputLines());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Line2D> inputLines = new ArrayList<Line2D>();
        List<Line2D> outputLines = new ArrayList<Line2D>();
        for(int i = 0; i < LineCorrespondenceAffineTransformation2DRobustEstimator.MINIMUM_SIZE; i++){
            inputLines.add(new Line2D());
            outputLines.add(new Line2D());
        }
        
        estimator.setLines(inputLines, outputLines);
        
        //check correctness
        assertSame(estimator.getInputLines(), inputLines);
        assertSame(estimator.getOutputLines(), outputLines);
        assertTrue(estimator.isReady());

        //Force IllegalArgumentException
        List<Line2D> linesEmpty = new ArrayList<Line2D>();
        try{
            //not enough lines
            estimator.setLines(linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator.setLines(linesEmpty, linesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }    
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException{
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();

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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        
        //set new value
        estimator.setComputeAndKeepInliersEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //set new value
        estimator.setComputeAndKeepResidualsEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }    
    
    @Test
    public void testEstimateWithoutRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException, AlgebraException{
        for(int t = 0; t < TIMES; t++){
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation2D.INHOM_COORDS, 
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation2D transformation1 =
                    new AffineTransformation2D(A, translation);
            
            //generate random lines
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            List<Line2D> inputLines = new ArrayList<Line2D>();
            List<Line2D> outputLines = new ArrayList<Line2D>();
            List<Line2D> outputLinesWithError = new ArrayList<Line2D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nLines; i++){
                Line2D inputLine = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Line2D outputLine = transformation1.transformAndReturnNew(inputLine);
                Line2D outputLineWithError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    outputLineWithError = new Line2D(outputLine.getA() + errorA,
                            outputLine.getB() + errorB,
                            outputLine.getC() + errorC);
                }else{
                    //inlier line (without error)
                    outputLineWithError = outputLine;
                }
                
                inputLines.add(inputLine);
                outputLines.add(outputLine);
                outputLinesWithError.add(outputLineWithError);
            }
            
            RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputLines, outputLinesWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            AffineTransformation2D transformation2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input lines
            //using estimated transformation (transformation2) and checking
            //that output lines are equal to the original output lines without
            //error
            Line2D l1, l2;
            for(int i = 0; i < nLines; i++){
                l1 = outputLines.get(i);
                l2 = transformation2.transformAndReturnNew(inputLines.get(i));
                l1.normalize();
                l2.normalize();
                assertEquals(
                        LineCorrespondenceAffineTransformation2DRobustEstimator.
                        getResidual(l1, l2), 0.0, ABSOLUTE_ERROR);
                assertTrue(l1.equals(l2, ABSOLUTE_ERROR));
            }
        }
    }

    @Test
    public void testEstimateWithRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException, AlgebraException{
        for(int t = 0; t < TIMES; t++){
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation2D.INHOM_COORDS, 
                        AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation2D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation2D transformation1 =
                    new AffineTransformation2D(A, translation);
            
            //generate random lines
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            List<Line2D> inputLines = new ArrayList<Line2D>();
            List<Line2D> outputLines = new ArrayList<Line2D>();
            List<Line2D> outputLinesWithError = new ArrayList<Line2D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nLines; i++){
                Line2D inputLine = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Line2D outputLine = transformation1.transformAndReturnNew(inputLine);
                Line2D outputLineWithError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    outputLineWithError = new Line2D(outputLine.getA() + errorA,
                            outputLine.getB() + errorB,
                            outputLine.getC() + errorC);
                }else{
                    //inlier line (without error)
                    outputLineWithError = outputLine;
                }
                
                inputLines.add(inputLine);
                outputLines.add(outputLine);
                outputLinesWithError.add(outputLineWithError);
            }
            
            RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                this, inputLines, outputLinesWithError);
            
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
            
            AffineTransformation2D transformation2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    AffineTransformation2D.INHOM_COORDS*
                    AffineTransformation2D.INHOM_COORDS +
                    AffineTransformation2D.NUM_TRANSLATION_COORDS);
            assertEquals(estimator.getCovariance().getColumns(),
                    AffineTransformation2D.INHOM_COORDS*
                    AffineTransformation2D.INHOM_COORDS +
                    AffineTransformation2D.NUM_TRANSLATION_COORDS);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input lines
            //using estimated transformation (transformation2) and checking
            //that output lines are equal to the original output lines without
            //error
            Line2D l1, l2;
            for(int i = 0; i < nLines; i++){
                l1 = outputLines.get(i);
                l2 = transformation2.transformAndReturnNew(inputLines.get(i));
                l1.normalize();
                l2.normalize();
                assertEquals(
                        LineCorrespondenceAffineTransformation2DRobustEstimator.
                        getResidual(l1, l2), 0.0, ABSOLUTE_ERROR);
                assertTrue(l1.equals(l2, ABSOLUTE_ERROR));
            }
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration = 
                estimateProgressChange = 0;
    }    
    
    @Override
    public void onEstimateStart(AffineTransformation2DRobustEstimator estimator) {
        estimateStart++;
        testLocked((RANSACLineCorrespondenceAffineTransformation2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(
            AffineTransformation2DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((RANSACLineCorrespondenceAffineTransformation2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            AffineTransformation2DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((RANSACLineCorrespondenceAffineTransformation2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            AffineTransformation2DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((RANSACLineCorrespondenceAffineTransformation2DRobustEstimator)estimator);
    }
    
    private void testLocked(
            RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator){
        List<Line2D> lines = new ArrayList<Line2D>();
        try{
            estimator.setLines(lines, lines);
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
