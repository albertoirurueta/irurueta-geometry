/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 14, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.Plane;
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

public class LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimatorTest 
        implements AffineTransformation3DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -1000.0;
    public static final double MAX_RANDOM_VALUE = 1000.0;
    
    public static final int INHOM_COORDS = 3;
    
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int MIN_PLANES = 500;
    public static final int MAX_PLANES = 1000;
    
    public static final double STOP_THRESHOLD = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
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
    
    public LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimatorTest() {}
    
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
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        

        
        //test constructor with planes
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                inputPlanes, outputPlanes);
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<Plane>();
        estimator = null;
        try{
            //not enough planes
            estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        //test constructor with listener
        estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this);
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //test constructor with listener and points
        estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes);        
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                AffineTransformation2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);    
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                AffineTransformation3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            //not enough points
            estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);                
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        
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
    public void testGetSetConfidence() throws LockedException{
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
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
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator.
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
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> inputPlanes = new ArrayList<Plane>();
        List<Plane> outputPlanes = new ArrayList<Plane>();
        for(int i = 0; i < PlaneCorrespondenceAffineTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator.setPlanes(inputPlanes, outputPlanes);
        
        //check correctness
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());

        //Force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<Plane>();
        try{
            //not enough planes
            estimator.setPlanes(planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator.setPlanes(planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }    
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException{
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

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
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                AffineTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
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
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }    
    
    @Test
    public void testEstimateWithoutRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation3D.INHOM_COORDS, 
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < AffineTransformation3D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation3D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation3D transformation1 =
                    new AffineTransformation3D(A, translation);
            
            //generate random planes
            int nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            List<Plane> inputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanesWithError = new ArrayList<Plane>();
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
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                }else{
                    //inlier line (without error)
                    outputPlaneWithError = outputPlane;
                }
                
                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }
            
            LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanesWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            AffineTransformation3D transformation2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input planes
            //using estimated transformation (transformation2) and checking
            //that output planes are equal to the original output planes without
            //error
            Plane p1, p2;
            for(int i = 0; i < nPlanes; i++){
                p1 = outputPlanes.get(i);
                p2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                p1.normalize();
                p2.normalize();
                assertEquals(
                        PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                        getResidual(p1, p2), 0.0, 10.0*ABSOLUTE_ERROR);
                assertTrue(p1.equals(p2, 10.0*ABSOLUTE_ERROR));
            }
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws WrongSizeException, 
            DecomposerException, LockedException, NotReadyException, 
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //create an affine transformation
            Matrix A = null;
            do{
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        AffineTransformation3D.INHOM_COORDS, 
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            }while(Utils.rank(A) < AffineTransformation3D.INHOM_COORDS);
            
            double[] translation = new double[
                    AffineTransformation3D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            AffineTransformation3D transformation1 =
                    new AffineTransformation3D(A, translation);
            
            //generate random planes
            int nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            List<Plane> inputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanes = new ArrayList<Plane>();
            List<Plane> outputPlanesWithError = new ArrayList<Plane>();
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
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(
                            outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                }else{
                    //inlier line (without error)
                    outputPlaneWithError = outputPlane;
                }
                
                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }
            
            LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanesWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            AffineTransformation3D transformation2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    AffineTransformation3D.INHOM_COORDS*
                    AffineTransformation3D.INHOM_COORDS +
                    AffineTransformation3D.NUM_TRANSLATION_COORDS);
            assertEquals(estimator.getCovariance().getColumns(),
                    AffineTransformation3D.INHOM_COORDS*
                    AffineTransformation3D.INHOM_COORDS +
                    AffineTransformation3D.NUM_TRANSLATION_COORDS);

            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input planes
            //using estimated transformation (transformation2) and checking
            //that output planes are equal to the original output planes without
            //error
            Plane p1, p2;
            for(int i = 0; i < nPlanes; i++){
                p1 = outputPlanes.get(i);
                p2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                p1.normalize();
                p2.normalize();
                assertEquals(
                        PlaneCorrespondenceAffineTransformation3DRobustEstimator.
                        getResidual(p1, p2), 0.0, 10.0*ABSOLUTE_ERROR);
                assertTrue(p1.equals(p2, 10.0*ABSOLUTE_ERROR));
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
    public void onEstimateStart(AffineTransformation3DRobustEstimator estimator) {
        estimateStart++;
        testLocked((LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(
            AffineTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            AffineTransformation3DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            AffineTransformation3DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator)estimator);
    }
    
    private void testLocked(
            LMedSPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator){
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
            estimator.setStopThreshold(0.5);
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
