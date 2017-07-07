/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 5, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
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

public class RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimatorTest 
        implements ProjectiveTransformation3DRobustEstimatorListener{
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final double THRESHOLD = 1e-6;
    
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
    
    
    public RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimatorTest() {}
    
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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertEquals(estimator.getThreshold(), 
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
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
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for(int i = 0; i < PointCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                inputPoints, outputPoints);
        
        assertEquals(estimator.getThreshold(), 
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
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
        List<Point3D> pointsEmpty = new ArrayList<Point3D>();
        estimator = null;
        try{
            //not enough points
            estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);

        
        //test constructor with listener
        estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                this);
        
        assertEquals(estimator.getThreshold(), 
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
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
        estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPoints, outputPoints);        
        
        assertEquals(estimator.getThreshold(), 
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
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
            estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            //different sizes
            estimator = new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator.
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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for(int i = 0; i < PointCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++){
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator.setPoints(inputPoints, outputPoints);
        
        //check correctness
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());

        //Force IllegalArgumentException
        List<Point3D> pointsEmpty = new ArrayList<Point3D>();
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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();

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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();

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
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        
        //set new value
        estimator.setComputeAndKeepInliersEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator();
        
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
            
            //generate random points
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nPoints; i++){
                Point3D inputPoint = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Point3D outputPoint = transformation1.transformAndReturnNew(inputPoint);
                Point3D outputPointWithError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                }else{
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPoints, outputPointsWithError);
            
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
            
            //check correctness of estimation by transforming input points
            //using estimated transformation (transformation2) and checking
            //that output points are equal to the original output points without
            //error
            Point3D p1, p2;
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
            
            //generate random points
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for(int i = 0; i < nPoints; i++){
                Point3D inputPoint = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Point3D outputPoint = transformation1.transformAndReturnNew(inputPoint);
                Point3D outputPointWithError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                }else{
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPoints, outputPointsWithError);
            
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
            
            //check correctness of estimation by transforming input points
            //using estimated transformation (transformation2) and checking
            //that output points are equal to the original output points without
            //error
            Point3D p1, p2;
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
    public void onEstimateStart(ProjectiveTransformation3DRobustEstimator estimator) {
        estimateStart++;
        testLocked((RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(
            ProjectiveTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            ProjectiveTransformation3DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            ProjectiveTransformation3DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }
    
    private void testLocked(
            RANSACPointCorrespondenceProjectiveTransformation3DRobustEstimator estimator){
        List<Point3D> points = new ArrayList<Point3D>();
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
