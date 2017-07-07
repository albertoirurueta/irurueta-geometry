/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.MSACEuclideanTransformation3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 26, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.EuclideanTransformation3D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Utils;
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

public class MSACEuclideanTransformation3DRobustEstimatorTest implements 
        EuclideanTransformation3DRobustEstimatorListener {
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double MIN_ANGLE_DEGREES = -90.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;
    
    public static final double MIN_TRANSLATION = -100.0;
    public static final double MAX_TRANSLATION = 100.0;
    
    public static final double THRESHOLD = 1.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
    public static final int TIMES = 10;
    
    public static final int MIN_POINTS = 500;
    public static final int MAX_POINTS = 1000;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public MSACEuclideanTransformation3DRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

   @Test
    public void testConstructor() {
        //test constructor without arguments
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE);        
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        
        //test constructor with points
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = new MSACEuclideanTransformation3DRobustEstimator(
                inputPoints, outputPoints);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE);        
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        List<Point3D> pointsEmpty = new ArrayList<Point3D>();
        estimator = null;
        try {
            //not enough points
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);

        //test constructor with listener
        estimator = new MSACEuclideanTransformation3DRobustEstimator(this);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE);        
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        
        //test constructor with listener and points
        estimator = new MSACEuclideanTransformation3DRobustEstimator(
                this, inputPoints, outputPoints);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE);                
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    this, pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    this, inputPoints, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
        
        
        //test constructor without arguments and weak minimum points
        estimator = new MSACEuclideanTransformation3DRobustEstimator(true);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        
        //test constructor with points
        inputPoints = new ArrayList<Point3D>();
        outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE; i++) {
            inputPoints.add(Point3D.create());
            outputPoints.add(Point3D.create());
        }
        
        estimator = new MSACEuclideanTransformation3DRobustEstimator(
                inputPoints, outputPoints, true);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
         assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
       
        //Force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    pointsEmpty, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    inputPoints, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);

        //test constructor with listener
        estimator = new MSACEuclideanTransformation3DRobustEstimator(this, 
                true);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);        
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        
        //test constructor with listener and points
        estimator = new MSACEuclideanTransformation3DRobustEstimator(
                this, inputPoints, outputPoints, true);
        
        assertEquals(estimator.getThreshold(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                        DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getInputPoints(), inputPoints);
        assertSame(estimator.getOutputPoints(), outputPoints);
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                EuclideanTransformation2DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    this, pointsEmpty, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator = new MSACEuclideanTransformation3DRobustEstimator(
                    this, inputPoints, pointsEmpty, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);                
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                MSACEuclideanTransformation3DRobustEstimator.
                DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }   
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                MSACEuclideanTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }   
    
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                MSACEuclideanTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }    

    @Test
    public void testGetSetPointsAndIsReady() throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputPoints());
        assertNull(estimator.getOutputPoints());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPoints = new ArrayList<Point3D>();
        for (int i = 0; i < MSACEuclideanTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
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
        try {
            //not enough points
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            //different sizes
            estimator.setPoints(pointsEmpty, pointsEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }     
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() 
            throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();

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
    public void testIsSetWeakMinimumPointsAllowed() throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();

        //check default value
        assertFalse(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.MINIMUM_SIZE);
        
        //set new value
        estimator.setWeakMinimumSizeAllowed(true);
        
        //check correctness
        assertTrue(estimator.isWeakMinimumSizeAllowed());
        assertEquals(estimator.getMinimumPoints(),
                EuclideanTransformation3DRobustEstimator.WEAK_MINIMUM_SIZE);
    }    
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                EuclideanTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }     

    @Test
    public void testIsSetResultRefined() throws LockedException {
        MSACEuclideanTransformation2DRobustEstimator estimator =
                new MSACEuclideanTransformation2DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        MSACEuclideanTransformation2DRobustEstimator estimator =
                new MSACEuclideanTransformation2DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }    

    @Test
    public void testEstimateWithoutRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException { 
        for (int t = 0; t < TIMES; t++) {
            //create an euclideantransformation
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
        
            EuclideanTransformation3D transformation1 = 
                    new EuclideanTransformation3D(q, translation);
            
                        
            //generate random points
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPoints; i++) {
                Point3D inputPoint = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE),                        
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE));
                Point3D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                } else {
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator(this, 
                        inputPoints, outputPointsWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            EuclideanTransformation3D transformation2 = estimator.estimate();
            
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
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
            
            //check paramaters of estimated transformation
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        }
    }    
    
    @Test
    public void testEstimateCoplanarWithoutRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException { 
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //create an euclideantransformation
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
        
            EuclideanTransformation3D transformation1 = 
                    new EuclideanTransformation3D(q, translation);
            
                        
            //generate random points
            
            //generate random plane
            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double d = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Plane plane = new Plane(a, b, c, d);
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            HomogeneousPoint3D inputPoint;
            for (int i = 0; i < nPoints; i++) {
                double homX, homY;
                double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(inputPoint));

                Point3D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                } else {
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator(this, 
                        inputPoints, outputPointsWithError, true);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            EuclideanTransformation3D transformation2 = estimator.estimate();
            
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
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }                
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
            
            if (!isValid) continue;
            
            //check paramaters of estimated transformation
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }    

    @Test
    public void testEstimateWithRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException { 
        for (int t = 0; t < TIMES; t++) {
            //create an euclideantransformation
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
        
            EuclideanTransformation3D transformation1 = 
                    new EuclideanTransformation3D(q, translation);
            
                        
            //generate random points
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPoints; i++) {
                Point3D inputPoint = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE),                        
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                                MAX_RANDOM_VALUE));
                Point3D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                } else {
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator(this, 
                        inputPoints, outputPointsWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            EuclideanTransformation3D transformation2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if(estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        Quaternion.N_PARAMS + 
                        EuclideanTransformation3D.NUM_TRANSLATION_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        Quaternion.N_PARAMS + 
                        EuclideanTransformation3D.NUM_TRANSLATION_COORDS);
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
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
            
            //check paramaters of estimated transformation
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        }
    }    
    
    @Test
    public void testEstimateCoplanarWithRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException { 
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //create an euclideantransformation
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        
            Quaternion q = new Quaternion(roll, pitch, yaw);
            q.normalize();
        
            double[] translation = new double[3];
            randomizer.fill(translation, MIN_TRANSLATION, MAX_TRANSLATION);
        
            EuclideanTransformation3D transformation1 = 
                    new EuclideanTransformation3D(q, translation);
            
                        
            //generate random points
            
            //generate random plane
            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double d = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Plane plane = new Plane(a, b, c, d);
            
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> inputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPoints = new ArrayList<Point3D>();
            List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            HomogeneousPoint3D inputPoint;
            for (int i = 0; i < nPoints; i++) {
                double homX, homY;
                double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                inputPoint = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(inputPoint));

                Point3D outputPoint = transformation1.transformAndReturnNew(
                        inputPoint);
                Point3D outputPointWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    outputPointWithError = new InhomogeneousPoint3D(
                            outputPoint.getInhomX() + errorX, 
                            outputPoint.getInhomY() + errorY,
                            outputPoint.getInhomZ() + errorZ);
                } else {
                    //inlier point (without error)
                    outputPointWithError = outputPoint;
                }
                
                inputPoints.add(inputPoint);
                outputPoints.add(outputPoint);
                outputPointsWithError.add(outputPointWithError);
            }
            
            MSACEuclideanTransformation3DRobustEstimator estimator =
                new MSACEuclideanTransformation3DRobustEstimator(this, 
                        inputPoints, outputPointsWithError, true);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            EuclideanTransformation3D transformation2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if(estimator.getCovariance() != null) {
                assertEquals(estimator.getCovariance().getRows(),
                        Quaternion.N_PARAMS + 
                        EuclideanTransformation3D.NUM_TRANSLATION_COORDS);
                assertEquals(estimator.getCovariance().getColumns(),
                        Quaternion.N_PARAMS + 
                        EuclideanTransformation3D.NUM_TRANSLATION_COORDS);
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
            boolean isValid = true;
            for (int i = 0; i < nPoints; i++) {
                p1 = outputPoints.get(i);
                p2 = transformation2.transformAndReturnNew(inputPoints.get(i));
                if (p1.distanceTo(p2) > ABSOLUTE_ERROR) {
                    isValid = false;
                    break;
                }                
                assertEquals(p1.distanceTo(p2), 0.0,
                        ABSOLUTE_ERROR);
            }
            
            if (!isValid) continue;
            
            //check paramaters of estimated transformation
            Quaternion q2 = transformation2.getRotation().toQuaternion();
            q2.normalize();        
            double[] translation2 = transformation2.getTranslation();
        
            assertEquals(q.getA(), q2.getA(), ABSOLUTE_ERROR);
            assertEquals(q.getB(), q2.getB(), ABSOLUTE_ERROR);
            assertEquals(q.getC(), q2.getC(), ABSOLUTE_ERROR);
            assertEquals(q.getD(), q2.getD(), ABSOLUTE_ERROR);
            assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }    
    
    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = 
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(EuclideanTransformation3DRobustEstimator estimator) {
        estimateStart++;
        testLocked((MSACEuclideanTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(EuclideanTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((MSACEuclideanTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            EuclideanTransformation3DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((MSACEuclideanTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            EuclideanTransformation3DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((MSACEuclideanTransformation3DRobustEstimator)estimator);
    }
    
    private void testLocked(
            MSACEuclideanTransformation3DRobustEstimator estimator) {
        List<Point3D> points = new ArrayList<Point3D>();
        try {
            estimator.setPoints(points, points);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setConfidence(0.5);            
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMaxIterations(10);
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
            estimator.setWeakMinimumSizeAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }        
        assertTrue(estimator.isLocked());
    }    
}
