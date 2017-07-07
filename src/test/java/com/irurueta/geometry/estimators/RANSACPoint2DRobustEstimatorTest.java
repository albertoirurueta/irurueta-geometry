/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.RANSACPoint2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 28, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
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

public class RANSACPoint2DRobustEstimatorTest implements 
        Point2DRobustEstimatorListener{

    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int MIN_LINES = 500;
    public static final int MAX_LINES = 1000;
    
    public static final double THRESHOLD = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 10;
    
    public static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public RANSACPoint2DRobustEstimatorTest() {}
    
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
        RANSACPoint2DRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new RANSACPoint2DRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
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
        
        //test constructor with lines
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++){
            lines.add(new Line2D());
        }
        
        estimator = new RANSACPoint2DRobustEstimator(lines);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
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
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        estimator = null;
        try{
            estimator = new RANSACPoint2DRobustEstimator(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        Point2DRobustEstimatorListener listener =
                new Point2DRobustEstimatorListener(){

            @Override
            public void onEstimateStart(Point2DRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(Point2DRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    Point2DRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    Point2DRobustEstimator estimator, float progress) {}
        };
        
        estimator = new RANSACPoint2DRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
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
        estimator = new RANSACPoint2DRobustEstimator(listener, lines);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
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
            estimator = new RANSACPoint2DRobustEstimator(listener, emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                RANSACPoint2DRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
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
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
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
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                Point2DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                Point2DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                Point2DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
    public void testGetSetLines() throws LockedException{
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < Point2DRobustEstimator.MINIMUM_SIZE; i++){
            lines.add(new Line2D());
        }
        estimator.setLines(lines);
        
        //check correctness
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        lines.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        try{
            estimator.setLines(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException{
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[
                Point2DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testIsSetResultRefined() throws LockedException {
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testGetSetRefinementCoordinatesType() throws LockedException {
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
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
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        
        //set new value
        estimator.setComputeAndKeepInliersEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }
    
    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() 
            throws LockedException {
        RANSACPoint2DRobustEstimator estimator =
                new RANSACPoint2DRobustEstimator();
        
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        
        //set new value
        estimator.setComputeAndKeepResidualsEnabled(true);
        
        //check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }
    
    @Test
    public void testEstimateWithoutRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            Point2D point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random lines passing through the point
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Line2D> lines = new ArrayList<Line2D>();
            List<Line2D> linesWithError = new ArrayList<Line2D>();
            Line2D line, lineWithError;
            for(int i = 0; i < nLines; i++){
                //get another point (far enough to compute a line)
                Point2D anotherPoint;
                do{
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(anotherPoint.distanceTo(point) < STD_ERROR);
                
                line = new Line2D(point, anotherPoint);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                            line.getB() + errorB, line.getC() + errorC);
                }else{
                    //inlier line
                    lineWithError = line;
                }
                
                lines.add(line);
                linesWithError.add(lineWithError);
                
                //check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }
            
            RANSACPoint2DRobustEstimator estimator =
                    new RANSACPoint2DRobustEstimator(this, linesWithError);
            
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point2D point2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all lines without
            //error have estimated point as locus
            for(Line2D l : lines){
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithInhomogeneousRefinement() 
            throws LockedException, NotReadyException, 
            RobustEstimatorException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            Point2D point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random lines passing through the point
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Line2D> lines = new ArrayList<Line2D>();
            List<Line2D> linesWithError = new ArrayList<Line2D>();
            Line2D line, lineWithError;
            for(int i = 0; i < nLines; i++){
                //get another point (far enough to compute a line)
                Point2D anotherPoint;
                do{
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(anotherPoint.distanceTo(point) < STD_ERROR);
                
                line = new Line2D(point, anotherPoint);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                            line.getB() + errorB, line.getC() + errorC);
                }else{
                    //inlier line
                    lineWithError = line;
                }
                
                lines.add(line);
                linesWithError.add(lineWithError);
                
                //check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }
            
            RANSACPoint2DRobustEstimator estimator =
                    new RANSACPoint2DRobustEstimator(this, linesWithError);
            
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
            
            Point2D point2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all lines without
            //error have estimated point as locus
            for(Line2D l : lines){
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithHomogeneousRefinement() 
            throws LockedException, NotReadyException, 
            RobustEstimatorException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            Point2D point = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random lines passing through the point
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Line2D> lines = new ArrayList<Line2D>();
            List<Line2D> linesWithError = new ArrayList<Line2D>();
            Line2D line, lineWithError;
            for(int i = 0; i < nLines; i++){
                //get another point (far enough to compute a line)
                Point2D anotherPoint;
                do{
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                }while(anotherPoint.distanceTo(point) < STD_ERROR);
                
                line = new Line2D(point, anotherPoint);
                
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                            line.getB() + errorB, line.getC() + errorC);
                }else{
                    //inlier line
                    lineWithError = line;
                }
                
                lines.add(line);
                linesWithError.add(lineWithError);
                
                //check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
            }
            
            RANSACPoint2DRobustEstimator estimator =
                    new RANSACPoint2DRobustEstimator(this, linesWithError);
            
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
            
            Point2D point2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all lines without
            //error have estimated point as locus
            for(Line2D l : lines){
                assertTrue(l.isLocus(point2, ABSOLUTE_ERROR));
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
    public void onEstimateStart(Point2DRobustEstimator estimator) {
        estimateStart++;
        testLocked((RANSACPoint2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(Point2DRobustEstimator estimator) {
        estimateEnd++;
        testLocked((RANSACPoint2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(Point2DRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((RANSACPoint2DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(Point2DRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((RANSACPoint2DRobustEstimator)estimator);
    }
    
    private void testLocked(RANSACPoint2DRobustEstimator estimator){
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
            estimator.setLines(null);
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
