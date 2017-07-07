/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.refiners.InhomogeneousPoint2DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 8, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPoint2DRobustEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InhomogeneousPoint2DRefinerTest implements
        RefinerListener<InhomogeneousPoint2D> {
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 5e-6;
    
    public static final int MIN_LINES = 500;
    public static final int MAX_LINES = 1000;
    
    public static final double THRESHOLD = 1e-6;
    
    public static final double STD_ERROR = 100.0;
    
    public static final int MIN_MAX_ITERATIONS = 500;
    public static final int MAX_MAX_ITERATIONS = 5000;
        
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final int TIMES = 100;    
    
    private int mRefineStart;
    private int mRefineEnd;    
    
    public InhomogeneousPoint2DRefinerTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws LockedException, NotReadyException, 
            ColinearPointsException, RobustEstimatorException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        InhomogeneousPoint2D point = new InhomogeneousPoint2D(
                estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        double[] residuals = inliersData.getResiduals();
        int numInliers = inliersData.getNumInliers();
        double refinementStandardDeviation = estimator.getThreshold();
        List<Line2D> samples = estimator.getLines();
        
        assertNotNull(point);
        assertNotNull(inliersData);
        
        //test empty constructor
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        assertNull(refiner.getSamples());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);
        assertEquals(refiner.getTotalSamples(), 0);
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
        
        //test non-empty constructor
        refiner = new InhomogeneousPoint2DRefiner(point, true, inliers, 
                residuals, numInliers, samples, refinementStandardDeviation);
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples(), samples);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples.size());
        assertSame(refiner.getInitialEstimation(), point);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
        
        //test non-empty constructor with InliersData
        refiner = new InhomogeneousPoint2DRefiner(point, true, inliersData, 
                samples, refinementStandardDeviation);
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples(), samples);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples.size());
        assertSame(refiner.getInitialEstimation(), point);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }
    
    @Test
    public void testGetSetListener() {
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double refinementStandardDeviation = randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);
        
        //check correctness
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
    }
    
    @Test
    public void testGetSetSamples() throws LockedException, 
            ColinearPointsException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        List<Line2D> samples = estimator.getLines();
        
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getSamples());
        
        //set new value
        refiner.setSamples(samples);
        
        //check correctness
        assertSame(refiner.getSamples(), samples);
    }
    
    @Test
    public void testGetSetInliers() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getInliers());
        
        //set new value
        refiner.setInliers(inliers);
        
        //check correctness
        assertSame(refiner.getInliers(), inliers);
    }
    
    @Test
    public void testGetSetResiduals() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        double[] residuals = inliersData.getResiduals();
        
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getResiduals());
        
        //set new value
        refiner.setResiduals(residuals);
        
        //check correctness
        assertSame(refiner.getResiduals(), residuals);
    }
    
    @Test
    public void testGetSetNumInliers() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        int numInliers = inliersData.getNumInliers();
        
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertEquals(refiner.getNumInliers(), 0);
        
        //set new value
        refiner.setNumInliers(numInliers);
        
        //check correctness
        assertEquals(refiner.getNumInliers(), numInliers);
        
        //Force IllegalArgumentException
        try {
            refiner.setNumInliers(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testSetInliersData() throws LockedException, NotReadyException,
            RobustEstimatorException, ColinearPointsException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);
        
        //set new value
        refiner.setInliersData(inliersData);
        
        //check correctness
        assertSame(refiner.getInliers(), inliersData.getInliers());
        assertSame(refiner.getResiduals(), inliersData.getResiduals());
        assertEquals(refiner.getNumInliers(), inliersData.getNumInliers());
    }
    
    @Test
    public void testGetSetInitialEstimation() throws LockedException {
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        InhomogeneousPoint2D point = new InhomogeneousPoint2D();
        refiner.setInitialEstimation(point);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), point);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        InhomogeneousPoint2DRefiner refiner = new InhomogeneousPoint2DRefiner();
        
        //check default value
        assertFalse(refiner.isCovarianceKept());
        
        //set new value
        refiner.setCovarianceKept(true);
        
        //check correctness
        assertTrue(refiner.isCovarianceKept());
    }
    
    @Test    
    public void testRefine() throws ColinearPointsException, LockedException, 
            NotReadyException, RobustEstimatorException, RefinerException {
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
            
            InhomogeneousPoint2D point = new InhomogeneousPoint2D(
                    estimator.estimate());
            InliersData inliersData = estimator.getInliersData();
            double refineStandardDeviation = estimator.getThreshold();
            List<Line2D> samples = estimator.getLines();
            
            InhomogeneousPoint2DRefiner refiner = 
                    new InhomogeneousPoint2DRefiner(point, true, inliersData,
                    samples, refineStandardDeviation);
            refiner.setListener(this);
            
            InhomogeneousPoint2D result1 = new InhomogeneousPoint2D();
            
            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);
            
            if (!refiner.refine(result1)) {
                continue;
            }
            
            InhomogeneousPoint2D result2 = refiner.refine();
            
            assertEquals(mRefineStart, 2);
            assertEquals(mRefineEnd, 2);
            
            assertEquals(result1, result2);
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    private RANSACPoint2DRobustEstimator createRobustEstimator() 
            throws ColinearPointsException, LockedException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);
            
            //compute random lines passing through the point
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Line2D> linesWithError = new ArrayList<Line2D>();
            Line2D line, lineWithError;
        for (int i = 0; i < nLines; i++) {
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
                
                linesWithError.add(lineWithError);
                
                //check that point is locus of line without error
                assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
        }
            
        RANSACPoint2DRobustEstimator estimator = 
                new RANSACPoint2DRobustEstimator(linesWithError);
            
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);            
        
        return estimator;
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }
    
    @Override
    public void onRefineStart(Refiner<InhomogeneousPoint2D> refiner, 
            InhomogeneousPoint2D initialEstimation) {
        mRefineStart++;
        checkLocked((InhomogeneousPoint2DRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<InhomogeneousPoint2D> refiner, 
            InhomogeneousPoint2D initialEstimation, InhomogeneousPoint2D result,
            boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((InhomogeneousPoint2DRefiner)refiner);
    }
    
    private void checkLocked(InhomogeneousPoint2DRefiner refiner) {
        assertTrue(refiner.isLocked());
        try {
            refiner.setInitialEstimation(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.refine(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {            
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.refine();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {            
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.setInliers(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setResiduals(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setNumInliers(0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setInliersData(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setSamples(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
    }    
}
