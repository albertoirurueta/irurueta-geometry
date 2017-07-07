/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.refiners.HomogeneousPoint3DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 7, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.ColinearPointsException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPoint3DRobustEstimator;
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

public class HomogeneousPoint3DRefinerTest implements 
        RefinerListener<HomogeneousPoint3D> {
    
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
    
    public HomogeneousPoint3DRefinerTest() {
    }
    
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
        RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        HomogeneousPoint3D point = new HomogeneousPoint3D(
                estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        double[] residuals = inliersData.getResiduals();
        int numInliers = inliersData.getNumInliers();
        double refinementStandardDeviation = estimator.getThreshold();
        List<Plane> samples = estimator.getPlanes();
        
        assertNotNull(point);
        assertNotNull(inliersData);
        
        //test empty constructor
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        refiner = new HomogeneousPoint3DRefiner(point, true, inliers, 
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
        refiner = new HomogeneousPoint3DRefiner(point, true, inliersData, 
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
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        List<Plane> samples = estimator.getPlanes();
        
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        double[] residuals = inliersData.getResiduals();
        
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        int numInliers = inliersData.getNumInliers();
        
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
        //check default value
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        HomogeneousPoint3D point = new HomogeneousPoint3D();
        refiner.setInitialEstimation(point);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), point);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        HomogeneousPoint3DRefiner refiner = new HomogeneousPoint3DRefiner();
        
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
            RANSACPoint3DRobustEstimator estimator = createRobustEstimator();
            
            HomogeneousPoint3D point = new HomogeneousPoint3D(
                    estimator.estimate());
            InliersData inliersData = estimator.getInliersData();
            double refineStandardDeviation = estimator.getThreshold();
            List<Plane> samples = estimator.getPlanes();
            
            HomogeneousPoint3DRefiner refiner = 
                    new HomogeneousPoint3DRefiner(point, true, inliersData,
                    samples, refineStandardDeviation);
            refiner.setListener(this);
            
            HomogeneousPoint3D result1 = new HomogeneousPoint3D();
            
            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);
            
            if (!refiner.refine(result1)) {
                continue;
            }
            
            HomogeneousPoint3D result2 = refiner.refine();
            
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
    
    private RANSACPoint3DRobustEstimator createRobustEstimator() 
            throws ColinearPointsException, LockedException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point3D point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                1.0);
            
        //compute random planes passing through the point
        int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        List<Plane> planesWithError = new ArrayList<Plane>();
        Plane plane, planeWithError;
        for (int i = 0; i < nPlanes; i++) {
            //get two more points(far enough to compute a plane)
            Point3D point2, point3;
            do {
                point2 = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),                            
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), 1.0);
            } while (point2.distanceTo(point) < STD_ERROR);
            do {
                point3 = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),                            
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE), 1.0);
            } while (point3.distanceTo(point) < STD_ERROR ||
                    point3.distanceTo(point2) < STD_ERROR);
                
            plane = new Plane(point, point2, point3);
                
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                //line is outlier
                double errorA = errorRandomizer.nextDouble();
                double errorB = errorRandomizer.nextDouble();
                double errorC = errorRandomizer.nextDouble();
                double errorD = errorRandomizer.nextDouble();
                planeWithError = new Plane(plane.getA() + errorA,
                        plane.getB() + errorB, plane.getC() + errorC,
                        plane.getD() + errorD);
            } else {
                //inlier plane
                planeWithError = plane;
            }
                
            planesWithError.add(planeWithError);
                
            //check that point is locus of plane without error
            assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
        }
            
        RANSACPoint3DRobustEstimator estimator = 
                new RANSACPoint3DRobustEstimator(planesWithError);
            
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
    public void onRefineStart(Refiner<HomogeneousPoint3D> refiner, 
            HomogeneousPoint3D initialEstimation) {
        mRefineStart++;
        checkLocked((HomogeneousPoint3DRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<HomogeneousPoint3D> refiner, 
            HomogeneousPoint3D initialEstimation, HomogeneousPoint3D result,
            boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((HomogeneousPoint3DRefiner)refiner);
    }
    
    private void checkLocked(HomogeneousPoint3DRefiner refiner) {
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
