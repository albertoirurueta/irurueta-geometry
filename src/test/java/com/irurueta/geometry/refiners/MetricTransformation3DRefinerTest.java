/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.refiners.MetricTransformation3DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 2, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Utils;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACMetricTransformation3DRobustEstimator;
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

public class MetricTransformation3DRefinerTest implements 
        RefinerListener<MetricTransformation3D> {
    
    public static final double MIN_RANDOM_VALUE = -1000.0;
    public static final double MAX_RANDOM_VALUE = 1000.0;
    
    public static final double MIN_RANDOM_DEGREES = -180.0;
    public static final double MAX_RANDOM_DEGREES = 180.0;
    
    public static final double MIN_SCALE = 0.5;
    public static final double MAX_SCALE = 2.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int MIN_POINTS = 50;
    public static final int MAX_POINTS = 100;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double STD_ERROR = 100.0;
    public static final double THRESHOLD = 1e-6;
    
    public static final int TIMES = 100;
    
    private int mRefineStart;
    private int mRefineEnd;
    
    public MetricTransformation3DRefinerTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws AlgebraException, LockedException, 
            NotReadyException, RobustEstimatorException {
        RANSACMetricTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        MetricTransformation3D transformation = estimator.estimate();
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        double[] residuals = inliersData.getResiduals();
        int numInliers = inliersData.getNumInliers();
        double refinementStandardDeviation = estimator.getThreshold();
        List<Point3D> samples1 = estimator.getInputPoints();
        List<Point3D> samples2 = estimator.getOutputPoints();
        
        assertNotNull(transformation);
        assertNotNull(inliersData);
        
        //test empty constructor
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertEquals(refiner.getNumInliers(), 0);
        assertEquals(refiner.getTotalSamples(), 0);
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        //test non-empty constructor
        refiner = new MetricTransformation3DRefiner(
                transformation, true, inliers, residuals, numInliers, samples1, 
                samples2, refinementStandardDeviation);

        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), transformation);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());        
        
        refiner = new MetricTransformation3DRefiner(
                transformation, true, inliersData, samples1, samples2, 
                refinementStandardDeviation);

        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), transformation);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());        
    }
    
    @Test
    public void testGetSetListener() {
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

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
    public void testGetSetSamples1() throws LockedException {
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();
        
        //check default value
        assertNull(refiner.getSamples1());
        
        //set new value
        List<Point3D> samples1 = new ArrayList<Point3D>();
        refiner.setSamples1(samples1);
        
        //check correctness
        assertSame(refiner.getSamples1(), samples1);        
    }
    
    @Test
    public void testGetSetSamples2() throws LockedException {
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();
        
        //check default value
        assertNull(refiner.getSamples2());
        
        //set new value
        List<Point3D> samples2 = new ArrayList<Point3D>();
        refiner.setSamples2(samples2);
        
        //check correctness
        assertSame(refiner.getSamples2(), samples2);        
    }
    
    @Test
    public void testGetSetInliers() throws AlgebraException, LockedException, 
            NotReadyException, RobustEstimatorException {
        RANSACMetricTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();        
        
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

        //check default value
        assertNull(refiner.getInliers());
        
        //set new value
        refiner.setInliers(inliers);
        
        //check correctness
        assertSame(refiner.getInliers(), inliers);
    }
    
    @Test
    public void testGetSetResiduals() throws AlgebraException, LockedException, 
            NotReadyException, RobustEstimatorException {
        RANSACMetricTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        double[] residuals = inliersData.getResiduals();
        
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

        //check default value
        assertNull(refiner.getResiduals());
        
        //set new value
        refiner.setResiduals(residuals);
        
        //check correctness
        assertSame(refiner.getResiduals(), residuals);
    }
    
    @Test
    public void testGetSetNumInliers() throws AlgebraException, 
            LockedException, NotReadyException, RobustEstimatorException {
        RANSACMetricTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        int numInliers = inliersData.getNumInliers();
        
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

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
    public void testSetInliersData() throws AlgebraException, LockedException, 
            NotReadyException, RobustEstimatorException {
        RANSACMetricTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        
        MetricTransformation2DRefiner refiner = 
                new MetricTransformation2DRefiner();

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
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

        //check default values
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        MetricTransformation3D transformation = 
                new MetricTransformation3D();
        refiner.setInitialEstimation(transformation);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), transformation);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        MetricTransformation3DRefiner refiner = 
                new MetricTransformation3DRefiner();

        //check default values
        assertFalse(refiner.isCovarianceKept());
        
        //set new value
        refiner.setCovarianceKept(true);
        
        //check correctness
        assertTrue(refiner.isCovarianceKept());
    }
    
    @Test
    public void testRefine() throws AlgebraException, LockedException, 
            NotReadyException, RobustEstimatorException, RefinerException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            RANSACMetricTransformation3DRobustEstimator estimator = 
                    createRobustEstimator();
            
            MetricTransformation3D transformation = estimator.estimate();
            InliersData inliersData = estimator.getInliersData();
            double refineStandardDeviation = estimator.getThreshold();
            List<Point3D> samples1 = estimator.getInputPoints();
            List<Point3D> samples2 = estimator.getOutputPoints();
            
            MetricTransformation3DRefiner refiner = 
                    new MetricTransformation3DRefiner(transformation, true,
                    inliersData, samples1, samples2, refineStandardDeviation);
            refiner.setListener(this);
            
            MetricTransformation3D result1 = new MetricTransformation3D();
            
            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);
            
            if (!refiner.refine(result1)) {
                continue;
            }
            
            MetricTransformation3D result2 = refiner.refine();
            
            assertEquals(mRefineStart, 2);
            assertEquals(mRefineEnd, 2);
            
            assertTrue(result1.asMatrix().equals(result2.asMatrix(), 
                    ABSOLUTE_ERROR));
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }            
        }
        
        assertTrue(numValid > 0);
    }    

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }
    
    @Override
    public void onRefineStart(Refiner<MetricTransformation3D> refiner, 
            MetricTransformation3D initialEstimation) {
        mRefineStart++;
        checkLocked((MetricTransformation3DRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<MetricTransformation3D> refiner, 
            MetricTransformation3D initialEstimation, 
            MetricTransformation3D result, boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((MetricTransformation3DRefiner)refiner);
    }
    
    private RANSACMetricTransformation3DRobustEstimator
            createRobustEstimator() throws LockedException {
                
        MetricTransformation3D transformation = createTransformation();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        List<Point3D> inputPoints = new ArrayList<Point3D>();
        List<Point3D> outputPointsWithError = new ArrayList<Point3D>();
        Point3D inputPoint, outputPoint, outputPointWithError;
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        for (int i = 0; i < nPoints; i++) {
            //generate input point
            inputPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            outputPoint = transformation.transformAndReturnNew(inputPoint);
            
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
            outputPointsWithError.add(outputPointWithError);
        }
        
        RANSACMetricTransformation3DRobustEstimator estimator =
                new RANSACMetricTransformation3DRobustEstimator(inputPoints,
                outputPointsWithError);
        
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);
        
        return estimator;        
    }
            
    private MetricTransformation3D createTransformation() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        
        Quaternion rotation = new Quaternion(roll, pitch, yaw);
        
        double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        
        return new MetricTransformation3D(rotation, translation, scale);
    }
    
    private void checkLocked(MetricTransformation3DRefiner refiner) {
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
            refiner.setSamples1(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            refiner.setSamples2(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }        
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }                
    }
}