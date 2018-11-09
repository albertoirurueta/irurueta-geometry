/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPoint2DRobustEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class HomogeneousPoint2DRefinerTest implements 
        RefinerListener<HomogeneousPoint2D> {
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double ABSOLUTE_ERROR = 5e-6;
    
    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;
    
    private static final double THRESHOLD = 1e-6;
    
    private static final double STD_ERROR = 100.0;
    
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final int TIMES = 100;
    
    private int mRefineStart;
    private int mRefineEnd;    
    
    public HomogeneousPoint2DRefinerTest() { }
    
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
            RobustEstimatorException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        HomogeneousPoint2D point = new HomogeneousPoint2D(
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
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
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
        refiner = new HomogeneousPoint2DRefiner(point, true, inliers, 
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
        refiner = new HomogeneousPoint2DRefiner(point, true, inliersData, 
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
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
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
    public void testGetSetSamples() throws LockedException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        List<Line2D> samples = estimator.getLines();
        
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getSamples());
        
        //set new value
        refiner.setSamples(samples);
        
        //check correctness
        assertSame(refiner.getSamples(), samples);
    }  
    
    @Test
    public void testGetSetInliers() throws LockedException, NotReadyException,
            RobustEstimatorException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getInliers());
        
        //set new value
        refiner.setInliers(inliers);
        
        //check correctness
        assertSame(refiner.getInliers(), inliers);
    }
    
    @Test
    public void testGetSetResiduals() throws LockedException, NotReadyException,
            RobustEstimatorException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        double[] residuals = inliersData.getResiduals();
        
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getResiduals());
        
        //set new value
        refiner.setResiduals(residuals);
        
        //check correctness
        assertSame(refiner.getResiduals(), residuals);
    }
    
    @Test
    public void testGetSetNumInliers() throws LockedException, NotReadyException,
            RobustEstimatorException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        int numInliers = inliersData.getNumInliers();
        
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
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
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testSetInliersData() throws LockedException, NotReadyException,
            RobustEstimatorException {
        RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
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
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
        //check default value
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        HomogeneousPoint2D point = new HomogeneousPoint2D();
        refiner.setInitialEstimation(point);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), point);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        HomogeneousPoint2DRefiner refiner = new HomogeneousPoint2DRefiner();
        
        //check default value
        assertFalse(refiner.isCovarianceKept());
        
        //set new value
        refiner.setCovarianceKept(true);
        
        //check correctness
        assertTrue(refiner.isCovarianceKept());
    }
    
    @Test    
    public void testRefine() throws LockedException, NotReadyException,
            RobustEstimatorException, RefinerException {
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            RANSACPoint2DRobustEstimator estimator = createRobustEstimator();
            
            HomogeneousPoint2D point = new HomogeneousPoint2D(
                    estimator.estimate());
            InliersData inliersData = estimator.getInliersData();
            double refineStandardDeviation = estimator.getThreshold();
            List<Line2D> samples = estimator.getLines();
            
            HomogeneousPoint2DRefiner refiner = 
                    new HomogeneousPoint2DRefiner(point, true, inliersData,
                    samples, refineStandardDeviation);
            refiner.setListener(this);
            
            HomogeneousPoint2D result1 = new HomogeneousPoint2D();
            
            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);
            
            if (!refiner.refine(result1)) {
                continue;
            }
            
            HomogeneousPoint2D result2 = refiner.refine();
            
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
            throws LockedException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                1.0);
            
        //compute random lines passing through the point
        int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);
        List<Line2D> linesWithError = new ArrayList<>();
        Line2D line, lineWithError;
        for (int i = 0; i < nLines; i++) {
                //get another point (far enough to compute a line)
                Point2D anotherPoint;
                do {
                    anotherPoint = new HomogeneousPoint2D(
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE),
                            randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE), 1.0);
                } while (anotherPoint.distanceTo(point) < STD_ERROR);
                
                line = new Line2D(point, anotherPoint);
                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                            line.getB() + errorB, line.getC() + errorC);
                } else {
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

    @Override
    public void onRefineStart(Refiner<HomogeneousPoint2D> refiner, 
            HomogeneousPoint2D initialEstimation) {
        mRefineStart++;
        checkLocked((HomogeneousPoint2DRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<HomogeneousPoint2D> refiner, 
            HomogeneousPoint2D initialEstimation, HomogeneousPoint2D result,
            boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((HomogeneousPoint2DRefiner)refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(HomogeneousPoint2DRefiner refiner) {
        assertTrue(refiner.isLocked());
        try {
            refiner.setInitialEstimation(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.refine(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.refine();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.setInliers(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setResiduals(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setNumInliers(0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setInliersData(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setSamples(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
    }        
}
