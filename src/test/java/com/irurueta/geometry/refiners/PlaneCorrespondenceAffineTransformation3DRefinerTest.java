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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator;
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

public class PlaneCorrespondenceAffineTransformation3DRefinerTest implements 
        RefinerListener<AffineTransformation3D> {
    
    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int MIN_POINTS = 50;
    private static final int MAX_POINTS = 100;
    
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final double STD_ERROR = 100.0;
    private static final double THRESHOLD = 1e-6;
    
    private static final int TIMES = 1000;
    
    private int mRefineStart;
    private int mRefineEnd;
    
    public PlaneCorrespondenceAffineTransformation3DRefinerTest() { }
    
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
        RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                createRobustEstimator();
        AffineTransformation3D transformation = estimator.estimate();
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        double[] residuals = inliersData.getResiduals();
        int numInliers = inliersData.getNumInliers();
        double refinementStandardDeviation = estimator.getThreshold();
        List<Plane> samples1 = estimator.getInputPlanes();
        List<Plane> samples2 = estimator.getOutputPlanes();
        
        assertNotNull(transformation);
        assertNotNull(inliersData);

        //test empty constructor
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();
        
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
        refiner = new PlaneCorrespondenceAffineTransformation3DRefiner(
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
        
        refiner = new PlaneCorrespondenceAffineTransformation3DRefiner(
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
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();
        
        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();
        
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
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();

        //check default value
        assertNull(refiner.getSamples1());
        
        //set new value
        List<Plane> samples1 = new ArrayList<>();
        refiner.setSamples1(samples1);
        
        //check correctness
        assertSame(refiner.getSamples1(), samples1);
    }
    
    @Test
    public void testGetSetSamples2() throws LockedException {
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();

        //check default value
        assertNull(refiner.getSamples2());
        
        //set new value
        List<Plane> samples2 = new ArrayList<>();
        refiner.setSamples2(samples2);
        
        //check correctness
        assertSame(refiner.getSamples2(), samples2);        
    }
    
    @Test
    public void testGetSetInliers() throws LockedException, AlgebraException, 
            NotReadyException, RobustEstimatorException {
        RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();
        
        //check default value
        assertNull(refiner.getInliers());
        
        //set new value
        refiner.setInliers(inliers);
        
        //check correctness
        assertSame(refiner.getInliers(), inliers);
    }
    
    @Test
    public void testGetSetResiduals() throws LockedException, AlgebraException, 
            NotReadyException, RobustEstimatorException {
        RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        double[] residuals = inliersData.getResiduals();
        
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();
        
        //check default value
        assertNull(refiner.getResiduals());
        
        //set new value
        refiner.setResiduals(residuals);
        
        //check correctness
        assertSame(refiner.getResiduals(), residuals);
    }
    
    @Test
    public void testGetSetNumInliers() throws LockedException, AlgebraException,
            NotReadyException, RobustEstimatorException {
        RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        int numInliers = inliersData.getNumInliers();
        
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();
        
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
    public void testSetInliersData() throws LockedException, AlgebraException, 
            NotReadyException, RobustEstimatorException {
        RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();

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
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();

        //check default value
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        AffineTransformation3D transformation = new AffineTransformation3D();
        refiner.setInitialEstimation(transformation);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), transformation);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                new PlaneCorrespondenceAffineTransformation3DRefiner();

        //check default value
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
            RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator = 
                    createRobustEstimator();
            
            AffineTransformation3D transformation = estimator.estimate();
            InliersData inliersData = estimator.getInliersData();
            double refineStandardDeviation = estimator.getThreshold();
            List<Plane> samples1 = estimator.getInputPlanes();
            List<Plane> samples2 = estimator.getOutputPlanes();            
            
            PlaneCorrespondenceAffineTransformation3DRefiner refiner =
                    new PlaneCorrespondenceAffineTransformation3DRefiner(
                    transformation, true, inliersData, samples1, samples2, 
                    refineStandardDeviation);
            refiner.setListener(this);
            
            AffineTransformation3D result1 = new AffineTransformation3D();
            
            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);
            
            if (!refiner.refine(result1)) {
                continue;
            }
            
            AffineTransformation3D result2 = refiner.refine();
            
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

    private RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator 
            createRobustEstimator() throws AlgebraException, LockedException {
            
        AffineTransformation3D transformation = createTransformation();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //generate random planes
        int nPlanes = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        List<Plane> inputPlanes = new ArrayList<>();
        List<Plane> outputPlanesWithError = new ArrayList<>();
        Plane inputPlane, outputPlane, outputPlaneWithError;
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);                    
        for (int i = 0; i < nPlanes; i++) {
            //generate input point
            inputPlane = new Plane(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            outputPlane = transformation.transformAndReturnNew(inputPlane);
            
            if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                //plane is outlier
                double errorA = errorRandomizer.nextDouble();
                double errorB = errorRandomizer.nextDouble();
                double errorC = errorRandomizer.nextDouble();
                double errorD = errorRandomizer.nextDouble();
                outputPlaneWithError = new Plane(
                        outputPlane.getA() + errorA,
                        outputPlane.getB() + errorB,
                        outputPlane.getC() + errorC,
                        outputPlane.getD() + errorD);
            } else {
                //inlier plane (without error)
                outputPlaneWithError = outputPlane;
            }
            
            inputPlanes.add(inputPlane);
            outputPlanesWithError.add(outputPlaneWithError);
        }
        
        RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator estimator =
                new RANSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
                inputPlanes, outputPlanesWithError);
        
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);
        
        return estimator;
    }    
    
    private AffineTransformation3D createTransformation() 
            throws AlgebraException {
            
        Matrix A;
        do {
            //ensure A matrix is invertible
            A = Matrix.createWithUniformRandomValues(
                    AffineTransformation3D.INHOM_COORDS, 
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            double norm = Utils.normF(A);
            //normalize T to increase accuracy
            A.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(A) < AffineTransformation3D.INHOM_COORDS);
            
        double[] translation = new double[AffineTransformation3D.INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);
        
        return new AffineTransformation3D(A, translation);
    }    

    @Override
    public void onRefineStart(Refiner<AffineTransformation3D> refiner, 
            AffineTransformation3D initialEstimation) {
        mRefineStart++;
        checkLocked((PlaneCorrespondenceAffineTransformation3DRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<AffineTransformation3D> refiner, 
            AffineTransformation3D initialEstimation, 
            AffineTransformation3D result, boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((PlaneCorrespondenceAffineTransformation3DRefiner)refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(
            PlaneCorrespondenceAffineTransformation3DRefiner refiner) {
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
            refiner.setSamples1(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setSamples2(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
    }    
}
