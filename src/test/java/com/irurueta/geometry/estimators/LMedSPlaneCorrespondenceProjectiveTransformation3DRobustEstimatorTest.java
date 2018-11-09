/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.*;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest 
        implements ProjectiveTransformation3DRobustEstimatorListener{
    
    private static final double MIN_RANDOM_VALUE = -1000.0;
    private static final double MAX_RANDOM_VALUE = 1000.0;
    
    private static final double ABSOLUTE_ERROR = 5e-5;
    
    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;
    
    private static final double STOP_THRESHOLD = 1e-6;
    
    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimatorTest() { }
    
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
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
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

        
        //test constructor with planes
        List<Plane> inputPlanes = new ArrayList<>();
        List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                inputPlanes, outputPlanes);
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
        
        //Force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<>();
        estimator = null;
        try {
            //not enough planes
            estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        
        //test constructor with listener
        estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this);
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
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

        
        //test constructor with listener and points
        estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                this, inputPlanes, outputPlanes);        
        
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
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
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            //not enough points
            estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator = new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
                    this, inputPlanes, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);                
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(), 
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_STOP_THRESHOLD, 0.0);
        
        //set new value
        estimator.setStopThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }  
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getConfidence(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }    

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getMaxIterations(),
                LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPlanesAndIsReady() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        //check default values
        assertNull(estimator.getInputPlanes());
        assertNull(estimator.getOutputPlanes());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> inputPlanes = new ArrayList<>();
        List<Plane> outputPlanes = new ArrayList<>();
        for (int i = 0; i < PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.MINIMUM_SIZE; i++) {
            inputPlanes.add(new Plane());
            outputPlanes.add(new Plane());
        }
        
        estimator.setPlanes(inputPlanes, outputPlanes);
        
        //check correctness
        assertSame(estimator.getInputPlanes(), inputPlanes);
        assertSame(estimator.getOutputPlanes(), outputPlanes);
        assertTrue(estimator.isReady());

        //Force IllegalArgumentException
        List<Plane> planesEmpty = new ArrayList<>();
        try {
            //not enough planes
            estimator.setPlanes(planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //different sizes
            estimator.setPlanes(planesEmpty, planesEmpty);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }    
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                ProjectiveTransformation3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }    
    
    @Test
    public void testIsSetResultRefined() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }    
    
    @Test
    public void testEstimateWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //create an affine transformation
            Matrix A;
            do {
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation3D.INHOM_COORDS, 
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(A) < ProjectiveTransformation3D.INHOM_COORDS);
            
            double[] translation = new double[
                    ProjectiveTransformation3D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            ProjectiveTransformation3D transformation1 =
                    new ProjectiveTransformation3D(A, translation);
            
            //generate random planes
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            List<Plane> inputPlanes = new ArrayList<>();
            List<Plane> outputPlanes = new ArrayList<>();
            List<Plane> outputPlanesWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPlanes; i++) {
                Plane inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Plane outputPlane = transformation1.transformAndReturnNew(inputPlane);
                Plane outputPlaneWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                } else {
                    //inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }
                
                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }
            
            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
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
            
            ProjectiveTransformation3D transformation2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by transforming input plane
            //using estimated transformation (transformation2) and checking
            //that output planes are equal to the original output planes without
            //error
            Plane plane1, plane2;
            boolean failed = false;
            for (int i = 0; i < nPlanes; i++) {
                plane1 = outputPlanes.get(i);
                plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (Math.abs(PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2)) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2), 0.0, ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //create an affine transformation
            Matrix A;
            do {
                //ensure A matrix is invertible
                A = Matrix.createWithUniformRandomValues(
                        ProjectiveTransformation3D.INHOM_COORDS, 
                        ProjectiveTransformation3D.INHOM_COORDS, -1.0, 1.0);
                double norm = Utils.normF(A);
                //normalize T to increase accuracy
                A.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(A) < ProjectiveTransformation3D.INHOM_COORDS);
            
            double[] translation = new double[
                    ProjectiveTransformation3D.INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);
            
            ProjectiveTransformation3D transformation1 =
                    new ProjectiveTransformation3D(A, translation);
            
            //generate random planes
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            List<Plane> inputPlanes = new ArrayList<>();
            List<Plane> outputPlanes = new ArrayList<>();
            List<Plane> outputPlanesWithError = new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < nPlanes; i++) {
                Plane inputPlane = new Plane(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                Plane outputPlane = transformation1.transformAndReturnNew(inputPlane);
                Plane outputPlaneWithError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //line is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    double errorD = errorRandomizer.nextDouble();
                    outputPlaneWithError = new Plane(outputPlane.getA() + errorA,
                            outputPlane.getB() + errorB,
                            outputPlane.getC() + errorC,
                            outputPlane.getD() + errorD);
                } else {
                    //inlier plane (without error)
                    outputPlaneWithError = outputPlane;
                }
                
                inputPlanes.add(inputPlane);
                outputPlanes.add(outputPlane);
                outputPlanesWithError.add(outputPlaneWithError);
            }
            
            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator =
                new LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator(
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
            
            //check correctness of estimation by transforming input plane
            //using estimated transformation (transformation2) and checking
            //that output planes are equal to the original output planes without
            //error
            Plane plane1, plane2;
            boolean valid = true;
            for (int i = 0; i < nPlanes; i++) {
                plane1 = outputPlanes.get(i);
                plane2 = transformation2.transformAndReturnNew(inputPlanes.get(i));
                plane1.normalize();
                plane2.normalize();
                if (PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2) > ABSOLUTE_ERROR) {
                    valid = false;
                    break;
                }

                assertEquals(
                        PlaneCorrespondenceProjectiveTransformation3DRobustEstimator.
                        getResidual(plane1, plane2), 0.0, ABSOLUTE_ERROR);
                assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
            }

            if (valid) {
                numValid++;
            }
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(ProjectiveTransformation3DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(
            ProjectiveTransformation3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            ProjectiveTransformation3DRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            ProjectiveTransformation3DRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        checkLocked((LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(
            LMedSPlaneCorrespondenceProjectiveTransformation3DRobustEstimator estimator) {
        List<Plane> planes = new ArrayList<>();
        try {
            estimator.setPlanes(planes, planes);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.01f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setStopThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setConfidence(0.5);            
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }    
}
