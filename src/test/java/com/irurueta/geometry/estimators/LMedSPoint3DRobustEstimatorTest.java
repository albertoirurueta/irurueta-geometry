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

import com.irurueta.geometry.*;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LMedSPoint3DRobustEstimatorTest implements 
        Point3DRobustEstimatorListener {
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double ABSOLUTE_ERROR = 5e-6;
    
    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;
    
    private static final double STOP_THRESHOLD = 1e-3;
    
    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public LMedSPoint3DRobustEstimatorTest() { }
    
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
        LMedSPoint3DRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new LMedSPoint3DRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());
        
        //test constructor with planes
        List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        
        estimator = new LMedSPoint3DRobustEstimator(planes);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<>();
        estimator = null;
        try {
            estimator = new LMedSPoint3DRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with listener
        Point3DRobustEstimatorListener listener =
                new Point3DRobustEstimatorListener() {

            @Override
            public void onEstimateStart(Point3DRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(Point3DRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    Point3DRobustEstimator estimator, int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    Point3DRobustEstimator estimator, float progress) { }
        };
        
        estimator = new LMedSPoint3DRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());        
        
        //test constructor with listener and planes
        estimator = new LMedSPoint3DRobustEstimator(listener, planes);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                Point3DRobustEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.getRefinementCoordinatesType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);    
        assertFalse(estimator.isCovarianceKept());        
        assertNull(estimator.getCovariance());        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMedSPoint3DRobustEstimator(listener, emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }
    
    @Test
    public void testGetSetStopThreshold() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(),
                LMedSPoint3DRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        
        //set new value
        estimator.setStopThreshold(0.5);
        
        assertEquals(estimator.getStopThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
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
    public void testGetSetProgressDelta() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                Point3DRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
    public void testGetSetConfidence() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                Point3DRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5f);
        
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
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                Point3DRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(1);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 1);
        
        //Fail IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPlanes() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < Point3DRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);
        
        //check correctness
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        planes.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<>();
        try {
            estimator.setPlanes(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[
                Point3DRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testIsSetResultRefined() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();

        assertTrue(estimator.isResultRefined());
        
        //set new value
        estimator.setResultRefined(false);
        
        //check correctness
        assertFalse(estimator.isResultRefined());
    }
    
    @Test
    public void testGetSetRefinementCoordinatesType() throws LockedException {
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
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
        LMedSPoint3DRobustEstimator estimator =
                new LMedSPoint3DRobustEstimator();
        
        assertFalse(estimator.isCovarianceKept());
        
        //set new value
        estimator.setCovarianceKept(true);
        
        //check correctness
        assertTrue(estimator.isCovarianceKept());
    }
    
    @Test
    public void testEstimateWithoutRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            ColinearPointsException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for(int t = 0; t < TIMES; t++){
            Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random planes passing through the point
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<>();
            List<Plane> planesWithError = new ArrayList<>();
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
                
                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            LMedSPoint3DRobustEstimator estimator =
                    new LMedSPoint3DRobustEstimator(this, planesWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);            
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point3D point2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes 
            //without error have estimated point as locus
            for (Plane p : planes) {
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            if (point.distanceTo(point2) > 2.0*ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(point.distanceTo(point2), 0.0, 2.0*ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithInhomogeneousRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            ColinearPointsException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int t = 0; t < TIMES; t++) {
            Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random planes passing through the point
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<>();
            List<Plane> planesWithError = new ArrayList<>();
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
                
                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            LMedSPoint3DRobustEstimator estimator =
                    new LMedSPoint3DRobustEstimator(this, planesWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point3D point2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes 
            //without error have estimated point as locus
            for (Plane p : planes) {
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, 2.0*ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithHomogeneousRefinement() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            ColinearPointsException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int t = 0; t < TIMES; t++) {
            Point3D point = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    1.0);
            
            //compute random planes passing through the point
            int nPlanes = randomizer.nextInt(MIN_LINES, MAX_LINES);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<>();
            List<Plane> planesWithError = new ArrayList<>();
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
                
                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point is locus of plane without error
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            LMedSPoint3DRobustEstimator estimator =
                    new LMedSPoint3DRobustEstimator(this, planesWithError);
            
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Point3D point2 = estimator.estimate();
            
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());
            assertEquals(estimator.getCovariance().getRows(),
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            assertEquals(estimator.getCovariance().getColumns(),
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes 
            //without error have estimated point as locus
            for (Plane p : planes) {
                assertTrue(p.isLocus(point2, ABSOLUTE_ERROR));
            }
            
            //check that both points are equal
            assertEquals(point.distanceTo(point2), 0.0, 2.0*ABSOLUTE_ERROR);
        }
    }

    @Override
    public void onEstimateStart(Point3DRobustEstimator estimator) {
        estimateStart++;
        checkLocked((LMedSPoint3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(Point3DRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((LMedSPoint3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(Point3DRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSPoint3DRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(Point3DRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        checkLocked((LMedSPoint3DRobustEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(LMedSPoint3DRobustEstimator estimator){
        try {
            estimator.setStopThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPlanes(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setRefinementCoordinatesType(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        assertTrue(estimator.isLocked());
    }
}
