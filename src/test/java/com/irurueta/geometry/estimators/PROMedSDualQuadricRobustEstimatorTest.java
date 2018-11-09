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

public class PROMedSDualQuadricRobustEstimatorTest implements 
        DualQuadricRobustEstimatorListener {
    
    private static final int MIN_PLANES = 500;
    private static final int MAX_PLANES = 1000;
    
    private static final double MIN_RANDOM_POINT_VALUE = -1.0;
    private static final double MAX_RANDOM_POINT_VALUE = 1.0;
    
    private static final double STD_ERROR = 1.0;
    
    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;
    
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final double THRESHOLD = 1e-9;
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int TIMES = 100;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public PROMedSDualQuadricRobustEstimatorTest() { }
    
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
        PROMedSDualQuadricRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROMedSDualQuadricRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with planes
        List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        
        estimator = new PROMedSDualQuadricRobustEstimator(planes);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Plane> emptyPlanes = new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with listener
        DualQuadricRobustEstimatorListener listener =
                new DualQuadricRobustEstimatorListener() {

            @Override
            public void onEstimateStart(DualQuadricRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(DualQuadricRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    DualQuadricRobustEstimator estimator, int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    DualQuadricRobustEstimator estimator, float progress) { }
        };
        
        estimator = new PROMedSDualQuadricRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with listener and points
        estimator = new PROMedSDualQuadricRobustEstimator(listener, planes);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(listener, 
                    emptyPlanes);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with quality scores
        double[] qualityScores = new double[
                DualQuadricRobustEstimator.MINIMUM_SIZE];
        estimator = new PROMedSDualQuadricRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with planes and quality scores
        estimator = new PROMedSDualQuadricRobustEstimator(planes, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                PROMedSDualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(emptyPlanes, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROMedSDualQuadricRobustEstimator(listener, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(listener, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with listener, points and quality scores
        estimator = new PROMedSDualQuadricRobustEstimator(listener, planes, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getStopThreshold(), 
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPlanes(), planes);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(listener, 
                    emptyPlanes, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROMedSDualQuadricRobustEstimator(listener, planes, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);                          
    }
    
    @Test
    public void testGetSetStopThreshold() throws LockedException {
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getStopThreshold(),
                PROMedSDualQuadricRobustEstimator.DEFAULT_STOP_THRESHOLD, 0.0);
        
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
    public void testGetSetListener() throws LockedException {
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
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
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                DualQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                DualQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                DualQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(1);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 1);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPlanes() throws LockedException {
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
        //check default value
        assertNull(estimator.getPlanes());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Plane> planes = new ArrayList<>();
        for (int i = 0; i < DualQuadricRobustEstimator.MINIMUM_SIZE; i++) {
            planes.add(new Plane());
        }
        estimator.setPlanes(planes);
        
        //check correctness
        assertSame(estimator.getPlanes(), planes);
        assertFalse(estimator.isReady());
        
        //if we set qualitys cores, then estimator becomes ready
        double[] qualityScores = new double[planes.size()];
        estimator.setQualityScores(qualityScores);
        
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
        PROMedSDualQuadricRobustEstimator estimator = 
                new PROMedSDualQuadricRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = 
                new double[DualQuadricRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        qualityScores = new double[1];
        try {
            estimator.setQualityScores(qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException, 
            RobustEstimatorException, DualQuadricNotAvailableException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int t = 0;  t < TIMES; t++) {
            //instantiate a random circle
            Point3D center = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0,
                    MAX_RANDOM_POINT_VALUE));
            
            Sphere sphere = new Sphere(center, radius);
            Quadric quadric = sphere.toQuadric();
            DualQuadric dualQuadric = quadric.getDualQuadric();
            
            //compute planes in the dual quadric locus
            int nPlanes = randomizer.nextInt(MIN_PLANES, MAX_PLANES);
            int halfPoints = (int)Math.ceil((double)nPlanes / 2.0);
            double theta = (double)nPlanes / 360.0 * Math.PI / 180.0;
            double[] qualityScores = new double[nPlanes];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Plane> planes = new ArrayList<>();
            List<Plane> planesWithError = new ArrayList<>();
            Point3D point;
            Plane plane, planeWithError;
            double[] directorVector = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (int i = 0; i < nPlanes; i++) {
                double angle1 = 0.0, angle2 = 0.0;
                if (i < halfPoints) {
                    angle1 = theta * (double)i;
                } else {
                    angle2 = theta * (double)(i - halfPoints);
                }
                point = new HomogeneousPoint3D(
                        center.getInhomX() + radius * Math.cos(angle1) * 
                        Math.cos(angle2),
                        center.getInhomY() + radius * Math.sin(angle1) * 
                        Math.cos(angle2),
                        center.getInhomZ() + radius * Math.sin(angle2), 
                        1.0);            
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                directorVector[2] = point.getInhomZ() - center.getInhomZ();
                
                plane = new Plane(point, directorVector);
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    double errorC = errorRandomizer.nextDouble();
                    planeWithError = new Plane(plane.getA() + errorA,
                        plane.getB() + errorB, plane.getC() + errorC, 
                        plane.getD());
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB +
                            errorC * errorC);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    //inlier plane
                    planeWithError = plane;
                }

                planes.add(plane);
                planesWithError.add(planeWithError);
                
                //check that point without error is within quadric locus
                assertTrue(sphere.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(quadric.isLocus(point, ABSOLUTE_ERROR));                
                assertTrue(dualQuadric.isLocus(plane, ABSOLUTE_ERROR));
            }
    
            PROMedSDualQuadricRobustEstimator estimator = 
                    new PROMedSDualQuadricRobustEstimator(this, planesWithError,
                    qualityScores);
            
            estimator.setStopThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            DualQuadric dualQuadric2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all planes
            //are within the estimated conic locus
            for (Plane p : planes) {
                assertTrue(dualQuadric2.isLocus(p, ABSOLUTE_ERROR));
            }    
        }
    }

    @Override
    public void onEstimateStart(DualQuadricRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSDualQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(DualQuadricRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSDualQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(DualQuadricRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSDualQuadricRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(DualQuadricRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSDualQuadricRobustEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(PROMedSDualQuadricRobustEstimator estimator) {
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
        assertTrue(estimator.isLocked());
    }        
}
