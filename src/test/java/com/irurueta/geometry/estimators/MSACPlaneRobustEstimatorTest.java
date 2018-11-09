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

import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACPlaneRobustEstimatorTest implements 
        PlaneRobustEstimatorListener {
    
    private static final double MIN_RANDOM_VALUE = -1.0;
    private static final double MAX_RANDOM_VALUE = 1.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int MIN_POINTS = 500;
    private static final int MAX_POINTS = 1000;
    
    private static final double THRESHOLD = 1e-6;
    
    private static final double STD_ERROR = 100.0;
    
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public MSACPlaneRobustEstimatorTest() { }
    
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
        MSACPlaneRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new MSACPlaneRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                RANSACPlaneRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PlaneRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point3D> points = new ArrayList<>();
        for (int i = 0; i < PlaneRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point3D.create());
        }
        
        estimator = new MSACPlaneRobustEstimator(points);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PlaneRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new MSACPlaneRobustEstimator(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with listener
        PlaneRobustEstimatorListener listener =
                new PlaneRobustEstimatorListener() {

            @Override
            public void onEstimateStart(PlaneRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(PlaneRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    PlaneRobustEstimator estimator, int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    PlaneRobustEstimator estimator, float progress) { }
        };
        
        estimator = new MSACPlaneRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PlaneRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with listener and points
        estimator = new MSACPlaneRobustEstimator(listener, points);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                PlaneRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACPlaneRobustEstimator(listener, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                MSACPlaneRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
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
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                PlaneRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                PlaneRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                PlaneRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
    public void testGetSetPoints() throws LockedException {
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Point3D> points = new ArrayList<>();
        for (int i = 0; i < PlaneRobustEstimator.MINIMUM_SIZE; i++) {
            points.add(Point3D.create());
        }
        estimator.setPoints(points);
        
        //check correctness
        assertSame(estimator.getPoints(), points);
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        points.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Point3D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        MSACPlaneRobustEstimator estimator =
                new MSACPlaneRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = new double[
                PlaneRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException,
            RobustEstimatorException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int t = 0; t < TIMES; t++) {
            double a = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double b = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double c = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            double d = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);            
            Plane plane = new Plane(a, b, c, d);
            
            //compute random points passing through the line
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Point3D> points = new ArrayList<>();
            List<Point3D> pointsWithError = new ArrayList<>();
            Point3D point, pointWithError;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane 
                //(a*x + b*y + c*z + d*w = 0)
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if (Math.abs(b) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                point = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double errorZ = errorRandomizer.nextDouble();
                    pointWithError = new HomogeneousPoint3D(
                            point.getHomX() + errorX * point.getHomW(),
                            point.getHomY() + errorY * point.getHomW(), 
                            point.getHomZ() + errorZ * point.getHomW(), 
                            point.getHomW());
                } else {
                    //inlier point
                    pointWithError = point;
                }
                
                points.add(point);
                pointsWithError.add(pointWithError);
                
                //check that point without error is locus of line
                assertTrue(plane.isLocus(point, ABSOLUTE_ERROR));
            }
            
            MSACPlaneRobustEstimator estimator =
                    new MSACPlaneRobustEstimator(this, pointsWithError);
            
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            Plane plane2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points without
            //error have estimated line as locus
            for (Point3D p : points) {
                assertTrue(plane2.isLocus(p, ABSOLUTE_ERROR));
            }
            
            //check that both lines are equal
            plane.normalize();
            plane2.normalize();
            assertTrue(plane.equals(plane2, ABSOLUTE_ERROR));
        }
    }

    @Override
    public void onEstimateStart(PlaneRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACPlaneRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(PlaneRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACPlaneRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(PlaneRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        checkLocked((MSACPlaneRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(PlaneRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        checkLocked((MSACPlaneRobustEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(MSACPlaneRobustEstimator estimator) {
        try {
            estimator.setThreshold(0.5);
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
            estimator.setPoints(null);
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
