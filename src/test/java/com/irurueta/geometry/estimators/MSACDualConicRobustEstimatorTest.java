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

public class MSACDualConicRobustEstimatorTest implements 
        DualConicRobustEstimatorListener {
    
    private static final int MIN_LINES = 500;
    private static final int MAX_LINES = 1000;
    
    private static final double MIN_RANDOM_POINT_VALUE = -1.0;
    private static final double MAX_RANDOM_POINT_VALUE = 1.0;
    
    private static final double STD_ERROR = 1.0;
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final double THRESHOLD = 1e-7;
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int TIMES = 100;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public MSACDualConicRobustEstimatorTest() { }
    
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
        MSACDualConicRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new MSACDualConicRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with lines
        List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++) {
            lines.add(new Line2D());
        }
        
        estimator = new MSACDualConicRobustEstimator(lines);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Line2D> emptyLines = new ArrayList<>();
        estimator = null;
        try {
            estimator = new MSACDualConicRobustEstimator(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with listener
        DualConicRobustEstimatorListener listener =
                new DualConicRobustEstimatorListener() {

            @Override
            public void onEstimateStart(DualConicRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(DualConicRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    DualConicRobustEstimator estimator, int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    DualConicRobustEstimator estimator, float progress) { }
        };
        
        estimator = new MSACDualConicRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with listener and points
        estimator = new MSACDualConicRobustEstimator(listener, lines);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                MSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getLines(), lines);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACDualConicRobustEstimator(listener, 
                    emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                MSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
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
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
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
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
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
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
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
    public void testGetSetLines() throws LockedException {
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
        //check default value
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Line2D> lines = new ArrayList<>();
        for (int i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++) {
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
        List<Line2D> emptyLines = new ArrayList<>();
        try {
            estimator.setLines(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        MSACDualConicRobustEstimator estimator = 
                new MSACDualConicRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = 
                new double[DualConicRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException, 
            RobustEstimatorException, DualConicNotAvailableException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int t = 0;  t < TIMES; t++) {
            //instantiate a random circle
            Point2D center = new HomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE), 1.0);
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_POINT_VALUE / 2.0,
                    MAX_RANDOM_POINT_VALUE));
            
            Circle circle = new Circle(center, radius);
            Conic conic = circle.toConic();
            DualConic dualConic = conic.getDualConic();
            
            //compute lines in the dual conic locus
            int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
            double theta = (double)nLines / 360.0 * Math.PI / 180.0;
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Line2D> lines = new ArrayList<>();
            List<Line2D> linesWithError = new ArrayList<>();
            Point2D point;
            Line2D line, lineWithError;
            double[] directorVector = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for (int i = 0; i < nLines; i++) {
                double angle = theta * (double)i;
                point = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                
                line = new Line2D(point, directorVector);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                        line.getB() + errorB, line.getC());
                } else {
                    //inlier point
                    lineWithError = line;
                }

                lines.add(line);
                linesWithError.add(lineWithError);
                
                //check that point without error is within conic locus
                assertTrue(circle.isLocus(point, ABSOLUTE_ERROR));
                assertTrue(conic.isLocus(point, ABSOLUTE_ERROR));                
                assertTrue(dualConic.isLocus(line, ABSOLUTE_ERROR));
            }
    
            MSACDualConicRobustEstimator estimator = 
                    new MSACDualConicRobustEstimator(this, linesWithError);
            
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            DualConic dualConic2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check correctness of estimation by checking that all points
            //are within the estimated conic locus
            for (Line2D p : lines) {
                assertTrue(dualConic2.isLocus(p, ABSOLUTE_ERROR));
            }    
        }
    }

    @Override
    public void onEstimateStart(DualConicRobustEstimator estimator) {
        estimateStart++;
        checkLocked((MSACDualConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(DualConicRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((MSACDualConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(DualConicRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        checkLocked((MSACDualConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(DualConicRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        checkLocked((MSACDualConicRobustEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(MSACDualConicRobustEstimator estimator){
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
            estimator.setLines(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        }catch(Exception e){
            fail("LockedException expected but not thrown");
        }
    }
}
