/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PROSACDualConicRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 22, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.Conic;
import com.irurueta.geometry.DualConic;
import com.irurueta.geometry.DualConicNotAvailableException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class PROSACDualConicRobustEstimatorTest implements 
        DualConicRobustEstimatorListener{
    
    public static final int MIN_LINES = 500;
    public static final int MAX_LINES = 1000;
    
    public static final double MIN_RANDOM_POINT_VALUE = -1.0;
    public static final double MAX_RANDOM_POINT_VALUE = 1.0;
    
    public static final double STD_ERROR = 1.0;
    
    public static final double MIN_SCORE_ERROR = -0.3;
    public static final double MAX_SCORE_ERROR = 0.3;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double THRESHOLD = 1e-7;
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 100;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public PROSACDualConicRobustEstimatorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        PROSACDualConicRobustEstimator estimator;
        
        //test constructor without arguments
        estimator = new PROSACDualConicRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++){
            lines.add(new Line2D());
        }
        
        estimator = new PROSACDualConicRobustEstimator(lines);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        estimator = null;
        try{
            estimator = new PROSACDualConicRobustEstimator(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener
        DualConicRobustEstimatorListener listener =
                new DualConicRobustEstimatorListener(){

            @Override
            public void onEstimateStart(DualConicRobustEstimator estimator) {}

            @Override
            public void onEstimateEnd(DualConicRobustEstimator estimator) {}

            @Override
            public void onEstimateNextIteration(
                    DualConicRobustEstimator estimator, int iteration) {}

            @Override
            public void onEstimateProgressChange(
                    DualConicRobustEstimator estimator, float progress) {}
        };
        
        estimator = new PROSACDualConicRobustEstimator(listener);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        estimator = new PROSACDualConicRobustEstimator(listener, lines);
        
        //check correctness
        assertEquals(estimator.getThreshold(),
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualConicRobustEstimator(listener, 
                    emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with quality scores
        double[] qualityScores = new double[DualConicRobustEstimator.MINIMUM_SIZE];
        estimator = new PROSACDualConicRobustEstimator(qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try{
            estimator = new PROSACDualConicRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with lines and quality scores
        estimator = new PROSACDualConicRobustEstimator(lines, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualConicRobustEstimator(emptyLines, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACDualConicRobustEstimator(lines, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener and quality scores
        estimator = new PROSACDualConicRobustEstimator(listener, qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualConicRobustEstimator(listener, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);
        
        //test constructor with listener, points and quality scores
        estimator = new PROSACDualConicRobustEstimator(listener, lines, 
                qualityScores);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
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
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        estimator = null;
        try{
            estimator = new PROSACDualConicRobustEstimator(listener, emptyLines, 
                    qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator = new PROSACDualConicRobustEstimator(listener, lines, 
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(estimator);                  
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                PROSACDualConicRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(0.5);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetListener() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
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
    public void testGetSetProgressDelta() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(),
                DualConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                DualConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try{
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(),
                DualConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(1);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 1);
        
        //Force IllegalArgumentException
        try{
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetLines() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
        //check default value
        assertNull(estimator.getLines());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < DualConicRobustEstimator.MINIMUM_SIZE; i++){
            lines.add(new Line2D());
        }
        estimator.setLines(lines);
        
        //check correctness
        assertSame(estimator.getLines(), lines);
        assertFalse(estimator.isReady());
        
        //if we set quality scores, then estimator becomes ready
        double[] qualityScores = new double[lines.size()];
        estimator.setQualityScores(qualityScores);
        
        assertTrue(estimator.isReady());
        
        //clearing list makes instance not ready
        lines.clear();
        
        assertFalse(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Line2D> emptyLines = new ArrayList<Line2D>();
        try{
            estimator.setLines(emptyLines);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException{
        PROSACDualConicRobustEstimator estimator = 
                new PROSACDualConicRobustEstimator();
        
        assertNull(estimator.getQualityScores());
        
        double[] qualityScores = 
                new double[DualConicRobustEstimator.MINIMUM_SIZE];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        qualityScores = new double[1];
        try{
            estimator.setQualityScores(qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testEstimate() throws LockedException, NotReadyException, 
            RobustEstimatorException, DualConicNotAvailableException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0;  t < TIMES; t++){
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
            double[] qualityScores = new double[nLines];
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Line2D> lines = new ArrayList<Line2D>();
            List<Line2D> linesWithError = new ArrayList<Line2D>();
            Point2D point;
            Line2D line, lineWithError;
            double[] directorVector = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            for(int i = 0; i < nLines; i++){
                double angle = theta * (double)i;
                point = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                directorVector[0] = point.getInhomX() - center.getInhomX();
                directorVector[1] = point.getInhomY() - center.getInhomY();
                
                line = new Line2D(point, directorVector);
                double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, 
                        MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER){
                    //point is outlier
                    double errorA = errorRandomizer.nextDouble();
                    double errorB = errorRandomizer.nextDouble();
                    lineWithError = new Line2D(line.getA() + errorA,
                        line.getB() + errorB, line.getC());
                    
                    double error = Math.sqrt(errorA * errorA + errorB * errorB);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                }else{
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
    
            PROSACDualConicRobustEstimator estimator = 
                    new PROSACDualConicRobustEstimator(this, linesWithError,
                    qualityScores);
            
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
            for(Line2D p : lines){
                assertTrue(dualConic2.isLocus(p, ABSOLUTE_ERROR));
            }    
        }
    }
    
    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    @Override
    public void onEstimateStart(DualConicRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACDualConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(DualConicRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACDualConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(DualConicRobustEstimator estimator, 
            int iteration) {
        estimateNextIteration++;
        testLocked((PROSACDualConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(DualConicRobustEstimator estimator, 
            float progress) {
        estimateProgressChange++;
        testLocked((PROSACDualConicRobustEstimator)estimator);
    }
    
    private void testLocked(PROSACDualConicRobustEstimator estimator){
        try{
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.setLines(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            estimator.estimate();
            fail("LockedException expected but not thrown");
        }catch(LockedException e){
        }catch(Exception e){
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}