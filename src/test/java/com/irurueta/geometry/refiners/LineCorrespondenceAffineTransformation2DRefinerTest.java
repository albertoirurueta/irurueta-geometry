/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.refiners.LineCorrespondenceAffineTransformation2DRefiner
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 29, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACLineCorrespondenceAffineTransformation2DRobustEstimator;
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

public class LineCorrespondenceAffineTransformation2DRefinerTest implements 
        RefinerListener<AffineTransformation2D> {
    
    public static final double MIN_RANDOM_VALUE = -1000.0;
    public static final double MAX_RANDOM_VALUE = 1000.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int MIN_LINES = 50;
    public static final int MAX_LINES = 100;
    
    public static final int PERCENTAGE_OUTLIER = 20;
    
    public static final double STD_ERROR = 100.0;
    public static final double THRESHOLD = 1e-6;
    
    public static final int TIMES = 1000;
    
    private int mRefineStart;
    private int mRefineEnd;        
    
    public LineCorrespondenceAffineTransformation2DRefinerTest() { }
    
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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                createRobustEstimator();
        AffineTransformation2D transformation = estimator.estimate();
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        double[] residuals = inliersData.getResiduals();
        int numInliers = inliersData.getNumInliers();
        double refinementStandardDeviation = estimator.getThreshold();
        List<Line2D> samples1 = estimator.getInputLines();
        List<Line2D> samples2 = estimator.getOutputLines();
        
        assertNotNull(transformation);
        assertNotNull(inliersData);

        //test empty constructor
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();
        
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
        refiner = new LineCorrespondenceAffineTransformation2DRefiner(
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
        
        
        refiner = new LineCorrespondenceAffineTransformation2DRefiner(
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
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();
        
        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();
        
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
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        //check default value
        assertNull(refiner.getSamples1());
        
        //set new value
        List<Line2D> samples1 = new ArrayList<Line2D>();
        refiner.setSamples1(samples1);
        
        //check correctness
        assertSame(refiner.getSamples1(), samples1);
    }
    
    @Test
    public void testGetSetSamples2() throws LockedException {
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        //check default value
        assertNull(refiner.getSamples2());
        
        //set new value
        List<Line2D> samples2 = new ArrayList<Line2D>();
        refiner.setSamples2(samples2);
        
        //check correctness
        assertSame(refiner.getSamples2(), samples2);        
    }
    
    @Test
    public void testGetSetInliers() throws LockedException, AlgebraException, 
            NotReadyException, RobustEstimatorException {
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        BitSet inliers = inliersData.getInliers();
        
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();
        
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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        double[] residuals = inliersData.getResiduals();
        
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();
        
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
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        int numInliers = inliersData.getNumInliers();
        
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();
        
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
    public void testSetInliersData() throws LockedException, AlgebraException, 
            NotReadyException, RobustEstimatorException {
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator = 
                createRobustEstimator();
        
        assertNotNull(estimator.estimate());
        InliersData inliersData = estimator.getInliersData();
        
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

        //check default value
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        AffineTransformation2D transformation = new AffineTransformation2D();
        refiner.setInitialEstimation(transformation);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), transformation);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        LineCorrespondenceAffineTransformation2DRefiner refiner =
                new LineCorrespondenceAffineTransformation2DRefiner();

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
            RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator = 
                    createRobustEstimator();
            
            AffineTransformation2D transformation = estimator.estimate();
            InliersData inliersData = estimator.getInliersData();
            double refineStandardDeviation = estimator.getThreshold();
            List<Line2D> samples1 = estimator.getInputLines();
            List<Line2D> samples2 = estimator.getOutputLines();            
            
            LineCorrespondenceAffineTransformation2DRefiner refiner =
                    new LineCorrespondenceAffineTransformation2DRefiner(
                    transformation, true, inliersData, samples1, samples2, 
                    refineStandardDeviation);
            refiner.setListener(this);
            
            AffineTransformation2D result1 = new AffineTransformation2D();
            
            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);
            
            if (!refiner.refine(result1)) {
                continue;
            }
            
            AffineTransformation2D result2 = refiner.refine();
            
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
    
    
    private RANSACLineCorrespondenceAffineTransformation2DRobustEstimator 
            createRobustEstimator() throws AlgebraException, LockedException {
            
        AffineTransformation2D transformation = createTransformation();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //generate random lines
        int nLines = randomizer.nextInt(MIN_LINES, MAX_LINES);
        
        List<Line2D> inputLines = new ArrayList<Line2D>();
        List<Line2D> outputLinesWithError = new ArrayList<Line2D>();
        Line2D inputLine, outputLine, outputLineWithError;
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);                    
        for (int i = 0; i < nLines; i++) {
            //generate input point
            inputLine = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            outputLine = transformation.transformAndReturnNew(inputLine);
            
            if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                //line is outlier
                double errorA = errorRandomizer.nextDouble();
                double errorB = errorRandomizer.nextDouble();
                double errorC = errorRandomizer.nextDouble();
                outputLineWithError = new Line2D(outputLine.getA() + errorA,
                        outputLine.getB() + errorB,
                        outputLine.getC() + errorC);
            } else {
                //inlier line (without error)
                outputLineWithError = outputLine;
            }
            
            inputLines.add(inputLine);
            outputLinesWithError.add(outputLineWithError);
        }
        
        RANSACLineCorrespondenceAffineTransformation2DRobustEstimator estimator =
                new RANSACLineCorrespondenceAffineTransformation2DRobustEstimator(
                inputLines, outputLinesWithError);
        
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);
        
        return estimator;
    }
        
    private AffineTransformation2D createTransformation() 
            throws AlgebraException {
            
        Matrix A = null;
        do {
            //ensure A matrix is invertible
            A = Matrix.createWithUniformRandomValues(
                    AffineTransformation2D.INHOM_COORDS, 
                    AffineTransformation2D.INHOM_COORDS, -1.0, 1.0);
            double norm = Utils.normF(A);
            //normalize T to increase accuracy
            A.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(A) < AffineTransformation2D.INHOM_COORDS);
            
        double[] translation = new double[AffineTransformation2D.INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);
        
        return new AffineTransformation2D(A, translation);
    }
    
    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    @Override
    public void onRefineStart(Refiner<AffineTransformation2D> refiner, 
            AffineTransformation2D initialEstimation) {
        mRefineStart++;
        checkLocked((LineCorrespondenceAffineTransformation2DRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<AffineTransformation2D> refiner, 
            AffineTransformation2D initialEstimation, 
            AffineTransformation2D result, boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((LineCorrespondenceAffineTransformation2DRefiner)refiner);
    }

    private void checkLocked(
            LineCorrespondenceAffineTransformation2DRefiner refiner) {
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
