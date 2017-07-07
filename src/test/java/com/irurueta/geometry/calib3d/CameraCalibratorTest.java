/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.CameraCalibrator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.calib3d;

import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CameraCalibratorTest {
    
    public CameraCalibratorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}
    
    @Test
    public void testCreate(){
        //test create with method
        
        //Error Optimization
        CameraCalibrator calibrator = CameraCalibrator.create(
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());

        //Alternating Calibrator
        calibrator = CameraCalibrator.create(
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        
        //test create with pattern, samples and method
        Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        List<CameraCalibratorSample> samples = 
                new ArrayList<CameraCalibratorSample>();
        samples.add(new CameraCalibratorSample());
        
        //Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples,
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        //Force IllegalArgumentException
        List<CameraCalibratorSample> emptySamples =
                new ArrayList<CameraCalibratorSample>();
        calibrator = null;
        try{
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(calibrator);

        //Alternating Calibrator
        calibrator = CameraCalibrator.create(pattern, samples,
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        //Force IllegalArgumentException
        calibrator = null;
        try{
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(calibrator);
        
        
        //test create with pattern, samples, quality scores and method
        double[] samplesQualityScores = new double[1];
        
        //Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples,
                samplesQualityScores, 
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertSame(calibrator.getSamplesQualityScores(), samplesQualityScores);
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        //Force IllegalArgumentException
        double[] shortSamplesQualityScores = new double[0];
        calibrator = null;
        try{
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    samplesQualityScores, 
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            calibrator = CameraCalibrator.create(pattern, samples, 
                    shortSamplesQualityScores, 
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(calibrator);

        //Alternating Calibrator
        calibrator = CameraCalibrator.create(pattern, samples,
                samplesQualityScores, 
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertSame(calibrator.getSamplesQualityScores(), samplesQualityScores);
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        calibrator = null;
        try{
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    samplesQualityScores, 
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            calibrator = CameraCalibrator.create(pattern, samples, 
                    shortSamplesQualityScores, 
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(calibrator);
        
    }
}
