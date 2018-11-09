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
package com.irurueta.ar.calibration;

import com.irurueta.ar.calibration.estimators.*;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.*;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class AlternatingCameraCalibratorTest 
        implements CameraCalibratorListener {
    
    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;
    
    private static final int INHOM_3D_COORDS = 3;
    
    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = -10.0;
    
    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;
    
    private static final int MIN_NUM_SAMPLES = 100;
    private static final int MAX_NUM_SAMPLES = 500;
        
    private static final int TIMES = 100;
    
    private static final double STD_ERROR = 10.0;
    private static final int PERCENTAGE_OUTLIERS = 20;
    
    private static final double MIN_DISTORTION_PARAM_VALUE = -1e-3;
    private static final double MAX_DISTORTION_PARAM_VALUE = 1e-3;
    
    private static final double REAL_IAC_THRESHOLD = 1e-8;
    
    private int calibrateStart;
    private int calibrateEnd;
    private int calibrateProgressChange;
    private int intrinsicParametersEstimationStarts;
    private int intrinsicParametersEstimationEnds;
    private int radialDistortionEstimationStarts;
    private int radialDistortionEstimationEnds;
    
    public AlternatingCameraCalibratorTest() { }
    
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
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default values
        assertEquals(calibrator.getMaxIterations(),
                AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS);
        assertEquals(calibrator.getConvergenceThreshold(),
                AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.getDistortionMethod(), 
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertEquals(calibrator.getDistortionEstimator().getMethod(),
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertTrue(calibrator.getDistortionEstimatorThreshold() > 0);
        assertEquals(calibrator.getDistortionEstimatorConfidence(),
                calibrator.getDistortionEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getDistortionEstimatorMaxIterations(),
                calibrator.getDistortionEstimator().getMaxIterations());
        assertEquals(calibrator.getMethod(), 
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getHomographyQualityScores());
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
        
        //test constructor with pattern and samples
        Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        List<CameraCalibratorSample> samples = 
                new ArrayList<>();
        samples.add(new CameraCalibratorSample());
        
        assertEquals(calibrator.getIACEstimator().
                getMinNumberOfRequiredHomographies(), 1);
        
        calibrator = new AlternatingCameraCalibrator(pattern, samples);
        
        //check default values
        assertEquals(calibrator.getMaxIterations(),
                AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS);
        assertEquals(calibrator.getConvergenceThreshold(),
                AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.getDistortionMethod(), 
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertEquals(calibrator.getDistortionEstimator().getMethod(),
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertTrue(calibrator.getDistortionEstimatorThreshold() > 0);
        assertEquals(calibrator.getDistortionEstimatorConfidence(),
                calibrator.getDistortionEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getDistortionEstimatorMaxIterations(),
                calibrator.getDistortionEstimator().getMaxIterations());
        assertEquals(calibrator.getMethod(), 
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getHomographyQualityScores());
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
                new ArrayList<>();
        calibrator = null;
        try {
            calibrator = new AlternatingCameraCalibrator(pattern, emptySamples);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(calibrator);
        
        //test constructor with pattern, samples and quality scores
        double[] samplesQualityScores = new double[1];
        calibrator = new AlternatingCameraCalibrator(pattern, samples,
                samplesQualityScores);
        
        //check default values
        assertEquals(calibrator.getMaxIterations(),
                AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS);
        assertEquals(calibrator.getConvergenceThreshold(),
                AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.getDistortionMethod(), 
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertEquals(calibrator.getDistortionEstimator().getMethod(),
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertTrue(calibrator.getDistortionEstimatorThreshold() > 0);
        assertEquals(calibrator.getDistortionEstimatorConfidence(),
                calibrator.getDistortionEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getDistortionEstimatorMaxIterations(),
                calibrator.getDistortionEstimator().getMaxIterations());
        assertEquals(calibrator.getMethod(), 
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertSame(calibrator.getSamplesQualityScores(), samplesQualityScores);
        assertNull(calibrator.getHomographyQualityScores());
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
        try {
            calibrator = new AlternatingCameraCalibrator(pattern, 
                    emptySamples, samplesQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            calibrator = new AlternatingCameraCalibrator(pattern, samples,
                    shortSamplesQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(calibrator);        
    }
    
    @Test
    public void testGetSetPattern() throws LockedException {
        AlternatingCameraCalibrator calibrator = 
                new AlternatingCameraCalibrator();
        
        //check default value
        assertNull(calibrator.getPattern());
        
        //set new value
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        calibrator.setPattern(pattern);
        
        //check correctness
        assertSame(calibrator.getPattern(), pattern);
    }
    
    @Test
    public void testGetSetSamples() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertNull(calibrator.getSamples());
        
        //set new value
        List<CameraCalibratorSample> samples = 
                new ArrayList<>();
        samples.add(new CameraCalibratorSample());
        
        calibrator.setSamples(samples);
        
        //check correctness
        assertSame(calibrator.getSamples(), samples);        
        
        //Force IllegalArgumentException
        List<CameraCalibratorSample> emptySamples = 
                new ArrayList<>();
        try {
            calibrator.setSamples(emptySamples);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetSamplesQualityScores() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertNull(calibrator.getSamplesQualityScores());
        
        //set new value
        double[] sampleQualityScores = new double[1];
        calibrator.setSamplesQualityScores(sampleQualityScores);
        
        //check correctness
        assertSame(calibrator.getSamplesQualityScores(), sampleQualityScores);
        
        //Force IllegalArgumentException
        double[] emptyQualityScores = new double[0];
        try {
            calibrator.setSamplesQualityScores(emptyQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetEstimateRadialDistortion() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        
        //set new value
        calibrator.setEstimateRadialDistortion(
                !CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        
        //check correctness
        assertEquals(calibrator.getEstimateRadialDistortion(),
                !CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
    }
    
    @Test
    public void testGetSetHomographyMethod() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getHomographyMethod(),
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getHomographyEstimator().getMethod(),
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        
        double threshold = calibrator.getHomographyEstimatorThreshold();
        double confidence = calibrator.getHomographyEstimatorConfidence();
        int maxIterations = calibrator.getHomographyEstimatorMaxIterations();
        
        //set new value
        calibrator.setHomographyMethod(RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertEquals(calibrator.getHomographyMethod(), 
                RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getHomographyEstimator().getMethod(),
                RobustEstimatorMethod.RANSAC);
        
        assertEquals(calibrator.getHomographyEstimatorThreshold(), threshold, 
                0.0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(), confidence,
                0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(), 
                maxIterations);
    }
    
    @Test
    public void testGetSetImageOfAbsoluteConicMethod() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.getIACEstimator().getMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        
        double threshold = calibrator.getIACEstimatorThreshold();
        double confidence = calibrator.getIACEstimatorConfidence();
        int maxIterations = calibrator.getIACEstimatorMaxIterations();
        boolean zeroSkewness = calibrator.isZeroSkewness();
        boolean principalPointAtOrigin = calibrator.isPrincipalPointAtOrigin();
        boolean focalDistanceAspectRatioKnown = 
                calibrator.isFocalDistanceAspectRatioKnown();
        double focalDistanceAspectRatio = 
                calibrator.getFocalDistanceAspectRatio();
        
        //set new value
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getIACEstimator().getMethod(),
                RobustEstimatorMethod.RANSAC);
        
        assertEquals(calibrator.getIACEstimatorThreshold(), threshold, 0.0);
        assertEquals(calibrator.getIACEstimatorConfidence(), confidence, 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(), maxIterations);
        assertEquals(calibrator.isZeroSkewness(), zeroSkewness);
        assertEquals(calibrator.isPrincipalPointAtOrigin(), 
                principalPointAtOrigin);
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                focalDistanceAspectRatioKnown);
        assertEquals(calibrator.getFocalDistanceAspectRatio(), 
                focalDistanceAspectRatio, 0.0);
    }
    
    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        AlternatingCameraCalibrator calibrator = 
                new AlternatingCameraCalibrator();
        
        //check default value
        boolean zeroSkewness = calibrator.isZeroSkewness();
        assertEquals(zeroSkewness, 
                calibrator.getIACEstimator().isZeroSkewness());
        
        //set new value
        calibrator.setZeroSkewness(!zeroSkewness);
        
        //check correctness
        assertEquals(calibrator.isZeroSkewness(), !zeroSkewness);
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), 
                !zeroSkewness);
    }
    
    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        boolean principalPointAtOrigin = calibrator.isPrincipalPointAtOrigin();
        assertEquals(principalPointAtOrigin, 
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        
        //set new value
        calibrator.setPrincipalPointAtOrigin(!principalPointAtOrigin);
        
        //check correctness
        assertEquals(calibrator.isPrincipalPointAtOrigin(), 
                !principalPointAtOrigin);
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                !principalPointAtOrigin);
    }
    
    @Test
    public void testIsSetFocalDistanceAspectRatioKnown() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        boolean focalDistanceAspectRatioKnown = 
                calibrator.isFocalDistanceAspectRatioKnown();
        assertEquals(focalDistanceAspectRatioKnown,
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        
        //set new value
        calibrator.setFocalDistanceAspectRatioKnown(
                !focalDistanceAspectRatioKnown);
        
        //check correctness
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(), 
                !focalDistanceAspectRatioKnown);
        assertEquals(calibrator.getIACEstimator().
                isFocalDistanceAspectRatioKnown(), 
                !focalDistanceAspectRatioKnown);
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        double focalDistanceAspectRatio = 
                calibrator.getFocalDistanceAspectRatio();
        assertEquals(focalDistanceAspectRatio,
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        
        //set new value
        calibrator.setFocalDistanceAspectRatio(0.5);
        
        //check correctness
        assertEquals(calibrator.getFocalDistanceAspectRatio(), 0.5, 0.0);
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getProgressDelta(),
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        calibrator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(calibrator.getProgressDelta(), 0.5f, 0.0);
    }
    
    @Test
    public void testGetSetHomographyEstimatorThreshold() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //set RANSAC homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.RANSAC);
        
        //check default value
        double threshold = calibrator.getHomographyEstimatorThreshold();
        assertEquals(threshold, 
                ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setHomographyEstimatorThreshold(1.0);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 1.0, 0.0);
        assertEquals(1.0, 
                ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getThreshold(), 0.0);
        
        //set LMedS homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
        
        //check default value
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 1.0, 0.0);
        assertEquals(1.0,
                ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getStopThreshold(), 0.0);
        
        //set new value
        calibrator.setHomographyEstimatorThreshold(2.0);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 2.0, 0.0);
        assertEquals(2.0,
                ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getStopThreshold(), 0.0);
        
        //set MSAC homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.MSAC);
        
        //check default value
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 2.0, 0.0);
        assertEquals(2.0,
                ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setHomographyEstimatorThreshold(3.0);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 3.0, 0.0);
        assertEquals(3.0,
                ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getThreshold(), 0.0);
        
        //set PROSAC homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.PROSAC);
        
        //check default value
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 3.0, 0.0);
        assertEquals(3.0,
                ((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setHomographyEstimatorThreshold(4.0);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 4.0, 0.0);
        assertEquals(4.0,
                ((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getThreshold(), 0.0);
        
        //set PROMedS homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.PROMedS);
        
        //check default value
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 4.0, 0.0);
        assertEquals(4.0,
                ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getStopThreshold(), 0.0);
        
        //set new value
        calibrator.setHomographyEstimatorThreshold(5.0);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorThreshold(), 5.0, 0.0);
        assertEquals(5.0,
                ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getStopThreshold(), 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setHomographyEstimatorThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetHomographyEstimatorConfidence() 
            throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        
        //set new value
        calibrator.setHomographyEstimatorConfidence(0.5);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setHomographyEstimatorConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            calibrator.setHomographyEstimatorConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetHomographyEstimatorMaxIterations()
            throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        
        //set new value
        calibrator.setHomographyEstimatorMaxIterations(10);
        
        //check correctness
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            calibrator.setHomographyEstimatorMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetIACEstimatorThreshold() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //set RANSAC homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.RANSAC);
        
        //check default value
        double threshold = calibrator.getIACEstimatorThreshold();
        assertEquals(threshold, ((RANSACImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setIACEstimatorThreshold(1.0);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorThreshold(), 1.0, 0.0);
        assertEquals(1.0, 
                ((RANSACImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getThreshold(), 0.0);
        
        //set LMedS homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.LMedS);
        
        //check default value
        assertEquals(calibrator.getIACEstimatorThreshold(), 1.0, 0.0);
        assertEquals(1.0,
                ((LMedSImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getStopThreshold(), 0.0);
        
        //set new value
        calibrator.setIACEstimatorThreshold(2.0);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorThreshold(), 2.0, 0.0);
        assertEquals(2.0,
                ((LMedSImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getStopThreshold(), 0.0);
        
        //set MSAC homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.MSAC);
        
        //check default value
        assertEquals(calibrator.getIACEstimatorThreshold(), 2.0, 0.0);
        assertEquals(2.0,
                ((MSACImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setIACEstimatorThreshold(3.0);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorThreshold(), 3.0, 0.0);
        assertEquals(3.0,
                ((MSACImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getThreshold(), 0.0);
        
        //set PROSAC homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
        
        //check default value
        assertEquals(calibrator.getIACEstimatorThreshold(), 3.0, 0.0);
        assertEquals(3.0,
                ((PROSACImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setIACEstimatorThreshold(4.0);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorThreshold(), 4.0, 0.0);
        assertEquals(4.0,
                ((PROSACImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getThreshold(), 0.0);
        
        //set PROMedS homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROMedS);
        
        //check default value
        assertEquals(calibrator.getIACEstimatorThreshold(), 4.0, 0.0);
        assertEquals(4.0,
                ((PROMedSImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getStopThreshold(), 0.0);
        
        //set new value
        calibrator.setIACEstimatorThreshold(5.0);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorThreshold(), 5.0, 0.0);
        assertEquals(5.0,
                ((PROMedSImageOfAbsoluteConicRobustEstimator)
                calibrator.getIACEstimator()).getStopThreshold(), 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setIACEstimatorThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetIACEstimatorConfidence() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        
        //set new value
        calibrator.setIACEstimatorConfidence(0.5);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setIACEstimatorConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            calibrator.setIACEstimatorConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetIACEstimatorMaxIterations()
            throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        
        //set new value
        calibrator.setIACEstimatorMaxIterations(10);
        
        //check correctness
        assertEquals(calibrator.getIACEstimatorMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            calibrator.setIACEstimatorMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertNull(calibrator.getListener());
        
        //set new value
        calibrator.setListener(this);
        
        //check correctness
        assertSame(calibrator.getListener(), this);
    }
        
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getMaxIterations(), 
                AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        calibrator.setMaxIterations(5);
        
        //check correctness
        assertEquals(calibrator.getMaxIterations(), 5);
        
        //Force IllegalArgumentException
        try {
            calibrator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetConvergenceThreshold() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getConvergenceThreshold(),
                AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        
        //set new value
        calibrator.setConvergenceThreshold(1.0);
        
        //check correctness
        assertEquals(calibrator.getConvergenceThreshold(), 1.0, 0.0);
    }
    
    @Test
    public void testGetSetDistortionMethod() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getDistortionMethod(),
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        assertEquals(calibrator.getDistortionEstimator().getMethod(),
                AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD);
        
        double threshold = calibrator.getDistortionEstimatorThreshold();
        double confidence = calibrator.getDistortionEstimatorConfidence();
        int maxIterations = calibrator.getDistortionEstimatorMaxIterations();
        
        //set new value
        calibrator.setDistortionMethod(RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertEquals(calibrator.getDistortionMethod(), 
                RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getDistortionEstimator().getMethod(),
                RobustEstimatorMethod.LMedS);
        
        assertEquals(calibrator.getDistortionEstimatorThreshold(), threshold, 
                0.0);
        assertEquals(calibrator.getDistortionEstimatorConfidence(), confidence,
                0.0);
        assertEquals(calibrator.getDistortionEstimatorMaxIterations(), 
                maxIterations);
    }
    
    @Test
    public void testGetSetDistortionEstimatorThreshold() throws LockedException {
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //set RANSAC method
        calibrator.setDistortionMethod(RobustEstimatorMethod.RANSAC);
        
        //check default value
        double threshold = calibrator.getDistortionEstimatorThreshold();
        assertEquals(threshold,
                ((RANSACRadialDistortionRobustEstimator)calibrator.
                        getDistortionEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setDistortionEstimatorThreshold(1.0);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 1.0, 0.0);
        assertEquals(1.0,
                ((RANSACRadialDistortionRobustEstimator)calibrator.
                        getDistortionEstimator()).getThreshold(), 0.0);

        //set LMedS method
        calibrator.setDistortionMethod(RobustEstimatorMethod.LMedS);
        
        //check default value
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 1.0, 0.0);
        assertEquals(1.0,
                ((LMedSRadialDistortionRobustEstimator)calibrator.
                        getDistortionEstimator()).getStopThreshold(), 0.0);
        
        //set new value
        calibrator.setDistortionEstimatorThreshold(2.0);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 2.0, 0.0);
        assertEquals(2.0,
                ((LMedSRadialDistortionRobustEstimator)calibrator.
                        getDistortionEstimator()).getStopThreshold(), 0.0);
        
        //set MSAC method
        calibrator.setDistortionMethod(RobustEstimatorMethod.MSAC);
        
        //check default value
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 2.0, 0.0);
        assertEquals(2.0,
                ((MSACRadialDistortionRobustEstimator)calibrator.
                getDistortionEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setDistortionEstimatorThreshold(3.0);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 3.0, 0.0);
        assertEquals(3.0,
                ((MSACRadialDistortionRobustEstimator)calibrator.
                getDistortionEstimator()).getThreshold(), 0.0);
        
        //set PROSAC method
        calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
        
        //check default value
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 3.0, 0.0);
        assertEquals(3.0,
                ((PROSACRadialDistortionRobustEstimator)calibrator.
                getDistortionEstimator()).getThreshold(), 0.0);
        
        //set new value
        calibrator.setDistortionEstimatorThreshold(4.0);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 4.0, 0.0);
        assertEquals(4.0,
                ((PROSACRadialDistortionRobustEstimator)calibrator.
                getDistortionEstimator()).getThreshold(), 0.0);
        
        //set PROMedS method        
        calibrator.setDistortionMethod(RobustEstimatorMethod.PROMedS);
        
        //check default value
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 4.0, 0.0);
        assertEquals(4.0,
                ((PROMedSRadialDistortionRobustEstimator)calibrator.
                getDistortionEstimator()).getStopThreshold(), 0.0);
        
        //set new value
        calibrator.setDistortionEstimatorThreshold(5.0);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorThreshold(), 5.0, 0.0);
        assertEquals(5.0,
                ((PROMedSRadialDistortionRobustEstimator)calibrator.
                getDistortionEstimator()).getStopThreshold(), 0.0);        
        
        //Force IllegalArgumentException
        try {
            calibrator.setDistortionEstimatorThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetDistortionEstimatorConfidence() 
            throws LockedException {
        
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();

        //check default value
        assertEquals(calibrator.getDistortionEstimatorConfidence(),
                calibrator.getDistortionEstimator().getConfidence(), 0.0);
        
        //set new value
        calibrator.setDistortionEstimatorConfidence(0.5);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setDistortionEstimatorConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            calibrator.setDistortionEstimatorConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetDistortionEstimatorMaxIterations() 
            throws LockedException {
        
        AlternatingCameraCalibrator calibrator =
                new AlternatingCameraCalibrator();
        
        //check default value
        assertEquals(calibrator.getDistortionEstimatorMaxIterations(),
                calibrator.getDistortionEstimator().getMaxIterations());
        
        //set new value
        calibrator.setDistortionEstimatorMaxIterations(10);
        
        //check correctness
        assertEquals(calibrator.getDistortionEstimatorMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            calibrator.setDistortionEstimatorMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    @SuppressWarnings("all")
    public void testCalibrateCirclesPatternNoDistortion() 
            throws LockedException, CalibrationException, NotReadyException {
        
        long startTime = System.currentTimeMillis();
        
        Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        
        List<Point2D> patternPoints = pattern.getIdealPoints();

        //assume that pattern points are located on a 3D plane
        //(for instance Z = 0), but can be really any plane
        List<Point3D> points3D = new ArrayList<>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
        }
                
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;                        
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add random noise to projected points with a certain outlier
                //proportion
                
                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }
            
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(false);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROMedS);                        
            
            
            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            calibrator.calibrate();
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();
            
            assertEquals(intrinsic.getHorizontalFocalLength(), 
                    intrinsic2.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic.getVerticalFocalLength(),
                    intrinsic2.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            assertNull(calibrator.getDistortion());
            
            //check camera poses were not estimated (because distortion was not
            //estimated)
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNull(samples.get(i).getCamera());
                    assertNull(samples.get(i).getCameraCenter());
                    assertNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            assertEquals(estimatedCams, 0);
        }
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        String msg = "Circles pattern - No distortion" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " avg min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError + 
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);
        
    }

    @Test
    @SuppressWarnings("all")
    public void testCalibrateQRPatternNoDistortion() 
            throws LockedException, CalibrationException, NotReadyException {
        
        long startTime = System.currentTimeMillis();
        
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        
        List<Point2D> patternPoints = pattern.getIdealPoints();

        //assume that pattern points are located on a 3D plane
        //(for instance Z = 0), but can be really any plane
        List<Point3D> points3D = new ArrayList<>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
        }
                
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add random noise to projected points with a certain outlier
                //proportion
                
                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }
            
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(false);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROMedS);                        
            
            
            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            calibrator.calibrate();
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();

            if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength()) >
                    3.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic.getHorizontalFocalLength(), 
                    intrinsic2.getHorizontalFocalLength(), 
                    3.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength()) >
                    3.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic.getVerticalFocalLength(),
                    intrinsic2.getVerticalFocalLength(), 
                    3.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            assertNull(calibrator.getDistortion());
            
            //check camera poses were not estimated (because distortion was not
            //estimated)
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNull(samples.get(i).getCamera());
                    assertNull(samples.get(i).getCameraCenter());
                    assertNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            assertEquals(estimatedCams, 0);

            numValid++;
        }

        assertTrue(numValid > 0);
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;

        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        String msg = "QR pattern - No distortion" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " avg min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError +
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);
        
    }
    
    @Test
    @SuppressWarnings("all")
    public void testCalibrateCirclesPatternDistortion() 
            throws LockedException, NotReadyException,
            NotSupportedException, DistortionException {
        
        long startTime = System.currentTimeMillis();
        
        Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        
        List<Point2D> patternPoints = pattern.getIdealPoints();

        //assume that pattern points are located on a 3D plane
        //(for instance Z = 0), but can be really any plane
        List<Point3D> points3D = new ArrayList<>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
        }
                
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double avgK1Error = 0.0;
        double avgK2Error = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double minK1Error = Double.MAX_VALUE;
        double minK2Error = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double maxK1Error = -Double.MAX_VALUE;
        double maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError, k1Error, k2Error;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create distortion
            double k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            //square k2 so that we ensure it is smaller than k1 (typically k1
            //is the dominant term)
            double signk2 = Math.signum(k2);
            k2 = signk2 * k2 * k2;
            
            RadialDistortion distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                                
                //add random noise to projected points with a certain outlier
                //proportion
                
                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }
                        
            //distort projected pattern points using created distortion 
            //(we modify the samples)
            //To avoid optimization exceptions while computing distortions,
            //we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (int i = 0; i < numSamples; i++) {
                sample = samples.get(i);
                                
                sample.setSampledMarkers(distortion.distort(
                        sample.getSampledMarkers()));                
            }
            
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            
            
            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            try {
                calibrator.calibrate();
            } catch (CalibrationException e) {
                continue;
            }
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();
            
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            RadialDistortion distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());
            
            avgK1Error += k1Error;
            avgK2Error += k2Error;
            
            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }
            
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            assertTrue(estimatedCams > 0);
            numValid++;
        }
        
        assertTrue(numValid > 0);
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        
        avgK1Error /= (double)TIMES;
        avgK2Error /= (double)TIMES;
        
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgSkewnessError, 0.0, ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, ABSOLUTE_ERROR);
        
        //check that average distortion parameters error is small enough

        String msg = "Circles pattern - With distortion" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +                
                avgVerticalPrincipalPointError +
                " avg distortion K1 error: " + avgK1Error +
                " avg distortion K2 error: " + avgK2Error +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " min distortion K1 error: " + minK1Error +
                " min distortion K2 error: " + minK2Error +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError +
                " max distortion K1 error: " + maxK1Error +
                " max distortion K2 error: " + maxK2Error +
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);        
    }

    @Test
    @SuppressWarnings("all")
    public void testCalibrateQRPatternDistortion() 
            throws LockedException, CalibrationException, NotReadyException,
            NotSupportedException, DistortionException {
        
        long startTime = System.currentTimeMillis();
        
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        
        List<Point2D> patternPoints = pattern.getIdealPoints();

        //assume that pattern points are located on a 3D plane
        //(for instance Z = 0), but can be really any plane
        List<Point3D> points3D = new ArrayList<>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
        }
                
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double avgK1Error = 0.0;
        double avgK2Error = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double minK1Error = Double.MAX_VALUE;
        double minK2Error = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double maxK1Error = -Double.MAX_VALUE;
        double maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError, k1Error, k2Error;
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create distortion
            double k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            //square k2 so that we ensure it is smaller than k1 (typically k1
            //is the dominant term)
            double signk2 = Math.signum(k2);
            k2 = signk2*k2*k2;
            
            RadialDistortion distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                                
                //add random noise to projected points with a certain outlier
                //proportion
                
                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }
                        
            //distort projected pattern points using created distortion 
            //(we modify the samples)
            //To avoid optimization exceptions while computing distortions,
            //we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (int i = 0; i < numSamples; i++) {
                sample = samples.get(i);
                                
                sample.setSampledMarkers(distortion.distort(
                        sample.getSampledMarkers()));                
            }
            
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);
            
            
            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            calibrator.calibrate();
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();
            
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            RadialDistortion distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());
            
            avgK1Error += k1Error;
            avgK2Error += k2Error;
            
            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }
            
            //check camera poses were not estimated (because distortion was not
            //estimated)
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            assertTrue(estimatedCams > 0);
        }
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        
        avgK1Error /= (double)TIMES;
        avgK2Error /= (double)TIMES;
        
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        //check that average distortion parameters error is small enough

        String msg = "QR pattern - With distortion" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +                
                avgVerticalPrincipalPointError +
                " avg distortion K1 error: " + avgK1Error +
                " avg distortion K2 error: " + avgK2Error +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " min distortion K1 error: " + minK1Error +
                " min distortion K2 error: " + minK2Error +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError +
                " max distortion K1 error: " + maxK1Error +
                " max distortion K2 error: " + maxK2Error +
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);        
    }

    @Test
    @SuppressWarnings("all")
    public void testCalibrateCirclesPatternDistortionAndOutliers() 
            throws LockedException, NotReadyException,
            NotSupportedException, DistortionException {
        
        long startTime = System.currentTimeMillis();
        
        Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        
        List<Point2D> patternPoints = pattern.getIdealPoints();

        //assume that pattern points are located on a 3D plane
        //(for instance Z = 0), but can be really any plane
        List<Point3D> points3D = new ArrayList<>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
        }
                
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double avgK1Error = 0.0;
        double avgK2Error = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double minK1Error = Double.MAX_VALUE;
        double minK2Error = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double maxK1Error = -Double.MAX_VALUE;
        double maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError, k1Error, k2Error;
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create distortion
            double k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            //square k2 so that we ensure it is smaller than k1 (typically k1
            //is the dominant term)
            double signk2 = Math.signum(k2);
            k2 = signk2 * k2 * k2;
            
            
            RadialDistortion distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add outliers
                List<Point2D> projectedPatternPointsWithError = 
                        new ArrayList<>();
                for (Point2D p : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        //outlier
                        double errorX = errorRandomizer.nextDouble();
                        double errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(
                                new HomogeneousPoint2D(p.getInhomX() + errorX,
                                p.getInhomY() + errorY, 1.0));
                    } else {
                        //inlier
                        projectedPatternPointsWithError.add(p);
                    }
                }
                                                                
                samples.add(new CameraCalibratorSample(
                        projectedPatternPointsWithError));
            }
                        
            //distort projected pattern points using created distortion 
            //(we modify the samples)
            //To avoid optimization exceptions while computing distortions,
            //we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (int i = 0; i < numSamples; i++) {
                sample = samples.get(i);
                                
                sample.setSampledMarkers(distortion.distort(
                        sample.getSampledMarkers()));
            }
            
                        
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);
            
            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            try {
                calibrator.calibrate();
            } catch (CalibrationException e) {
                reset();
                continue;
            }
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();
            
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            RadialDistortion distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());
            
            avgK1Error += k1Error;
            avgK2Error += k2Error;
            
            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }
            
            //check camera poses were not estimated (because distortion was not
            //estimated)
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            assertTrue(estimatedCams > 0);
        }
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        
        avgK1Error /= (double)TIMES;
        avgK2Error /= (double)TIMES;
        
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        //check that average distortion parameters error is small enough

        String msg = "Circles pattern - With distortion and outliers" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +                
                avgVerticalPrincipalPointError +
                " avg distortion K1 error: " + avgK1Error +
                " avg distortion K2 error: " + avgK2Error +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " min distortion K1 error: " + minK1Error +
                " min distortion K2 error: " + minK2Error +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError +
                " max distortion K1 error: " + maxK1Error +
                " max distortion K2 error: " + maxK2Error +
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);        
    }

    @Test
    @SuppressWarnings("all")
    public void testCalibrateQRPatternDistortionAndOutliers() 
            throws LockedException, CalibrationException, NotReadyException, 
            NotSupportedException, DistortionException {
        
        long startTime = System.currentTimeMillis();
        
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        
        List<Point2D> patternPoints = pattern.getIdealPoints();

        //assume that pattern points are located on a 3D plane
        //(for instance Z = 0), but can be really any plane
        List<Point3D> points3D = new ArrayList<>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
        }
                
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double avgK1Error = 0.0;
        double avgK2Error = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double minK1Error = Double.MAX_VALUE;
        double minK2Error = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double maxK1Error = -Double.MAX_VALUE;
        double maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError, k1Error, k2Error;
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create distortion
            double k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            //square k2 so that we ensure it is smaller than k1 (typically k1
            //is the dominant term)
            k2 = k2 * k2;
            
            RadialDistortion distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);            
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add outliers
                List<Point2D> projectedPatternPointsWithError = 
                        new ArrayList<>();
                for (Point2D p : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        //outlier
                        double errorX = errorRandomizer.nextDouble();
                        double errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(
                                new HomogeneousPoint2D(p.getInhomX() + errorX,
                                p.getInhomY() + errorY, 1.0));
                    } else {
                        //inlier
                        projectedPatternPointsWithError.add(p);
                    }
                }
                                
                samples.add(new CameraCalibratorSample(
                        projectedPatternPointsWithError));
            }
                        
            //distort projected pattern points using created distortion 
            //(we modify the samples)
            //To avoid optimization exceptions while computing distortions,
            //we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (int i = 0; i < numSamples; i++) {
                sample = samples.get(i);
                                
                sample.setSampledMarkers(distortion.distort(
                        sample.getSampledMarkers()));                
            }
            
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);
            
            
            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            calibrator.calibrate();
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();
            
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            RadialDistortion distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());
            
            avgK1Error += k1Error;
            avgK2Error += k2Error;
            
            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }
            
            //check camera poses were not estimated (because distortion was not
            //estimated)
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            assertTrue(estimatedCams > 0);
        }
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        
        avgK1Error /= (double)TIMES;
        avgK2Error /= (double)TIMES;
        
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        //check that average distortion parameters error is small enough

        String msg = "QR pattern - With distortion and outliers" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +                
                avgVerticalPrincipalPointError +
                " avg distortion K1 error: " + avgK1Error +
                " avg distortion K2 error: " + avgK2Error +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " min distortion K1 error: " + minK1Error +
                " min distortion K2 error: " + minK2Error +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError +
                " max distortion K1 error: " + maxK1Error +
                " max distortion K2 error: " + maxK2Error +
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);        
    }
    
    @Test
    @SuppressWarnings("all")
    public void testCalibrateMixedPatternDistortionAndOutliers() 
            throws LockedException, NotReadyException,
            NotSupportedException, DistortionException {
        
        long startTime = System.currentTimeMillis();        
        
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double avgK1Error = 0.0;
        double avgK2Error = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double minK1Error = Double.MAX_VALUE;
        double minK2Error = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double maxK1Error = -Double.MAX_VALUE;
        double maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError, k1Error, k2Error;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            //create distortion
            double k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, 
                    MAX_DISTORTION_PARAM_VALUE);
            //square k2 so that we ensure it is smaller than k1 (typically k1
            //is the dominant term)
            double signk2 = Math.signum(k2);
            k2 = signk2*k2*k2;
            
            
            RadialDistortion distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);
            
            //create samples (each sample has one pinhole camera associated and
            //its corresponding sampled markers)
            int numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, 
                    MAX_NUM_SAMPLES);
            List<CameraCalibratorSample> samples = 
                    new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            for (int i = 0; i < numSamples; i++) {
                //create random camera
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //chose random pattern
                int patternPos = randomizer.nextInt(0, 
                        Pattern2DType.values().length);
                Pattern2D samplePattern = Pattern2D.create(
                        Pattern2DType.values()[patternPos]);

                //build 3D points corresponding to sample pattern (assuming they
                //are in plane Z = 0)
                List<Point2D> samplePatternPoints = 
                        samplePattern.getIdealPoints();
                List<Point3D> points3D = new ArrayList<>();
                for (Point2D patternPoint : samplePatternPoints) {
                    points3D.add(new HomogeneousPoint3D(
                            patternPoint.getInhomX(),
                            patternPoint.getInhomY(), 0.0, 1.0));
                }
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add outliers
                List<Point2D> projectedPatternPointsWithError = 
                        new ArrayList<>();
                for (Point2D p : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        //outlier
                        double errorX = errorRandomizer.nextDouble();
                        double errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(
                                new HomogeneousPoint2D(p.getInhomX() + errorX,
                                p.getInhomY() + errorY, 1.0));
                    } else {
                        //inlier
                        projectedPatternPointsWithError.add(p);
                    }
                }
                                                
                //add random noise to projected points with a certain outlier
                //proportion and a random pattern                
                samples.add(new CameraCalibratorSample(samplePattern,
                        projectedPatternPointsWithError));
            }
                        
            //distort projected pattern points using created distortion 
            //(we modify the samples)
            //To avoid optimization exceptions while computing distortions,
            //we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (int i = 0; i < numSamples; i++) {
                sample = samples.get(i);
                                
                sample.setSampledMarkers(distortion.distort(
                        sample.getSampledMarkers()));
            }            
            
            Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
            
            //create calibrator
            AlternatingCameraCalibrator calibrator = 
                    new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            //check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());
            
            assertEquals(calibrateStart, 0);
            assertEquals(calibrateEnd, 0);
            assertEquals(calibrateProgressChange, 0);
            assertEquals(intrinsicParametersEstimationStarts, 0);
            assertEquals(intrinsicParametersEstimationEnds, 0);
            assertEquals(radialDistortionEstimationStarts, 0);
            assertEquals(radialDistortionEstimationEnds, 0);                    
            
            //calibrate
            try {
                calibrator.calibrate();
            } catch (CalibrationException e) {
                reset();
                continue;
            }
            
            assertEquals(calibrateStart, 1);
            assertEquals(calibrateEnd, 1);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();
            
            //check correctness
            assertFalse(calibrator.isLocked());
            
            //check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    calibrator.getEstimatedIntrinsicParameters();
            
            assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
            assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                    0.0);
            assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

            horizontalFocalDistanceError = Math.abs(
                    intrinsic.getHorizontalFocalLength() -
                    intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(
                    intrinsic.getVerticalFocalLength() -
                    intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() -
                    intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(
                    intrinsic.getHorizontalPrincipalPoint() -
                    intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(
                    intrinsic.getVerticalPrincipalPoint() -
                    intrinsic2.getVerticalPrincipalPoint());
            
            avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
            avgVerticalFocalDistanceError += verticalFocalDistanceError;
            avgSkewnessError += skewnessError;
            avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
            avgVerticalPrincipalPointError += verticalPrincipalPointError;

            if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                minHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                minVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError < minSkewnessError) {
                minSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                minHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                minVerticalPrincipalPointError = verticalPrincipalPointError;
            }

            if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
            }
            if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                maxVerticalFocalDistanceError = verticalFocalDistanceError;
            }
            if (skewnessError > maxSkewnessError) {
                maxSkewnessError = skewnessError;
            }
            if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
            }
            if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                maxVerticalPrincipalPointError = verticalPrincipalPointError;
            }  
            
            //check radial distortion
            RadialDistortion distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());
            
            avgK1Error += k1Error;
            avgK2Error += k2Error;
            
            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }
            
            //check camera poses were not estimated (because distortion was not
            //estimated)
            int estimatedCams = 0;
            for (int i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }
            
            numValid++;
            
            assertTrue(estimatedCams > 0);            
        } 
        
        assertTrue(numValid > 0);
        
        long endTime = System.currentTimeMillis();
        
        avgHorizontalFocalDistanceError /= (double)TIMES;
        avgVerticalFocalDistanceError /= (double)TIMES;
        avgSkewnessError /= (double)TIMES;
        avgHorizontalPrincipalPointError /= (double)TIMES;
        avgVerticalPrincipalPointError /= (double)TIMES;
        
        avgK1Error /= (double)TIMES;
        avgK2Error /= (double)TIMES;
        
        double avgTimeSeconds = (double)(endTime - startTime) / (double)TIMES / 
                (double)1000;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        //check that average distortion parameters error is small enough
        String msg = "Mixed pattern - With distortion and outliers" +
                " avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +                
                avgVerticalPrincipalPointError +
                " avg distortion K1 error: " + avgK1Error +
                " avg distortion K2 error: " + avgK2Error +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " min distortion K1 error: " + minK1Error +
                " min distortion K2 error: " + minK2Error +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError +
                " max distortion K1 error: " + maxK1Error +
                " max distortion K2 error: " + maxK2Error +
                " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);        
    }
    
    @Test
    public void testCalibrateRealData() throws LockedException, 
            CalibrationException, NotReadyException {
        //For a QR pattern, assuming zero skewness, equal focal lengths and
        //principal point at origin on a nexus 5 device
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 774.5, 1084.5 | 6.5, -60.5
        Point[1] = 791.5, 840.0 | 23.5, 184.0
        Point[2] = 1037.5, 854.0 | 269.5, 170.0
        Point[3] = 999.0, 1074.0 | 231.0, -50.0        
        */        
        List<Point2D> sampledPoints1 = new ArrayList<>();
        sampledPoints1.add(new InhomogeneousPoint2D(6.5, -60.5));
        sampledPoints1.add(new InhomogeneousPoint2D(23.5, 184.0));
        sampledPoints1.add(new InhomogeneousPoint2D(269.5, 170.0));
        sampledPoints1.add(new InhomogeneousPoint2D(231.0, -50.0));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        List<Point2D> sampledPoints2 = new ArrayList<>();
        sampledPoints2.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-176.0, 370.5));

        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 988.0, 486.5 | 220.0, 537.5
        Point[1] = 1028.5, 278.5 | 260.5, 745.5
        Point[2] = 1241.0, 316.5 | 473.0, 707.5
        Point[3] = 1185.5, 498.5 | 417.5, 525.5
        */
        List<Point2D> sampledPoints3 = new ArrayList<>();
        sampledPoints3.add(new InhomogeneousPoint2D(220.0, 537.5));
        sampledPoints3.add(new InhomogeneousPoint2D(260.5, 745.5));
        sampledPoints3.add(new InhomogeneousPoint2D(473.0, 707.5));
        sampledPoints3.add(new InhomogeneousPoint2D(417.5, 525.5));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 576.0, 1404.4166 | -192.0, -380.4166259765625
        Point[1] = 544.5, 1151.5 | -223.5, -127.5
        Point[2] = 792.0, 1117.5 | 24.0, -93.5
        Point[3] = 798.5, 1347.0 | 30.5, -323.0
        */
        List<Point2D> sampledPoints4 = new ArrayList<>();
        sampledPoints4.add(new InhomogeneousPoint2D(-192.0, -380.4166259765625));
        sampledPoints4.add(new InhomogeneousPoint2D(-223.5, -127.5));
        sampledPoints4.add(new InhomogeneousPoint2D(24.0, -93.5));
        sampledPoints4.add(new InhomogeneousPoint2D(30.5, -323.0));

        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 913.5, 1596.0 | 145.5, -572.0
        Point[1] = 939.5, 1360.7 | 171.5, -336.699951171875
        Point[2] = 1170.5, 1391.0 | 402.5, -367.0
        Point[3] = 1126.5, 1600.5 | 358.5, -576.5 
        */   
        List<Point2D> sampledPoints5 = new ArrayList<>();
        sampledPoints5.add(new InhomogeneousPoint2D(145.5, -572.0));
        sampledPoints5.add(new InhomogeneousPoint2D(171.5, -336.699951171875));
        sampledPoints5.add(new InhomogeneousPoint2D(402.5, -367.0));
        sampledPoints5.add(new InhomogeneousPoint2D(358.5, -576.5));

        //create samples for each set of sampled QR points
        List<CameraCalibratorSample> samples = 
                new ArrayList<>();
        CameraCalibratorSample sample1 = new CameraCalibratorSample(pattern, 
                sampledPoints1);
        CameraCalibratorSample sample2 = new CameraCalibratorSample(pattern, 
                sampledPoints2);
        CameraCalibratorSample sample3 = new CameraCalibratorSample(pattern, 
                sampledPoints3);
        CameraCalibratorSample sample4 = new CameraCalibratorSample(pattern, 
                sampledPoints4);
        CameraCalibratorSample sample5 = new CameraCalibratorSample(pattern, 
                sampledPoints5);
        
        samples.add(sample1);
        samples.add(sample2);
        samples.add(sample3);
        samples.add(sample4);
        samples.add(sample5);
        
        //create calibrator
        AlternatingCameraCalibrator calibrator = 
                new AlternatingCameraCalibrator(pattern, samples);
        calibrator.setListener(this);
        calibrator.setEstimateRadialDistortion(true);
        calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
        calibrator.setImageOfAbsoluteConicMethod(
                RobustEstimatorMethod.PROSAC);
        calibrator.setIACEstimatorThreshold(REAL_IAC_THRESHOLD);
        calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);

        //check correctness
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isLocked());

        assertEquals(calibrateStart, 0);
        assertEquals(calibrateEnd, 0);
        assertEquals(calibrateProgressChange, 0);
        assertEquals(intrinsicParametersEstimationStarts, 0);
        assertEquals(intrinsicParametersEstimationEnds, 0);
        assertEquals(radialDistortionEstimationStarts, 0);
        assertEquals(radialDistortionEstimationEnds, 0);                    

        //calibrate
        calibrator.calibrate();

        assertEquals(calibrateStart, 1);
        assertEquals(calibrateEnd, 1);
        assertTrue(calibrateProgressChange >= 0);
        assertTrue(intrinsicParametersEstimationStarts > 0);
        assertTrue(intrinsicParametersEstimationEnds > 0);
        assertTrue(radialDistortionEstimationStarts > 0);
        assertTrue(radialDistortionEstimationEnds > 0);
        reset();

        //check correctness
        assertFalse(calibrator.isLocked());

        //check intrinsic parameters
        assertNotNull(calibrator.getHomographyQualityScores());
        assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNotNull(calibrator.getEstimatedIntrinsicParameters());
        PinholeCameraIntrinsicParameters intrinsic = 
                calibrator.getEstimatedIntrinsicParameters();
            
        double horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        double verticalFocalLength = intrinsic.getVerticalFocalLength();
        double skewness = intrinsic.getSkewness();
        double horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        double verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength, 
                ABSOLUTE_ERROR);
        assertEquals(skewness, 0.0, 0.0);
        assertEquals(horizontalPrincipalPoint, 0.0, 0.0);
        assertEquals(verticalPrincipalPoint, 0.0, 0.0); 
        
        //check distortion
        RadialDistortion distortion = calibrator.getDistortion();
        
        double k1 = distortion.getK1();
        double k2 = distortion.getK2();
                   
        String msg = "Real data - focal length: " + horizontalFocalLength +
                "Distortion K1: " + k1 + " K2: " + k2;
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).
                log(Level.INFO, msg);                    
    }

    @Override
    public void onCalibrateStart(CameraCalibrator calibrator) {
        calibrateStart++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    @Override
    public void onCalibrateEnd(CameraCalibrator calibrator) {
        calibrateEnd++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    @Override
    public void onCalibrateProgressChange(CameraCalibrator calibrator, 
            float progress) {
        calibrateProgressChange++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    @Override
    public void onIntrinsicParametersEstimationStarts(
            CameraCalibrator calibrator) {
        intrinsicParametersEstimationStarts++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    @Override
    public void onIntrinsicParametersEstimationEnds(CameraCalibrator calibrator,
            PinholeCameraIntrinsicParameters intrinsicParameters) {
        intrinsicParametersEstimationEnds++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    @Override
    public void onRadialDistortionEstimationStarts(
            CameraCalibrator calibrator) {
        radialDistortionEstimationStarts++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    @Override
    public void onRadialDistortionEstimationEnds(CameraCalibrator calibrator, 
            RadialDistortion distortion) {
        radialDistortionEstimationEnds++;
        checkLocked((AlternatingCameraCalibrator)calibrator);
    }

    private void reset() {
        calibrateStart = calibrateEnd = calibrateProgressChange =
                intrinsicParametersEstimationStarts =
                        intrinsicParametersEstimationEnds =
                                radialDistortionEstimationStarts =
                                        radialDistortionEstimationEnds = 0;
    }

    private void checkLocked(AlternatingCameraCalibrator calibrator){
        try {
            calibrator.setMaxIterations(1);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setConvergenceThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setDistortionMethod(RobustEstimatorMethod.LMedS);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setDistortionEstimatorThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setDistortionEstimatorConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setDistortionEstimatorMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setPattern(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setSamples(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setSamplesQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setEstimateRadialDistortion(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMedS);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setImageOfAbsoluteConicMethod(
                    RobustEstimatorMethod.PROSAC);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setFocalDistanceAspectRatioKnown(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setFocalDistanceAspectRatio(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setHomographyEstimatorThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setHomographyEstimatorConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setHomographyEstimatorMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setIACEstimatorThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setIACEstimatorConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setIACEstimatorMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(calibrator.isLocked());
    }
}
