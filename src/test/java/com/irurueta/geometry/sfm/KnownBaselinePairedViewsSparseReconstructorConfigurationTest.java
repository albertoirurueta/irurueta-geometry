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

package com.irurueta.geometry.sfm;

import com.irurueta.geometry.epipolar.CorrectorType;
import com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import static org.junit.Assert.*;

public class KnownBaselinePairedViewsSparseReconstructorConfigurationTest {

    public KnownBaselinePairedViewsSparseReconstructorConfigurationTest() { }

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
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getBaseline(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.DEFAULT_BASELINE, 0.0);
    }

    @Test
    public void testMake() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                KnownBaselinePairedViewsSparseReconstructorConfiguration.make();

        //check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getBaseline(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.DEFAULT_BASELINE, 0.0);
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM), cfg);

        //check correctness
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
    }

    @Test
    public void testGetSetRobustFundamentalMatrixEstimatorMethod() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.LMedS), cfg);

        //check correctness
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        //set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        //check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        //set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        //check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetPairedCamerasEstimatorMethod() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg);

        //check correctness
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        //set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        //check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetPairedCamerasAspectRatio() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);

        //set new value
        assertSame(cfg.setPairedCamerasAspectRatio(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPairedCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPairedCamerasCorrectorType() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);

        //set new value
        assertSame(cfg.setPairedCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg);

        //check correctness
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }

    @Test
    public void testGetSetPairedCamerasMarkValidTriangulatedPoints() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        //set new value
        assertSame(cfg.setPairedCamerasMarkValidTriangulatedPoints(false),
                cfg);

        //check correctness
        assertFalse(cfg.getPairedCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testAreSetIntrinsicParametersKnown() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.areIntrinsicParametersKnown(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);

        //set new value
        assertSame(cfg.setIntrinsicParametersKnown(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS), cfg);

        //check correctness
        assertEquals(cfg.areIntrinsicParametersKnown(),
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);

        //set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);

        //set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustPlanarHomographyEstimatorMethod(
                RobustEstimatorMethod.RANSAC), cfg);

        //check correctness
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        //set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        //set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE), cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        //check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetBaseline() {
        KnownBaselinePairedViewsSparseReconstructorConfiguration cfg =
                new KnownBaselinePairedViewsSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getBaseline(),
                KnownBaselinePairedViewsSparseReconstructorConfiguration.DEFAULT_BASELINE, 0.0);

        //set new value
        assertSame(cfg.setBaseline(2.0), cfg);

        //check correctness
        assertEquals(cfg.getBaseline(), 2.0, 0.0);
    }
}
