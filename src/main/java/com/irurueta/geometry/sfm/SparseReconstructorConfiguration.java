/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.SparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 12, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.epipolar.CorrectorType;
import com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.geometry.epipolar.estimators.FundamentalMatrixRobustEstimator;
import com.irurueta.geometry.epipolar.estimators.PROSACFundamentalMatrixRobustEstimator;
import com.irurueta.geometry.estimators.EPnPPointCorrespondencePinholeCameraEstimator;
import com.irurueta.geometry.estimators.PinholeCameraRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

/**
 *
 * @author albertoirurueta
 */
public class SparseReconstructorConfiguration {
    
    /**
     * Default robust fundamental matrix estimator method.
     */
    public static final RobustEstimatorMethod 
            DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            RobustEstimatorMethod.PROSAC;
    
    /**
     * Default non robust fundamental matrix estimator method used internally
     * within a robust estimator.
     */
    public static final FundamentalMatrixEstimatorMethod 
            DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;
    
    /**
     * Indicates that estimated fundamental matrices are refined by default
     * using all found inliers.
     */
    public static final boolean DEFAULT_REFINE_FUNDAMENTAL_MATRIX = true;
    
    /**
     * Indicates that fundamental matrix covariance is kept by default after the
     * estimation.
     */
    public static final boolean DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE =
            false;
    
    /**
     * Default confidence of robustly estimated fundamental matrix. By default
     * this is 99%.
     */
    public static final double DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE =
            FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE;
    
    /**
     * Default maximum number of iterations to make while robustly estimating
     * fundamental matrix. By default this is 5000 iterations.
     */
    public static final int DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS =
            FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS;
    
    /**
     * Default threshold to determine whether samples for robust fundamental 
     * matrix estimation are inliers or not.
     */
    public static final double DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD =
            PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD;
    
    /**
     * Default value indicating that inlier data is kept after robust 
     * fundamental matrix estimation.
     */
    public static final boolean 
            DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS = true;
    
    /**
     * Default value indicating that residual data is kept after robust
     * fundamental matrix estimation.
     */
    public static final boolean
            DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS = true;
    
    /**
     * Default method to use for initial cameras estimation.
     */
    public static final InitialCamerasEstimatorMethod 
            DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD = 
            InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC;
    
    /**
     * Default aspect ratio for initial cameras.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO = 1.0;
    
    /**
     * Default horizontal principal point value to use for initial cameras
     * estimation using Dual Image of Absolute Conic (DIAC) method.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_DIAC_PRINCIPAL_POINT_X = 
            0.0;
    
    /**
     * Default vertical principal point value to use for initial cameras
     * estimation using Dual Image of Absolute Conic (DIAC) method.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_DIAC_PRINCIPAL_POINT_Y =
            0.0;
    
    /**
     * Default corrector type to use for point triangulation when initial 
     * cameras are being estimated using either Dual Image of Absolute Conic 
     * (DIAC) or essential matrix methods.
     */
    public static CorrectorType DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE = 
            CorrectorType.SAMPSON_CORRECTOR;
    
    /**
     * Default value indicating whether points must be triangulated during
     * initial cameras estimation using either DIAC or essential matrix methods.
     */
    public static final boolean DEFAULT_INITIAL_CAMERAS_TRIANGULATE_POINTS =
            true;
    
    /**
     * Default value indicating whether valid triangulated points are marked
     * during initial cameras estimation using either Dual Image of Absolute 
     * Conic (DIAC) or essential matrix methods.
     */
    public static final boolean 
            DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS = true;
    
    /**
     * Default value indicating that initial cameras are estimated using the
     * Dual Absolute Quadric (DAQ).
     */
    public static final boolean 
            DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS = true;
    
    /**
     * Default value indicating that initial cameras are estimated using the 
     * Dual Image of Absolute Conic (DIAC).
     */
    public static final boolean 
            DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS = false;
    
    /**
     * Default skewness for additional cameras when UPnP (Uncalibrated 
     * Perspective-n-Point) method is used for additional cameras estimation and
     * neither Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric 
     * (DAQ) are estimated to find intrinsic parameters when adding new cameras.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS = 0.0;
    
    /**
     * Default horizontal coordinate of principal point for additional cameras
     * when UPnP (Uncalibrated Perspective-n-Point) method is used for 
     * additional cameras estimation and neither Dual Image of Absolute Conic 
     * (DIAC) or Dual Absolute Quadric (DAQ) are estimated to find intrinsic 
     * parameters when adding new cameras.
     */
    public static final double 
            DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT = 0.0;
    
    /**
     * Default vertical coordinate of principal point for additional cameras
     * when UPnP (Uncalibrated Perspective-n-Point) method is used for 
     * additional cameras estimation and neither Dual Image of Absolute Conic
     * (DIAC) or Dual Absolute Quadric (DAQ) are estimated to find intrinsic
     * parameters when adding new cameras.
     */
    public static final double 
            DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT = 0.0;
    
    /**
     * Indicates that by default EPnP (Efficient Perspective-n-Point) method is
     * used for additional cameras estimation.
     */
    public static final boolean 
            DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION = true;
    
    /**
     * Indicates that by default UPnP (Uncalibrated Perspective-n-Point) method 
     * is NOT used for additional cameras estimation.
     */
    public static final boolean 
            DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION = false;
    
    /**
     * Default robust method to estimate additional cameras.
     */
    public static final RobustEstimatorMethod 
            DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD =
            RobustEstimatorMethod.PROSAC;
    
    /**
     * Default value indicating that planar configuration is allowed for 
     * additional cameras estimation using either EPnP or UPnP.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_PLANAR_CONFIGURATION_ALLOWED;
    
    /**
     * Default value indicating that dimension 2 nullspace is allowed while
     * estimating additional cameras using either EPnP or UPnP.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2 = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_NULLSPACE_DIMENSION2_ALLOWED;
    
    /**
     * Default value indicating that dimension 3 nullspace is allowed while
     * estimating additional cameras using EPnP.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3 = 
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_NULLSPACE_DIMENSION3_ALLOWED;
    
    /**
     * Default threshold to determine whether 3D matched points to estimate 
     * additional cameras are in a planar configuration.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD =
            EPnPPointCorrespondencePinholeCameraEstimator.
            DEFAULT_PLANAR_THRESHOLD;
    
    /**
     * Default value indicating that additional cameras are refined to minimize
     * overall projection error among all found inliers.
     */
    public static final boolean DEFAULT_REFINE_ADDITIONAL_CAMERAS =
            PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT;
    
    /**
     * Default value indicating that covariance is kept after refining result
     * of additional cameras estimation.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS =
            true;
    
    /**
     * Default value indicating that fast refinement is used for additional
     * cameras estimation.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT = 
            true;
    
    /**
     * Default confidence of estimated additional cameras, which is 99%. This 
     * means that with a probability of 99% estimation will be accurate because
     * chosen subsamples will be inliers.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE =
            PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE;
    
    /**
     * Default maximum allowed number of iterations for additional cameras 
     * estimation.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS =
            PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS;
    
    /**
     * Default value indicating that skewness is not suggested during additional
     * cameras estimation.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED;
    
    /**
     * Default value of skewness to be suggested when suggestion is enabled 
     * during additional cameras estimation.
     * By default suggested skewness is zero.
     */
    public static final double 
            DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE =
            PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE;
    
    /**
     * Default value indicating whether horizontal focal length value is 
     * suggested or not during additional cameras estimation. By default this is
     * disabled.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED =
            PinholeCameraRobustEstimator.
            DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;
    
    /**
     * Default value indicating whether vertical focal length value is suggested
     * or not during additional cameras estimation. By default this is disabled.
     */
    public static final boolean
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED =
            PinholeCameraRobustEstimator.
            DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;
    
    /**
     * Default value indicating whether aspect ratio is suggested or not. By 
     * default this is disabled.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED;
    
    /**
     * Default value of aspect ratio to be suggested when suggestion is enabled.
     * By default suggested aspect ratio is 1.0, although also -1.0 is a typical
     * value when vertical coordinates increase downwards.
     */
    public static final double 
            DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE =
            PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE;
    
    /**
     * Default value indicating whether principal point is suggested or not. By
     * default this is disabled.
     */
    public static final boolean 
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED =
            PinholeCameraRobustEstimator.
            DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED;
    
    /**
     * Indicates that an homogeneous point triangulator will be used by default.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR =
            RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION;

    /**
     * Default robust point triangulator method.
     */
    public static final RobustEstimatorMethod 
            DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD = 
            RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD;
    
    /**
     * Default confidence of robustly triangulated points. By default this is
     * 99%.
     */
    public static final double DEFAULT_POINT_TRIANGULATOR_CONFIDENCE =
            RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum number of iterations to make while robustly estimating
     * triangulated points. By default this is 5000 iterations.
     */
    public static final int DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS =
            RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust point 
     * triangulator are inliers or not.
     */
    public static final double DEFAULT_POINT_TRIANGULATOR_THRESHOLD =
            PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD;
    
    
    /**
     * Method to use for non robust fundamental matrix estimation.
     */
    private FundamentalMatrixEstimatorMethod 
            mNonRobustFundamentalMatrixEstimatorMethod = 
            DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD;
    
    /**
     * Method to use for robust fundamental matrix estimation.
     */
    private RobustEstimatorMethod mRobustFundamentalMatrixEstimatorMethod =
            DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD;
    
    /**
     * Indicates whether estimated fundamental matrix is refined among all found
     * inliers.
     */
    private boolean mRefineFundamentalMatrix =
            DEFAULT_REFINE_FUNDAMENTAL_MATRIX;
    
    /**
     * Indicates whether covariance of estimated fundamental matrix is kept 
     * after the estimation.
     */
    private boolean mKeepFundamentalMatrixCovariance =
            DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE;
    
    /**
     * Confidence of robustly estimated fundamental matrix.
     */
    private double mFundamentalMatrixConfidence =
            DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE;
    
    /**
     * Maximum number of iterations to robustly estimate fundamental matrix.
     */
    private int mFundamentalMatrixMaxIterations =
            DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS;
    
    /**
     * Threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     */
    private double mFundamentalMatrixThreshold =
            DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD;
    
    /**
     * Indicates whether inliers must be kept during robust fundamental matrix
     * estimation.
     */
    private boolean mFundamentalMatrixComputeAndKeepInliers =
            DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS;
    
    /**
     * Indicates whether residuals must be computed and kept during robust
     * fundamental matrix estimation.
     */
    private boolean mFundamentalMatrixComputeAndKeepResiduals =
            DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS;
    
    /**
     * Method to use for initial cameras estimation.
     */
    private InitialCamerasEstimatorMethod mInitialCamerasEstimatorMethod =
            DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD;

    /**
     * Aspect ratio for initial cameras.
     */
    private double mInitialCamerasAspectRatio =
            DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO;
    
    /**
     * Horizontal principal point value to use for initial cameras estimation 
     * using Dual Image of Absolute Conic (DIAC) method.
     */
    private double mDiacPrincipalPointX = 
            DEFAULT_INITIAL_CAMERAS_DIAC_PRINCIPAL_POINT_X;
    
    /**
     * Vertical principal point value to use for initial cameras estimation 
     * using Dual Image of Absolute Conic (DIAC) method.
     */
    private double mDiacPrincipalPointY = 
            DEFAULT_INITIAL_CAMERAS_DIAC_PRINCIPAL_POINT_Y;
    
    /**
     * Corrector type to use for point triangulation when initial cameras are
     * being estimated using either Dual Image of Absolute Conic (DIAC) or 
     * essential matrix methods or null if no corrector is used.
     */
    private CorrectorType mInitialCamerasCorrectorType = 
            DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE;
    
    /**
     * Value indicating whether points must be triangulated during initial 
     * cameras estimation using either Dual Image of Absolute Conic (DIAC) or 
     * essential matrix methods.
     */
    private boolean mInitialCamerasTriangulatePoints = 
            DEFAULT_INITIAL_CAMERAS_TRIANGULATE_POINTS;
    
    /**
     * Value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either Dual Image of Absolute Conic 
     * (DIAC) or essential matrix methods.
     */
    private boolean mInitialCamerasMarkValidTriangulatedPoints =
            DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS;    
    
    /**
     * Intrinsic parameters of first camera estimated using the essential matrix
     * method.
     */
    private PinholeCameraIntrinsicParameters mInitialIntrinsic1;
    
    /**
     * Intrinsic parameters of second camera estimated using the essential 
     * matrix method.
     */
    private PinholeCameraIntrinsicParameters mInitialIntrinsic2;
    
    /**
     * Indicates that initial cameras are estimated using the Dual Absolute 
     * Quadric (DAQ).
     */
    private boolean mUseDAQForAdditionalCamerasIntrinsics = 
            DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS;
    
    /**
     * Indicates that additional cameras are estimated using the Dual Image of 
     * Absolute Conic (DIAC).
     */
    private boolean mUseDIACForAdditionalCamerasIntrinsics = 
            DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS;
    
    /**
     * Intrinsic parameters to use for additional cameras estimation when 
     * neither Dual Image of Absolute Conic (DIAC) nor Dual Absolute Quadric 
     * (DAQ) are used for intrinsic parameters estimation.
     */
    private PinholeCameraIntrinsicParameters mAdditionalCamerasIntrinsics;
    
    /**
     * Skewness for additional cameras when UPnP (Uncalibrated 
     * Perspective-n-Point) method is used for additional cameras estimation.
     */
    private double mAdditionalCamerasSkewness = 
            DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS;
    
    /**
     * Horizontal coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras
     * estimation and neither Dual Image of Absolute Conic (DIAC) or Dual 
     * Absolute Quadric (DAQ) are estimated to find intrinsic parameters when
     * adding new cameras.
     */
    private double mAdditionalCamerasHorizontalPrincipalPoint = 
            DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT;
    
    /**
     * Vertical coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras
     * estimation and neither Dual Image of Absolute Conic (DIAC) or Dual 
     * Absolute Quadric (DAQ) are estimated to find intrinsic parameters when
     * adding new cameras.
     */
    private double mAdditionalCamerasVerticalPrincipalPoint = 
            DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT;
        
    /**
     * Indicates whether EPnP (Efficient Perspective-n-Point) method is used
     * for additional cameras estimation. Either EPnP or UPnP must be used
     * for additional cameras estimation.
     */
    private boolean mUseEPnPForAdditionalCamerasEstimation = 
            DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION;
    
    /**
     * Indicates whether UPnP (Uncalibrated Perspective-n-Point) method is used
     * for additional cameras estimation. Either EPnP or UPnP must be used
     * for additional cameras estimation.
     */
    private boolean mUseUPnPForAdditionalCamerasEstimation =
            DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION;
    
    /**
     * Robust method to estimate additional cameras.
     */
    private RobustEstimatorMethod mAdditionalCamerasRobustEstimationMethod =
            DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD;
    
    /**
     * Indicates whether planar configuration is allowed for additional cameras
     * estimation using either EPnP or UPnP.
     */
    private boolean mAdditionalCamerasAllowPlanarConfiguration =
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION;
    
    /**
     * Indicates whether dimension 2 nullspace is allowed while estimating 
     * additional cameras using either EPnP or UPnP.
     */
    private boolean mAdditionalCamerasAllowNullspaceDimension2 =
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2;
    
    /**
     * Indicates whether dimension 3 nullspace is allowed while estimating
     * additional cameras using EPnP.
     */
    private boolean mAdditionalCamerasAllowNullspaceDimension3 =
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3;

    /**
     * Threshold to determine whether 3D matched points to estimate additional
     * cameras are in a planar configuration.
     */
    private double mAdditionalCamerasPlanarThreshold =
            DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD;
    
    /**
     * Indicates whether additional cameras are refined to minimize overall
     * projection error among all found inliers.
     */
    private boolean mRefineAdditionalCameras =
            DEFAULT_REFINE_ADDITIONAL_CAMERAS;
    
    /**
     * Indicates whether covariance is kept after refining result of additional
     * cameras estimation.
     */
    private boolean mKeepCovarianceAdditionalCameras =
            DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS;
    
    /**
     * Value indicating whether fast refinement is used for additional cameras
     * estimation.
     */
    private boolean mAdditionalCamerasUseFastRefinement =
            DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT;
    
    /**
     * Confidence of estimated additional cameras.
     */
    private double mAdditionalCamerasConfidence =
            DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE;
    
    /**
     * Maximum allowed number of iterations for additional cameras estimation.
     */
    private double mAdditionalCamerasMaxIterations =
            DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS;

    /**
     * Value indicating whether skewness is not suggested during additional
     * cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestSkewnessValueEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED;
    
    /**
     * Value of skewness to be suggested when suggestion is enabled during 
     * additional cameras estimation.
     */
    private double mAdditionalCamerasSuggestedSkewnessValue =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE;
    
    /**
     * Value indicating whether horizontal focal length value is suggested or
     * not during additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestHorizontalFocalLengthEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;
    
    /**
     * Value of suggested horizontal focal length during additional cameras
     * estimation.
     */
    private double mAdditionalCamerasSuggestedHorizontalFocalLengthValue;
    
    /**
     * Value indicating whether vertical focal length value is suggested or
     * not during additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestVerticalFocalLengthEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;
    
    /**
     * Value of suggested vertical focal length during additional cameras 
     * estimation.
     */
    private double mAdditionalCamerasSuggestedVerticalFocalLengthValue;

    /**
     * Value indicating whether aspect ratio is suggested or not during 
     * additional cameras estimaton.
     */
    private boolean mAdditionalCamerasSuggestAspectRatioEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED;
    
    /**
     * Value of aspect ratio to be suggested when suggestion is enabled during
     * additional cameras estimation.
     */
    private double mAdditionalCamerasSuggestedAspectRatioValue =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE;
    
    /**
     * Value indicating whether principal point is suggested or not during 
     * additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestPrincipalPointEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED;
    
    /**
     * Value of principal point to be suggested when suggestion is enabled 
     * during additional cameras estimation.
     */
    private InhomogeneousPoint2D mAdditionalCamerasSuggestedPrincipalPointValue;
    
    /**
     * Indicates whether homogeneous point triangulator must be used or not to
     * estimate 3D points when only two matches are available.
     */
    private boolean mUseHomogeneousPointTriangulator = 
            DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR;

    /**
     * Robust method for point triangulation when points are matched in more
     * than two views.
     */
    private RobustEstimatorMethod mRobustPointTriangulatorMethod =
            DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD;

    /**
     * Confidence of robustly triangulated points. By default this is 99%.
     */
    private double mPointTriangulatorConfidence = 
            DEFAULT_POINT_TRIANGULATOR_CONFIDENCE;
    
    /**
     * Maximum number of iterations to make while robustly estimating 
     * triangulated points. By default this is 5000 iterations.
     */
    private int mPointTriangulatorMaxIterations = 
            DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust point triangulator are
     * inliers or not.
     */
    private double mPointTriangulatorThreshold =
            DEFAULT_POINT_TRIANGULATOR_THRESHOLD;

    
    /**
     * Constructor.
     */
    public SparseReconstructorConfiguration() { }
    
    /**
     * Creates an instance of a sparse reconstructor configuration.
     * @return configuration instance.
     */
    public static SparseReconstructorConfiguration make() {
        return new SparseReconstructorConfiguration();
    }
    
    /**
     * Gets method to use for non robust fundamental matrix estimation.
     * @return method to use for non robust fundamental matrix estimation.
     */
    public FundamentalMatrixEstimatorMethod 
        getNonRobustFundamentalMatrixEstimatorMethod() {
        return mNonRobustFundamentalMatrixEstimatorMethod;
    }
        
    /**
     * Sets method to use for non robust fundamental matrix estimation.
     * @param method method to use for non robust fundamental matrix estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setNonRobustFundamentalMatrixEstimatorMethod(
                    FundamentalMatrixEstimatorMethod method) {
        mNonRobustFundamentalMatrixEstimatorMethod = method;
        return this;
    }
            
    /**
     * Gets method to use for robust fundamental matrix estimation.
     * @return method to use for robust fundamental matrix estimation.
     */
    public RobustEstimatorMethod getRobustFundamentalMatrixEstimatorMethod() {
        return mRobustFundamentalMatrixEstimatorMethod;
    }
    
    /**
     * Sets method to use for robust fundamental matrix estimation.
     * @param method method to use for robust fundamental matrix estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setRobustFundamentalMatrixEstimatorMethod(
                    RobustEstimatorMethod method) {
        mRobustFundamentalMatrixEstimatorMethod = method;
        return this;
    }
    
    /**
     * Indicates whether estimated fundamental matrix is refined among all found
     * inliers.
     * @return true if fundamental matrix is refined, false otherwise.
     */
    public boolean isFundamentalMatrixRefined() {
        return mRefineFundamentalMatrix;
    }
    
    /**
     * Specifies whether estimated fundamental matrix is refined among all found
     * inliers.
     * @param refineFundamentalMatrix true if fundamental matrix is refined, 
     * false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration
            setFundamentalMatrixRefined(boolean refineFundamentalMatrix) {
        mRefineFundamentalMatrix = refineFundamentalMatrix;
        return this;
    }

    /**
     * Indicates whether covariance of estimated fundamental matrix is kept
     * after the estimation.
     * @return true if covariance is kept, false otherwise.
     */
    public boolean isFundamentalMatrixCovarianceKept() {
        return mKeepFundamentalMatrixCovariance;
    }
    
    /**
     * Specifies whether covariance of estimated fundamental matrix is kept
     * after the estimation.
     * @param keepFundamentalMatrixCovariance true if covariance is kept, false
     * otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setFundamentalMatrixCovarianceKept(
            boolean keepFundamentalMatrixCovariance) {
        mKeepFundamentalMatrixCovariance = keepFundamentalMatrixCovariance;
        return this;
    }

    /**
     * Gets confidence of robustly estimated fundamental matrix.
     * @return confidence of robustly estimated fundamental matrix.
     */
    public double getFundamentalMatrixConfidence() {
        return mFundamentalMatrixConfidence;
    }
    
    /**
     * Sets confidence of robustly estimated fundamental matrix.
     * @param fundamentalMatrixConfidence confidence of robustly estimated 
     * fundamental matrix.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setFundamentalMatrixConfidence(
            double fundamentalMatrixConfidence) {
        mFundamentalMatrixConfidence = fundamentalMatrixConfidence;
        return this;
    }
    
    /**
     * Gets maximum number of iterations to robustly estimate fundamental 
     * matrix.
     * @return maximum number of iterations to robustly estimate fundamental
     * matrix.
     */
    public int getFundamentalMatrixMaxIterations() {
        return mFundamentalMatrixMaxIterations;
    }
    
    /**
     * Sets maximum number of iterations to robustly estimate fundamental 
     * matrix.
     * @param fundamentalMatrixMaxIterations maximum number of iterations to
     * robustly estimate fundamental matrix.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setFundamentalMatrixMaxIterations(
            int fundamentalMatrixMaxIterations) {
        mFundamentalMatrixMaxIterations = fundamentalMatrixMaxIterations;
        return this;
    }
    
    /**
     * Gets threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     * @return threshold to determine whether samples for robust fundamental
     * matrix estimation are inliers or not.
     */
    public double getFundamentalMatrixThreshold() {
        return mFundamentalMatrixThreshold;
    }
    
    /**
     * Sets threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     * @param fundamentalMatrixThreshold threshold to determine whether samples
     * for robust fundamental matrix estimation are inliers or not.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setFundamentalMatrixThreshold(
            double fundamentalMatrixThreshold) {
        mFundamentalMatrixThreshold = fundamentalMatrixThreshold;
        return this;
    }

    /**
     * Indicates whether inliers must be kept during robust fundamental matrix
     * estimation.
     * @return true if inliers must be kept during robust fundamental matrix 
     * estimation, false otherwise.
     */
    public boolean getFundamentalMatrixComputeAndKeepInliers() {
        return mFundamentalMatrixComputeAndKeepInliers;
    }
    
    /**
     * Specifies whether inliers must be kept during robust fundamental matrix
     * estimation.
     * @param fundamentalMatrixComputeAndKeepInliers true if inliers must be
     * kept during robust fundamental matrix estimation, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
        setFundamentalMatrixComputeAndKeepInliers(
                boolean fundamentalMatrixComputeAndKeepInliers) {
        mFundamentalMatrixComputeAndKeepInliers = 
                fundamentalMatrixComputeAndKeepInliers;
        return this;
    }

    /**
     * Indicates whether residuals must be computed and kept during robust
     * fundamental matrix estimation.
     * @return true if residuals must be computed and kept, false otherwise.
     */
    public boolean getFundamentalMatrixComputeAndKeepResiduals() {
        return mFundamentalMatrixComputeAndKeepResiduals;
    }
    
    /**
     * Specifies whether residuals must be computed and kept during robust
     * fundamental matrix estimation.
     * @param fundamentalMatrixComputeAndKeepResiduals true if residuals must be
     * computed and kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
        setFundamentalMatrixComputeAndKeepResiduals(
                boolean fundamentalMatrixComputeAndKeepResiduals) {
        mFundamentalMatrixComputeAndKeepResiduals = 
                fundamentalMatrixComputeAndKeepResiduals;
        return this;
    }          
    
    /**
     * Gets method to use for initial cameras estimation.
     * @return method to use for initial cameras estimation.
     */
    public InitialCamerasEstimatorMethod getInitialCamerasEstimatorMethod() {
        return mInitialCamerasEstimatorMethod;
    }
    
    /**
     * Sets method to use for initial cameras estimation.
     * @param method method to use for initial cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setInitialCamerasEstimatorMethod(
            InitialCamerasEstimatorMethod method) {
        mInitialCamerasEstimatorMethod = method;
        return this;
    }
    
    /**
     * Gets aspect ratio for initial cameras estimation using DAQ or DIAC
     * methods.
     * @return aspect ratio for initial cameras using DAQ or DIAC methods.
     */
    public double getInitialCamerasAspectRatio() {
        return mInitialCamerasAspectRatio;
    }
    
    /**
     * Sets aspect ratio for initial cameras using DAQ or DIAC methods.
     * @param initialCamerasAspectRatio aspect ratio for initial cameras using
     * DAQ or DIAC methods.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setInitialCamerasAspectRatio(
            double initialCamerasAspectRatio) {
        mInitialCamerasAspectRatio = initialCamerasAspectRatio;
        return this;
    }

    /**
     * Gets horizontal principal point value to use for initial cameras 
     * estimation using DIAC method.
     * @return horizontal principal point value to use for initial cameras 
     * estimation using DIAC method.
     */
    public double getDiacPrincipalPointX() {
        return mDiacPrincipalPointX;
    }
    
    /**
     * Sets horizontal principal point value to use for initial cameras 
     * estimation using DIAC method.
     * @param diacPrincipalPointX horizontal principal point value to use for
     * initial cameras estimation using DIAC method.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setDiacPrincipalPointX(
            double diacPrincipalPointX) {
        mDiacPrincipalPointX = diacPrincipalPointX;
        return this;
    }

    /**
     * Gets vertical principal point value to use for initial cameras estimation
     * using DIAC method.
     * @return vertical principal point value to use for initial cameras
     * estimation using DIAC method.
     */
    public double getDiacPrincipalPointY() {
        return mDiacPrincipalPointY;
    }
    
    /**
     * Sets vertical principal point value to use for initial cameras estimation
     * using DIAC method.
     * @param diacPrincipalPointY vertical principal point value to use for
     * initial cameras estimation using DIAC method.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setDiacPrincipalPointY(
            double diacPrincipalPointY) {
        mDiacPrincipalPointY = diacPrincipalPointY;
        return this;
    }
    
    /**
     * Gets corrector type to use for point triangulation when initial cameras 
     * are being estimated using either DIAC or essential matrix methods or null
     * if no corrector is used.
     * @return corrector type to use for point triangulation when initial 
     * cameras are being estimated using either DIAC or essential matrix methods
     * or null if no corrector is used.
     */
    public CorrectorType getInitialCamerasCorrectorType() {
        return mInitialCamerasCorrectorType;
    }
    
    /**
     * Sets corrector type to use for point triangulation when initial cameras
     * are being estimated using either DIAC or essential matrix methods or null
     * if no corrector is used.
     * @param type corrector type to use for point triangulation when initial
     * cameras are being estimated using either DIAC or essential matrix methods
     * or null if no corrector is used.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setInitialCamerasCorrectorType(
            CorrectorType type) {
        mInitialCamerasCorrectorType = type;
        return this;
    }
    
    /**
     * Gets value indicating whether points must be triangulated during initial
     * cameras estimation using either DIAC or essential matrix methods.
     * @return value indicating whether points must be triangulated during 
     * initial cameras estimation using either DIAC or essential matrix methods.
     */
    public boolean getInitialCamerasTriangulatePoints() {
        return mInitialCamerasTriangulatePoints;
    }
    
    /**
     * Sets value indicating whether points must be triangulated during initial
     * cameras estimation using either DIAC or essential matrix methods.
     * @param initialCamerasTriangulatePoints value indicating whether points 
     * must be triangulated during initial cameras estimation using either DIAC
     * or essential matrix methods.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setInitialCamerasTriangulatePoints(
            boolean initialCamerasTriangulatePoints) {
        mInitialCamerasTriangulatePoints = initialCamerasTriangulatePoints;
        return this;
    }

    /**
     * Gets value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either DIAC or essential matrix methods.
     * @return value indicating whether valid triangulated points are marked 
     * during initial cameras estimation using either DIAC or essential matrix
     * methods.
     */
    public boolean getInitialCamerasMarkValidTriangulatedPoints() {
        return mInitialCamerasMarkValidTriangulatedPoints;
    }
    
    /**
     * Sets value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either DIAC or essential matrix methods.
     * @param initialCamerasMarkValidTriangulatedPoints value indicating whether
     * valid triangulated points are marked during initial cameras estimation
     * using either DIAC or essential matrix methods.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setInitialCamerasMarkValidTriangulatedPoints(
                    boolean initialCamerasMarkValidTriangulatedPoints) {
        mInitialCamerasMarkValidTriangulatedPoints = 
                initialCamerasMarkValidTriangulatedPoints;
        return this;
    }

    /**
     * Gets intrinsic parameters of first camera estimated using the essential
     * matrix method.
     * @return intrinsic parameters of first camera estimated using the 
     * essential matrix method.
     */
    public PinholeCameraIntrinsicParameters getInitialIntrinsic1() {
        return mInitialIntrinsic1;
    }
    
    /**
     * Sets intrinsic parameters of first camera estimated using the essential
     * matrix method.
     * @param initialIntrinsic1 intrinsic parameters of first camera estimated 
     * using the essential matrix method.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setInitialIntrinsic1(
            PinholeCameraIntrinsicParameters initialIntrinsic1) {
        mInitialIntrinsic1 = initialIntrinsic1;
        return this;
    }

    /**
     * Gets intrinsic parameters of second camera estimated using the essential
     * matrix method.
     * @return intrinsic parameters of second camera estimated using the 
     * essential matrix method.
     */
    public PinholeCameraIntrinsicParameters getInitialIntrinsic2() {
        return mInitialIntrinsic2;
    }
    
    /**
     * Sets intrinsic parameters of second camera estimated using the essential
     * matrix method.
     * @param initialIntrinsic2 intrinsic parameters of second camera estimated
     * using the essential matrix method.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setInitialIntrinsic2(
            PinholeCameraIntrinsicParameters initialIntrinsic2) {
        mInitialIntrinsic2 = initialIntrinsic2;
        return this;
    }
    
    /**
     * Indicates that initial cameras are estimated using the Dual Absolute
     * Quadric (DAQ).
     * @return true if initial cameras are estimated using the Dual Absolute 
     * Quadric (DAQ), false otherwise.
     */
    public boolean getUseDAQForAdditionalCamerasIntrinsics() {
        return mUseDAQForAdditionalCamerasIntrinsics;
    }
    
    /**
     * Specifies whether initial cameras are estimated using the Dual Absolute
     * Quadric (DAQ).
     * @param useDAQForAdditionalCamerasIntrinsics true if initial cameras are 
     * estimated using the Dual Absolute Quadric (DAQ), false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setUseDAQForAdditionalCamerasIntrinsics(
            boolean useDAQForAdditionalCamerasIntrinsics) {
        mUseDAQForAdditionalCamerasIntrinsics = 
                useDAQForAdditionalCamerasIntrinsics;
        return this;
    }

    /**
     * Indicates that initial cameras are estimated using the Dual Image of
     * Absolute Conic (DIAC).
     * @return true if initial cameras are estimated using the Dual Image of 
     * Absolute Conic (DIAC), false otherwise.
     */
    public boolean getUseDIACForAdditionalCamerasIntrinsics() {
        return mUseDIACForAdditionalCamerasIntrinsics;
    }

    /**
     * Specifies whether initial cameras are estimated using the Dual Image of
     * Absolute Conic (DIAC).
     * @param useDIACForAdditionalCamerasIntrinsics true if initial cameras are 
     * estimated using the Dual Image of Absolute Conic (DIAC), false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setUseDIACForAdditionalCamerasIntrinsics(
            boolean useDIACForAdditionalCamerasIntrinsics) {
        mUseDIACForAdditionalCamerasIntrinsics = 
                useDIACForAdditionalCamerasIntrinsics;
        return this;
    }

    /**
     * Gets intrinsic parameters to use for additional cameras estimation when
     * neither Dual Image of Absolute Conic (DIAC) nor Dual Absolute Quadric 
     * (DAQ) are used for intrinsic parameters estimation.
     * @return intrinsic parameters to use for additional cameras estimation
     * when neither DIAC nor DAQ are used for intrinsic parameters estimation.
     */
    public PinholeCameraIntrinsicParameters getAdditionalCamerasIntrinsics() {
        return mAdditionalCamerasIntrinsics;
    }
    
    /**
     * Specifies intrinsic parameters to use for additional cameras estimation 
     * when neither Dual Image of Absolute Conic (DIAC) nor Dual Absolute 
     * Quadric (DAQ) are used for intrinsic parameters estimation.
     * @param additionalCamerasIntrinsics intrinsic parameters to use for 
     * additional cameras estimation when neither DIAC nor DAQ are used for
     * intrinsic parameters estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasIntrinsics(
            PinholeCameraIntrinsicParameters additionalCamerasIntrinsics) {
        mAdditionalCamerasIntrinsics = additionalCamerasIntrinsics;
        return this;
    }

    /**
     * Gets skewness for additional cameras when UPnP (Uncalibrated 
     * Perspective-n-Point) method is used for additional cameras estimation.
     * @return skewness for additional cameras when UPnP method is used for
     * additional cameras estimation.
     */
    public double getAdditionalCamerasSkewness() {
        return mAdditionalCamerasSkewness;
    }
    
    /**
     * Sets skewness for additional cameras when UPnP (Uncalibrated 
     * Perspective-n-Point) method is used for additional cameras estimation.
     * @param additionalCamerasSkewness skewness for additional cameras when
     * UPnP method is used for additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasSkewness(
            double additionalCamerasSkewness) {
        mAdditionalCamerasSkewness = additionalCamerasSkewness;
        return this;
    }
    
    /**
     * Gets horizontal coordinate of principal point for additional cameras when
     * UPnP (Uncalibrated Perspective-n-Point) method is used for additional 
     * cameras estimation and neither Dual Image of Absolute Conic (DIAC) or 
     * Dual Absolute Quadric (DAQ) are estimated to find intrinsic parameters
     * when adding new cameras.
     * @return horizontal coordinate of principal point for additional cameras
     * when UPnP method is used for additional cameras estimation and neither
     * DIAC or DAQ are estimated to find intrinsic parameters when adding new 
     * cameras.
     */
    public double getAdditionalCamerasHorizontalPrincipalPoint() {
        return mAdditionalCamerasHorizontalPrincipalPoint;
    }
    
    /**
     * Sets horizontal coordinate of principal point for additional cameras when
     * UPnP (Uncalibrated Perspective-n-Point) method is used for additional
     * cameras estimation and neither Dual Image of Absolute Conic (DIAC) or
     * Dual Absolute Quadric (DAQ) are estimated to find intrinsic parameters
     * when adding new cameras.
     * @param additionalCamerasHorizontalPrincipalPoint horizontal coordinate of
     * principal point for additional cameras when UPnP method is used for 
     * additional cameras estimation and neither DIAC or DAQ are estimated to 
     * find intrinsic parameters when adding new cameras.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasHorizontalPrincipalPoint(
                    double additionalCamerasHorizontalPrincipalPoint) {
        mAdditionalCamerasHorizontalPrincipalPoint = 
                additionalCamerasHorizontalPrincipalPoint;
        return this;
    }

    /**
     * Gets vertical coordinate of principal point for additional cameras when 
     * UPnP (Uncalibrated Perspective-n-Point) method is used for additional
     * cameras estimation and neither Dual Image of Absolute Conic (DIAC) or
     * Dual Absolute Quadric (DAQ) are estimated to find intrinsic parameters
     * when adding new cameras.
     * @return vertical coordinate of principal point for additional cameras 
     * when UPnP method is used for additional cameras estimation and neither 
     * DIAC or DAQ are estimated to find intrinsic parameters when adding new
     * cameras.
     */
    public double getAdditionalCamerasVerticalPrincipalPoint() {
        return mAdditionalCamerasVerticalPrincipalPoint;
    }
    
    /**
     * Sets vertical coordinate of principal point for additional cameras when 
     * UPnP (Uncalibrated Perspective-n-Point) method is used for additional
     * cameras estimation and neither Dual Image of Absolute Conic (DIAC) or
     * Dual Absolute Quadric (DAQ) are estimated to find intrinsic parameters
     * when adding new cameras.
     * @param additionalCamerasVerticalPrincipalPoint vertical coordinate of
     * principal point for additional cameras when UPnP method is used for
     * additional cameras estimation and neither DIAC or DAQ are estimated to
     * find intrinsic parameters when adding new cameras.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasVerticalPrincipalPoint(
                    double additionalCamerasVerticalPrincipalPoint) {
        mAdditionalCamerasVerticalPrincipalPoint = 
                additionalCamerasVerticalPrincipalPoint;
        return this;
    }
    
    /**
     * Indicates whether EPnP (Efficient Perspective-n-Point) method is used
     * for additional cameras estimation. Either EPnP or UPnP must be used for
     * additional cameras estimation.
     * @return true if EPnP method is used for additional cameras estimation,
     * false otherwise.
     */
    public boolean getUseEPnPForAdditionalCamerasEstimation() {
        return mUseEPnPForAdditionalCamerasEstimation;
    }
    
    /**
     * Specifies whether EPnP (Efficient Perspective-n-Point) method is used
     * for additional cameras estimation. Either EPnP or UPnP must be used for
     * additional cameras estimation.
     * @param useEPnPForAdditionalCamerasEstimation true if EPnP method is used
     * for additional cameras estimation, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setUseEPnPForAdditionalCamerasEstimation(
                    boolean useEPnPForAdditionalCamerasEstimation) {
        mUseEPnPForAdditionalCamerasEstimation = 
                useEPnPForAdditionalCamerasEstimation;
        return this;
    }

    /**
     * Indicates whether UPnP (Uncalibrated Perspective-n-Point) method is used
     * for additional cameras estimation. Either EPnP or UPnP must be used for
     * additional cameras estimation.
     * @return true if UPnP method is used for additional cameras estimation, 
     * false otherwise.
     */
    public boolean getUseUPnPForAdditionalCamerasEstimation() {
        return mUseUPnPForAdditionalCamerasEstimation;
    }
    
    /**
     * Specifies whether UPnP (Uncalibrated Perspective-n-Point) method is used
     * for additional cameras estimation. Either EPnP or UPnP must be used for
     * additional cameras estimation.
     * @param useUPnPForAdditionalCamerasEstimation true if UPnP method is used 
     * for additional cameras estimation, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
        setUseUPnPForAdditionalCamerasEstimation(
                boolean useUPnPForAdditionalCamerasEstimation) {
        mUseUPnPForAdditionalCamerasEstimation = 
                useUPnPForAdditionalCamerasEstimation;
        return this;
    }
        
    /**
     * Gets robust method to estimate additional cameras.
     * @return robust method to estimate additional cameras.
     */
    public RobustEstimatorMethod getAdditionalCamerasRobustEstimationMethod() {
        return mAdditionalCamerasRobustEstimationMethod;
    }
    
    /**
     * Sets robust method to estimate additional cameras.
     * @param method robust method to estimate additional cameras.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasRobustEstimationMethod(
                    RobustEstimatorMethod method) {
        mAdditionalCamerasRobustEstimationMethod = method;
        return this;
    }
            
    /**
     * Indicates whether planar configuration is allowed for additional cameras
     * estimation using either EPnP or UPnP.
     * @return true if planar configuration is allowed, false otherwise.
     */
    public boolean getAdditionalCamerasAllowPlanarConfiguration() {
        return mAdditionalCamerasAllowPlanarConfiguration;
    }
    
    /**
     * Specifies whether planar configuration is allowed for additional cameras
     * estimation using either EPnP or UPnP.
     * @param allowPlanarConfiguration true if planar configuration is allowed,
     * false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasAllowPlanarConfiguration(
                    boolean allowPlanarConfiguration) {
        mAdditionalCamerasAllowPlanarConfiguration = allowPlanarConfiguration;
        return this;
    }
    
    /**
     * Indicates whether dimension 2 nullspace is allowed while estimating
     * additional cameras using either EPnP or UPnP.
     * @return true if dimension 2 nullspace is allowed while estimating
     * additional cameras, false otherwise.
     */
    public boolean getAdditionalCamerasAllowNullspaceDimension2() {
        return mAdditionalCamerasAllowNullspaceDimension2;
    }
    
    /**
     * Specifies whether dimension 2 nullspace is allowed while estimating
     * additional cameras using either EPnP or UPnP.
     * @param allowNullspaceDimension2 true if dimension 2 nullspace is allowed
     * while estimating aditional cameras, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
        setAdditionalCamerasAllowNullspaceDimension2(
                boolean allowNullspaceDimension2) {
        mAdditionalCamerasAllowNullspaceDimension2 = allowNullspaceDimension2;
        return this;
    }
        
    /**
     * Indicates whether dimension 3 nullspace is allowed while estimating
     * additional cameras using EPnP.
     * @return true if dimension 3 nullspace is allowed while estimating
     * additional cameras, false otherwise.
     */
    public boolean getAdditionalCamerasAllowNullspaceDimension3() {
        return mAdditionalCamerasAllowNullspaceDimension3;
    }
    
    /**
     * Specifies whether dimension 3 nullspace is allowed while estimating
     * additional cameras using either EPnP or UPnP.
     * @param allowNullspaceDimension3 true if dimension 3 nullspace is allowed 
     * while estimating additional cameras, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasAllowNullspaceDimension3(
                    boolean allowNullspaceDimension3) {
        mAdditionalCamerasAllowNullspaceDimension3 = allowNullspaceDimension3;
        return this;
    }
       
    /**
     * Gets threshold to determine whether 3D matched points to estimate 
     * additional cameras are in a planar configuration.
     * @return threshold to determine whether 3D matched points to estimate
     * additional cameras are in a planar configuration.
     */
    public double getAdditionalCamerasPlanarThreshold() {
        return mAdditionalCamerasPlanarThreshold;
    }
    
    /**
     * Specifies threshold to determine whether 3D matched points to estimate
     * additional cameras are in a planar configuration.
     * @param additionalCamerasPlanarThreshold threshold to determine whether 3D
     * matched points to estimate additional cameras are in a planar 
     * configuration.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasPlanarThreshold(
            double additionalCamerasPlanarThreshold) {
        mAdditionalCamerasPlanarThreshold = additionalCamerasPlanarThreshold;
        return this;
    }
    
    /**
     * Indicates whether additional cameras are refined to minimize overall 
     * projection error among all found inliers.
     * @return true if additional cameras are refined, false otherwise.
     */
    public boolean areAdditionalCamerasRefined() {
        return mRefineAdditionalCameras;
    }
    
    /**
     * Specifies whether additional cameras are refined to minimize overall
     * projection error among all found inliers.
     * @param refineAdditionalCameras true if additional cameras are refined, 
     * false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasRefined(
            boolean refineAdditionalCameras) {
        mRefineAdditionalCameras = refineAdditionalCameras;
        return this;
    }

    /**
     * Indicates whether covariance is kept after refining result of additional
     * cameras estimation.
     * @return true if covariance is kept, false otherwise.
     */
    public boolean isAdditionalCamerasCovarianceKept() {
        return mKeepCovarianceAdditionalCameras;
    }
    
    /**
     * Specifies whether covariance is kept after refining result of additional
     * cameras estimation.
     * @param keepCovarianceAdditionalCameras true if covariance is kept, false
     * otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasCovarianceKept(
            boolean keepCovarianceAdditionalCameras) {
        mKeepCovarianceAdditionalCameras = keepCovarianceAdditionalCameras;
        return this;
    }

    /**
     * Gets value indicating whether fast refinement is used for additional 
     * cameras estimation.
     * @return true if fast refinement is used for additional cameras 
     * estimation, false otherwise.
     */
    public boolean getAdditionalCamerasUseFastRefinement() {
        return mAdditionalCamerasUseFastRefinement;
    }
    
    /**
     * Sets value indicating whether fast refinement is used for additional
     * cameras estimation.
     * @param additionalCamerasUseFastRefinement true if fast refinement is used
     * for additional cameras estimation, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
        setAdditionalCamerasUseFastRefinement(
                boolean additionalCamerasUseFastRefinement) {
        mAdditionalCamerasUseFastRefinement = 
                additionalCamerasUseFastRefinement;
        return this;
    }

    /**
     * Gets confidence of estimated additional cameras.
     * @return confidence of estimated additional cameras.
     */
    public double getAdditionalCamerasConfidence() {
        return mAdditionalCamerasConfidence;
    }
    
    /**
     * Sets confidence of estimated additional cameras.
     * @param additionalCamerasConfidence confidence of estimated additional
     * cameras.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasConfidence(
            double additionalCamerasConfidence) {
        mAdditionalCamerasConfidence = additionalCamerasConfidence;
        return this;
    }

    /**
     * Gets maximum allowed number of iterations for additional cameras 
     * estimation.
     * @return maximum allowed number of iterations for additional cameras 
     * estimation.
     */
    public double getAdditionalCamerasMaxIterations() {
        return mAdditionalCamerasMaxIterations;
    }
    
    /**
     * Sets maximum allowed number of iterations for additional cameras 
     * estimation.
     * @param additionalCamerasMaxIterations maximum allowed number of 
     * iterations for additional cameras estimation.
     * @return this instance so that method can be eaisly chained.
     */
    public SparseReconstructorConfiguration setAdditionalCamerasMaxIterations(
            double additionalCamerasMaxIterations) {
        mAdditionalCamerasMaxIterations = additionalCamerasMaxIterations;
        return this;
    }
    
    /**
     * Gets value indicating whether skewness is not suggested during additional
     * cameras estimation.
     * @return true if skewness is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestSkewnessValueEnabled() {
        return mAdditionalCamerasSuggestSkewnessValueEnabled;
    }
    
    /**
     * Sets value indicating whether skewness is not suggested during additional
     * cameras estimation.
     * @param additionalCamerasSuggestSkewnessValueEnabled true if skewness is 
     * suggested, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestSkewnessValueEnabled(
                    boolean additionalCamerasSuggestSkewnessValueEnabled) {
        mAdditionalCamerasSuggestSkewnessValueEnabled =
                additionalCamerasSuggestSkewnessValueEnabled;
        return this;
    }
    
    /**
     * Gets value of skewness to be suggested when suggestion is enabled during
     * additional cameras estimation.
     * @return value of skewness to be suggested when suggestion is enabled 
     * during additional cameras estimation.
     */
    public double getAdditionalCamerasSuggestedSkewnessValue() {
        return mAdditionalCamerasSuggestedSkewnessValue;
    }
    
    /**
     * Sets value of skewness to be suggested when suggestion is enabled during
     * additional cameras estimation.
     * @param additionalCamerasSuggestedSkewnessValue value of skewness to be 
     * suggested when suggestion is enabled during additional cameras 
     * estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestedSkewnessValue(
                    double additionalCamerasSuggestedSkewnessValue) {
        mAdditionalCamerasSuggestedSkewnessValue = 
                additionalCamerasSuggestedSkewnessValue;
        return this;
    }

    /**
     * Indicates whether horizontal focal length value is suggested or not 
     * during additional cameras estimation.
     * @return true if horizontal focal length value is suggested, false 
     * otherwise.
     */
    public boolean isAdditionalCamerasSuggestHorizontalFocalLengthEnabled() {
        return mAdditionalCamerasSuggestHorizontalFocalLengthEnabled;
    }
    
    /**
     * Specifies whether horizontal focal length value is suggested or not 
     * during additional cameras estimation.
     * @param additionalCamerasSuggestHorizontalFocalLengthEnabled true if 
     * horizontal focal length value is suggested, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(
            boolean additionalCamerasSuggestHorizontalFocalLengthEnabled) {
        mAdditionalCamerasSuggestHorizontalFocalLengthEnabled = 
                additionalCamerasSuggestHorizontalFocalLengthEnabled;    
        return this;
    }

    /**
     * Gets value of suggested horizontal focal length during additional cameras
     * estimation.
     * @return value of suggested horizontal focal length during additional
     * cameras estimation.
     */
    public double getAdditionalCamerasSuggestedHorizontalFocalLengthValue() {
        return mAdditionalCamerasSuggestedHorizontalFocalLengthValue;
    }
    
    /**
     * Sets value of suggested horizontal focal length during additional cameras
     * estimation.
     * @param additionalCamerasSuggestedHorizontalFocalLengthValue value of
     * suggested horizontal focal length during additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestedHorizontalFocalLengthValue(
            double additionalCamerasSuggestedHorizontalFocalLengthValue) {
        mAdditionalCamerasSuggestedHorizontalFocalLengthValue =
                additionalCamerasSuggestedHorizontalFocalLengthValue;
        return this;
    }

    /**
     * Gets value indicating whether vertical focal length value is suggested or
     * not during additional cameras estimation.
     * @return true if vertical focal length value is suggested, false 
     * otherwise.
     */
    public boolean isAdditionalCamerasSuggestVerticalFocalLengthEnabled() {
        return mAdditionalCamerasSuggestVerticalFocalLengthEnabled;
    }
    
    /**
     * Sets value indicating whether vertical focal length value is suggested or
     * not during additional cameras estimation.
     * @param additionalCamerasSuggestVerticalFocalLengthEnabled true if 
     * vertical focal length value is suggested, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestVerticalFocalLengthEnabled(
            boolean additionalCamerasSuggestVerticalFocalLengthEnabled) {
        mAdditionalCamerasSuggestVerticalFocalLengthEnabled =
                additionalCamerasSuggestVerticalFocalLengthEnabled;
        return this;
    }

    /**
     * Gets value of suggested vertical focal length during additional cameras
     * estimation.
     * @return value of suggested vertical focal length during additional 
     * cameras estimation.
     */
    public double getAdditionalCamerasSuggestedVerticalFocalLengthValue() {
        return mAdditionalCamerasSuggestedVerticalFocalLengthValue;
    }
    
    /**
     * Sets value of suggested vertical focal length during additional cameras
     * estimation.
     * @param additionalCamerasSuggestedVerticalFocalLengthValue value of 
     * suggested vertical focal length during additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestedVerticalFocalLengthValue(
                    double additionalCamerasSuggestedVerticalFocalLengthValue) {
        mAdditionalCamerasSuggestedVerticalFocalLengthValue = 
                additionalCamerasSuggestedVerticalFocalLengthValue;
        return this;
    }

    /**
     * Gets value indicating whether aspect ratio is suggested or not during
     * additional cameras estimation.
     * @return true if aspect ratio is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestAspectRatioEnabled() {
        return mAdditionalCamerasSuggestAspectRatioEnabled;
    }
    
    /**
     * Sets value indicating whether aspect ratio is suggested or not during
     * additional cameras estimation.
     * @param additionalCamerasSuggestAspectRatioEnabled true if aspect ratio is
     * suggested, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestAspectRatioEnabled(
                    boolean additionalCamerasSuggestAspectRatioEnabled) {
        mAdditionalCamerasSuggestAspectRatioEnabled = 
                additionalCamerasSuggestAspectRatioEnabled;
        return this;
    }

    /**
     * Gets value of aspect ratio to be suggested when suggestion is enabled
     * during additional cameras estimation.
     * @return value of aspect ratio to be suggested.
     */
    public double getAdditionalCamerasSuggestedAspectRatioValue() {
        return mAdditionalCamerasSuggestedAspectRatioValue;
    }
    
    /**
     * Sets value of aspect ratio to be suggested when suggestion is enabled
     * during additional cameras estimation.
     * @param additionalCamerasSuggestedAspectRatioValue value of aspect ratio 
     * to be suggested.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestedAspectRatioValue(
                    double additionalCamerasSuggestedAspectRatioValue) {
        mAdditionalCamerasSuggestedAspectRatioValue = 
                additionalCamerasSuggestedAspectRatioValue;
        return this;
    }

    /**
     * Gets value indicating whether principal point is suggested or not during
     * additional cameras estimation.
     * @return true if principal point is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestPrincipalPointEnabled() {
        return mAdditionalCamerasSuggestPrincipalPointEnabled;
    }
    
    /**
     * Sets value indicating whether principal point is suggested or not during
     * additional cameras estimation.
     * @param additionalCamerasSuggestPrincipalPointEnabled true if principal 
     * point is suggested, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestPrincipalPointEnabled(
                    boolean additionalCamerasSuggestPrincipalPointEnabled) {
        mAdditionalCamerasSuggestPrincipalPointEnabled =
                additionalCamerasSuggestPrincipalPointEnabled;
        return this;
    }
    
    /**
     * Gets value of principal point to be suggested when suggestion is enabled
     * during additional cameras estimation.
     * @return principal point to be suggested.
     */
    public InhomogeneousPoint2D 
            getAdditionalCamerasSuggestedPrincipalPointValue() {
        return mAdditionalCamerasSuggestedPrincipalPointValue;
    }
         
    /**
     * Sets value of principal point to be suggested when suggestion is enabled
     * during additional cameras estimation.
     * @param additionalCamerasSuggestedPrincipalPointValue principal point to
     * be suggested.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration 
            setAdditionalCamerasSuggestedPrincipalPointValue(
                    InhomogeneousPoint2D additionalCamerasSuggestedPrincipalPointValue) {
        mAdditionalCamerasSuggestedPrincipalPointValue = 
                additionalCamerasSuggestedPrincipalPointValue;
        return this;
    }    
            
    /**
     * Indicates whether homogeneous point triangulator must be used or not to
     * estimate 3D points when only two matches are available.
     * @return true if homogeneous point triangulator must be used, false 
     * otherwise.
     */
    public boolean isHomogeneousPointTriangulatorUsed() {
        return mUseHomogeneousPointTriangulator;
    }
    
    /**
     * Specifies whether homogeneous point triangulator must be used or not to
     * estimate 3D points when only two matches are available.
     * @param useHomogeneousPointTriangulator true if homogeneous point 
     * triangulator must be used, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setHomogeneousPointTriangulatorUsed(
            boolean useHomogeneousPointTriangulator) {
        mUseHomogeneousPointTriangulator = useHomogeneousPointTriangulator;
        return this;
    }

    /**
     * Gets robust method for point triangulation when points are matched in
     * more thatn two views.
     * @return robust method for point triangulation.
     */
    public RobustEstimatorMethod getRobustPointTriangulatorMethod() {
        return mRobustPointTriangulatorMethod;
    }
    
    /**
     * Sets robust method for point triangulation when points are matched in
     * more than two views.
     * @param robustPointTriangulatorMethod robust method for point 
     * triangulation.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setRobustPointTriangulatorMethod(
            RobustEstimatorMethod robustPointTriangulatorMethod) {
        mRobustPointTriangulatorMethod = robustPointTriangulatorMethod;
        return this;
    }
    
    /**
     * Gets confidence of robustly triangulated points. By default this is 99%.
     * @return confidence of robustly triangulated points.
     */
    public double getPointTriangulatorConfidence() {
        return mPointTriangulatorConfidence;
    }
    
    /**
     * Sets confidence of robustly triangulated points. By default this is 99%.
     * @param pointTriangulatorConfidence confidence of robustly triangulated 
     * points.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setPointTriangulatorConfidence(
            double pointTriangulatorConfidence) {
        mPointTriangulatorConfidence = pointTriangulatorConfidence;
        return this;
    }
    
    /**
     * Gets maximum number of iterations to make while robustly estimating
     * triangulated points. By default this is 5000 iterations.
     * @return maximum number of iterations to make while robustly estimating
     * triangulated points.
     */
    public int getPointTriangulatorMaxIterations() {
        return mPointTriangulatorMaxIterations;
    }
    
    /**
     * Sets maximum number of iterations to make while robustly estimating
     * triangulated points. By default this is 5000 iterations.
     * @param pointTriangulatorMaxIterations maximum number of iterations to
     * make while robustly estimating triangulated points.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setPointTriangulatorMaxIterations(
            int pointTriangulatorMaxIterations) {
        mPointTriangulatorMaxIterations = pointTriangulatorMaxIterations;
        return this;
    }

    /**
     * Gets threshold to determine whether samples for robust point triangulator
     * are inliers or not.
     * @return threshold to determine whether samples for robust point 
     * triangulator are inliers or not.
     */
    public double getPointTriangulatorThreshold() {
        return mPointTriangulatorThreshold;
    }
    
    /**
     * Sets threshold to determine whether samples for robust point triangulator
     * are inliers or not.
     * @param pointTriangulatorThreshold threshold to determine whether samples
     * for robust point triangulator are inliers or not.
     * @return this instance so that method can be easily chained.
     */
    public SparseReconstructorConfiguration setPointTriangulatorThreshold(
            double pointTriangulatorThreshold) {
        mPointTriangulatorThreshold = pointTriangulatorThreshold;
        return this;
    }            
    
    //TODO: refine triangulated points
    //TODO: keep covariance of triangulated points
    //TODO: use fast refined triangulated points?
    
    //TODO: bundle adjustment (refine cameras and points as a last step?)
}
