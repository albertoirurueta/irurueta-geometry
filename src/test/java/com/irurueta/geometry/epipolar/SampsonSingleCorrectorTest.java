/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.SampsonSingleCorrector
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 27, 2015
 */
package com.irurueta.geometry.epipolar;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.LockedException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class SampsonSingleCorrectorTest {
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final double MIN_PROJECTED_ERROR = -1.0;
    public static final double MAX_PROJECTED_ERROR = 1.0;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    public static final double MIN_FOCAL_LENGTH = 1.0;
    public static final double MAX_FOCAL_LENGTH = 100.0;
    public static final double MIN_SKEWNESS = -1.0;
    public static final double MAX_SKEWNESS = 1.0;
    public static final double MIN_PRINCIPAL_POINT = 0.0;
    public static final double MAX_PRINCIPAL_POINT = 100.0;
    public static final double MIN_ANGLE_DEGREES = -30.0;
    public static final double MAX_ANGLE_DEGREES = 30.0;
    public static final double MIN_CAMERA_SEPARATION = 5.0;
    public static final double MAX_CAMERA_SEPARATION = 10.0;
    
    public static final int TIMES = 100;
    
    public SampsonSingleCorrectorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}
    
    @Test
    public void testConstructor() throws WrongSizeException, 
            NotAvailableException, NotReadyException, LockedException, 
            DecomposerException, InvalidFundamentalMatrixException{
        //test constructor without arguments
        SampsonSingleCorrector corrector = new SampsonSingleCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with fundamental matrix
        FundamentalMatrix emptyFundamentalMatrix = new FundamentalMatrix();
        corrector = new SampsonSingleCorrector(emptyFundamentalMatrix);
        
        //check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with left and right points
        Point2D leftPoint = Point2D.create();
        Point2D rightPoint = Point2D.create();
        corrector = new SampsonSingleCorrector(leftPoint, rightPoint);
        
        //check default values
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with left and right points and a valid fundamental 
        //matrix
        FundamentalMatrix fundamentalMatrix = null;
        int rank;
        do{
            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS, 
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
        
            //ensure that internal matrix has rank 2
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);
            decomposer.decompose();
            
            rank = decomposer.getRank(); //if rank is less than 2 we need to 
                                        //pick another random matrix
        
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();

            //set last element to 0 to force rank 2
            W.setElementAt(2, 2, 0.0);
            
            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));
            
            fundamentalMatrix = new FundamentalMatrix(internalMatrix);
        }while(rank < 2);
        
        corrector = new SampsonSingleCorrector(leftPoint, rightPoint, 
                fundamentalMatrix);
        
        //check default values
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertTrue(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //if we use a fundamental matrix without internal matrix defined, then
        //corrector is not ready
        corrector = new SampsonSingleCorrector(leftPoint, rightPoint, 
                emptyFundamentalMatrix);
        
        //check default values
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);        
    }
    
    @Test
    public void testGetSetPointsAndFundamentalMatrix(){
        SampsonSingleCorrector corrector = new SampsonSingleCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        
        //set new values
        Point2D leftPoint = Point2D.create();
        Point2D rightPoint = Point2D.create();
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        
        corrector.setPointsAndFundamentalMatrix(leftPoint, rightPoint, 
                fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
    }
    
    @Test
    public void testGetSetFundamentalMatrix(){
        SampsonSingleCorrector corrector = new SampsonSingleCorrector();
        
        //check default value
        assertNull(corrector.getFundamentalMatrix());
        
        //set new value
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();        
        corrector.setFundamentalMatrix(fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
    }

    @Test
    public void testGetSetPoints(){
        SampsonSingleCorrector corrector = new SampsonSingleCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        
        //set new values
        Point2D leftPoint = Point2D.create();
        Point2D rightPoint = Point2D.create();
        
        corrector.setPoints(leftPoint, rightPoint);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
    }
    
    @Test
    public void testCorrect() throws InvalidPairOfCamerasException, 
            com.irurueta.geometry.estimators.NotReadyException{
        int numImproved = 0;
        for(int t = 0; t < TIMES; t++){
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            
            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                    verticalFocalLength2, horizontalPrincipalPoint2,
                    verticalPrincipalPoint2, skewness2);
            
            //camera centers
            Point3D cameraCenter1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D cameraCenter2 = new InhomogeneousPoint3D(
                    cameraCenter1.getInhomX() + cameraSeparation,
                    cameraCenter1.getInhomY() + cameraSeparation,
                    cameraCenter1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
            //create random 3D point to project
            Point3D pointToProject = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            
            //create two cameras
            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    cameraCenter1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    cameraCenter2);
            
            //project 3D point with both cameras
            Point2D leftPoint = camera1.project(pointToProject);
            Point2D rightPoint = camera2.project(pointToProject);
            
            //add error to projected points
            double errorLeftX = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                    MAX_PROJECTED_ERROR);
            double errorLeftY = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                    MAX_PROJECTED_ERROR);
            double errorRightX = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                    MAX_PROJECTED_ERROR);
            double errorRightY = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                    MAX_PROJECTED_ERROR);
            
            Point2D wrongLeftPoint = new HomogeneousPoint2D(
                    leftPoint.getInhomX() + errorLeftX,
                    leftPoint.getInhomY() + errorLeftY, 1.0);
            Point2D wrongRightPoint = new HomogeneousPoint2D(
                    rightPoint.getInhomX() + errorRightX,
                    rightPoint.getInhomY() + errorRightY, 1.0);
            
            //create fundamental matrix for the same pair of cameras used to
            //project point
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
            
            //check that points without error belong to epipolar lines
            Line2D rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(
                    leftPoint);
            Line2D leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                    rightPoint);
            assertTrue(rightEpipolarLine.isLocus(rightPoint, ABSOLUTE_ERROR));
            assertTrue(leftEpipolarLine.isLocus(leftPoint, ABSOLUTE_ERROR));
            
            fundamentalMatrix.normalize();
            
            //use corrector to fix points with error
            SampsonSingleCorrector corrector = new SampsonSingleCorrector(
                    wrongLeftPoint, wrongRightPoint, fundamentalMatrix);
            
            assertTrue(corrector.isReady());
            
            corrector.correct();
            
            Point2D correctedLeftPoint = corrector.getLeftCorrectedPoint();
            Point2D correctedRightPoint = corrector.getRightCorrectedPoint();
            
            rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(
                    correctedLeftPoint);
            leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                    correctedRightPoint);
            
            double correctedDistanceLeft = leftEpipolarLine.signedDistance(
                    correctedLeftPoint);
            double correctedDistanceRight = rightEpipolarLine.signedDistance(
                    correctedRightPoint);
            
            rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(
                    wrongLeftPoint);
            leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                    wrongRightPoint);
            
            double wrongDistanceLeft = leftEpipolarLine.signedDistance(
                    wrongLeftPoint);
            double wrongDistanceRight = rightEpipolarLine.signedDistance(
                    wrongRightPoint);
            
            //check that corrector has indeed reduced the amount of projection
            //error
            if (Math.abs(correctedDistanceLeft) > Math.abs(wrongDistanceLeft)) {
                continue;
            }
            assertTrue(Math.abs(correctedDistanceLeft) <= 
                    Math.abs(wrongDistanceLeft));
            if (Math.abs(correctedDistanceRight) > Math.abs(wrongDistanceRight)) {
                continue;
            }
            assertTrue(Math.abs(correctedDistanceRight) <= 
                    Math.abs(wrongDistanceRight));
            
            numImproved++;
        }
        assertTrue(numImproved > 0);
    }
}
