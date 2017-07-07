/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.GoldStandardCorrector
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 28, 2015
 */
package com.irurueta.geometry.epipolar;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
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

public class GoldStandardCorrectorTest implements CorrectorListener{
    
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
    
    public static final int MIN_POINTS = 100;
    public static final int MAX_POINTS = 500;
    
    public static final int TIMES = 100;
    
    private int correctStart;
    private int correctEnd;
    private int correctProgressChange;    
    
    public GoldStandardCorrectorTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor() throws WrongSizeException, NotReadyException, 
            LockedException, DecomposerException, NotAvailableException, 
            InvalidFundamentalMatrixException, 
            com.irurueta.algebra.LockedException, com.irurueta.algebra.NotReadyException, com.irurueta.algebra.NotAvailableException{
        //test constructor without arguments
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(), 
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        //test constructor with fundamental matrix
        FundamentalMatrix emptyFundamentalMatrix = new FundamentalMatrix();
        corrector = new GoldStandardCorrector(emptyFundamentalMatrix);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        //test constructor with left and right points
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        
        corrector = new GoldStandardCorrector(leftPoints, rightPoints);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertNull(corrector.getFundamentalMatrix());
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        //Force IllegalArgumentException
        List<Point2D> badPoints = new ArrayList<Point2D>();
        badPoints.add(Point2D.create());
        
        corrector = null;
        try{
            corrector = new GoldStandardCorrector(badPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            corrector = new GoldStandardCorrector(leftPoints, badPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(corrector);
        
        //test constructor with left and right points and fundamental matrix
        corrector = new GoldStandardCorrector(leftPoints, rightPoints, 
                emptyFundamentalMatrix);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady()); //fundamental matrix not defined
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
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
        
        corrector = new GoldStandardCorrector(leftPoints, rightPoints, 
                fundamentalMatrix);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertTrue(corrector.isReady()); //fundamental matrix not defined
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);        
        
        corrector = null;
        try{
            corrector = new GoldStandardCorrector(badPoints, rightPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            corrector = new GoldStandardCorrector(leftPoints, badPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(corrector);
        
        //test constructor with listener
        corrector = new GoldStandardCorrector(this);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        //test constructor with fundamental matrix and listener
        corrector = new GoldStandardCorrector(fundamentalMatrix, this);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        //test constructor with left and right points and listener
        corrector = new GoldStandardCorrector(leftPoints, rightPoints, this);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertNull(corrector.getFundamentalMatrix());
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        corrector = null;
        try{
            corrector = new GoldStandardCorrector(badPoints, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            corrector = new GoldStandardCorrector(leftPoints, badPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(corrector);
        
        //test constructor with left and right points, fundamental matrix
        //and listener
        corrector = new GoldStandardCorrector(leftPoints, rightPoints, 
                emptyFundamentalMatrix, this);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady()); //fundamental matrix not defined
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        //test constructor with left and right points, valid fundamental matrix
        //and listener
        corrector = new GoldStandardCorrector(leftPoints, rightPoints, 
                fundamentalMatrix, this);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(), 
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);        
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertTrue(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        
        corrector = null;
        try{
            corrector = new GoldStandardCorrector(badPoints, rightPoints, 
                    fundamentalMatrix, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            corrector = new GoldStandardCorrector(leftPoints, badPoints, 
                    fundamentalMatrix, this);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(corrector);        
    }
    
    @Test
    public void testIsFallbackToSampsonEnabled() throws LockedException{
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check default value
        assertEquals(corrector.isFallbackToSampsonEnabled(),
                GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);
        
        //set new value
        corrector.setFallbackToSampsonEnabled(
                !GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);
        
        //check correctness
        assertEquals(corrector.isFallbackToSampsonEnabled(),
                !GoldStandardCorrector.DEFAULT_FALLBACK_TO_SAMPSON_ENABLED);
    }
    
    @Test
    public void testGetSetFundamentalMatrix() throws LockedException{
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check default value
        assertNull(corrector.getFundamentalMatrix());
        
        //set new value
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        corrector.setFundamentalMatrix(fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
    }
    
    @Test
    public void testGetSetLeftAndRightPoints() throws LockedException{
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        
        //set new values
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        
        corrector.setLeftAndRightPoints(leftPoints, rightPoints);
        
        //check correctness
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        
        //Force IllegalArgumentException
        List<Point2D> badPoints = new ArrayList<Point2D>();
        badPoints.add(Point2D.create());
        
        try{
            corrector.setLeftAndRightPoints(leftPoints, badPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            corrector.setLeftAndRightPoints(badPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetPointsAndFundamentalMatrix() throws LockedException{
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getFundamentalMatrix());
        
        //set new values
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        
        corrector.setPointsAndFundamentalMatrix(leftPoints, rightPoints, 
                fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        
        //Force IllegalArgumentException
        List<Point2D> badPoints = new ArrayList<Point2D>();
        badPoints.add(Point2D.create());
        
        try{
            corrector.setPointsAndFundamentalMatrix(badPoints, rightPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            corrector.setPointsAndFundamentalMatrix(leftPoints, badPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException{
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check default values
        assertEquals(corrector.getProgressDelta(), 
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        corrector.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(corrector.getProgressDelta(), 0.5, 0.0);
    }
    
    @Test
    public void testGetSetListener() throws LockedException{
        GoldStandardCorrector corrector = new GoldStandardCorrector();
        
        //check default values
        assertNull(corrector.getListener());
        
        //set new value
        corrector.setListener(this);
        
        //check correctness
        assertSame(corrector.getListener(), this);
    }
    
    @Test
    public void testAreValidPoints(){
        List<Point2D> leftPoints = new ArrayList<Point2D>();
        List<Point2D> rightPoints = new ArrayList<Point2D>();
        
        assertFalse(GoldStandardCorrector.areValidPoints(null, null));
        assertFalse(GoldStandardCorrector.areValidPoints(leftPoints, null));
        assertFalse(GoldStandardCorrector.areValidPoints(null, rightPoints));
        assertTrue(GoldStandardCorrector.areValidPoints(leftPoints, 
                rightPoints));
    }
    
    @Test
    public void testCorrect() throws InvalidPairOfCamerasException, 
            com.irurueta.geometry.estimators.NotReadyException, LockedException, 
            CorrectionException{
        
        int improved = 0, total = 0;
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
            
            //create random list of 3D points to project
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> pointsToProject = new ArrayList<Point3D>(nPoints);
            for(int i = 0; i < nPoints; i++){
                pointsToProject.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }
            total += nPoints;
            
            //create two cameras
            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    cameraCenter1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    cameraCenter2);
            
            //project 3D points with both cameras
            List<Point2D> leftPoints = camera1.project(pointsToProject);
            List<Point2D> rightPoints = camera2.project(pointsToProject);
            
            //add error to projected points
            List<Point2D> wrongLeftPoints = new ArrayList<Point2D>(nPoints);
            List<Point2D> wrongRightPoints = new ArrayList<Point2D>(nPoints);
            for(int i = 0; i < nPoints; i++){
                double errorLeftX = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                        MAX_PROJECTED_ERROR);
                double errorLeftY = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                        MAX_PROJECTED_ERROR);
                double errorRightX = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                        MAX_PROJECTED_ERROR);
                double errorRightY = randomizer.nextDouble(MIN_PROJECTED_ERROR,
                        MAX_PROJECTED_ERROR);
                
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                
                Point2D wrongLeftPoint = new HomogeneousPoint2D(
                        leftPoint.getInhomX() + errorLeftX, 
                        leftPoint.getInhomY() + errorLeftY, 1.0);
                Point2D wrongRightPoint = new HomogeneousPoint2D(
                        rightPoint.getInhomX() + errorRightX, 
                        rightPoint.getInhomY() + errorRightY, 1.0);
                
                wrongLeftPoints.add(wrongLeftPoint);
                wrongRightPoints.add(wrongRightPoint);
            }
            
            //create fundamental matrix for the same pair of cameras used to
            //project points
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);            
            
            //check that points without error belong to epipolar lines
            for(int i = 0; i < nPoints; i++){
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                
                Line2D rightEpipolarLine = 
                        fundamentalMatrix.getRightEpipolarLine(leftPoint);
                Line2D leftEpipolarLine =
                        fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                
                assertTrue(rightEpipolarLine.isLocus(rightPoint, 
                        ABSOLUTE_ERROR));
                assertTrue(leftEpipolarLine.isLocus(leftPoint, 
                        2.0*ABSOLUTE_ERROR));
            }     
            
            //use corrector to fix points with error
            GoldStandardCorrector corrector = new GoldStandardCorrector(
                    wrongLeftPoints, wrongRightPoints, fundamentalMatrix, this);
            
            assertTrue(corrector.isReady());
            assertFalse(corrector.isLocked());
            assertEquals(correctStart, 0);
            assertEquals(correctEnd, 0);
            assertEquals(correctProgressChange, 0);
            
            corrector.correct();
            
            assertTrue(corrector.isReady());
            assertFalse(corrector.isLocked());
            assertEquals(correctStart, 1);
            assertEquals(correctEnd, 1);
            assertTrue(correctProgressChange > 0);
            reset();
            
            List<Point2D> correctedLeftPoints = 
                    corrector.getLeftCorrectedPoints();
            List<Point2D> correctedRightPoints =
                    corrector.getRightCorrectedPoints();
            
            //check correctness
            for(int i = 0; i < nPoints; i++){
                Point2D correctedLeftPoint = correctedLeftPoints.get(i);
                Point2D correctedRightPoint = correctedRightPoints.get(i);
                
                Point2D wrongLeftPoint = wrongLeftPoints.get(i);
                Point2D wrongRightPoint = wrongRightPoints.get(i);
                
                Line2D rightEpipolarLine = 
                        fundamentalMatrix.getRightEpipolarLine(
                        correctedLeftPoint);
                Line2D leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                        correctedRightPoint);
                
                double correctedDistanceLeft = leftEpipolarLine.signedDistance(
                        correctedLeftPoint);
                double correctedDistanceRight = 
                        rightEpipolarLine.signedDistance(correctedRightPoint);
                
                rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(
                        wrongLeftPoint);
                leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                        wrongRightPoint);
                
                double wrongDistanceLeft = leftEpipolarLine.signedDistance(
                        wrongLeftPoint);
                double wrongDistanceRight = rightEpipolarLine.signedDistance(
                        wrongRightPoint);
                
                //check that corrector has indeed reduced the amount of
                //projection error
                if((Math.abs(correctedDistanceLeft) <=
                        Math.abs(wrongDistanceLeft)) && 
                        (Math.abs(correctedDistanceRight) <=
                        Math.abs(wrongDistanceRight))){
                    improved++;
                }
            }
        }
        
        assertTrue(improved > 3 * total / 4);
    }
    
    private void reset(){
        correctStart = correctEnd = correctProgressChange = 0;
    }
    
    @Override
    public void onCorrectStart(Corrector corrector) {
        correctStart++;
        testLocked((GoldStandardCorrector)corrector);
    }

    @Override
    public void onCorrectEnd(Corrector corrector) {
        correctEnd++;
        testLocked((GoldStandardCorrector)corrector);
    }

    @Override
    public void onCorrectProgressChange(Corrector corrector, float progress) {
        correctProgressChange++;
        testLocked((GoldStandardCorrector)corrector);
    }
    
    private void testLocked(GoldStandardCorrector corrector){
        try{
            corrector.setFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            corrector.setLeftAndRightPoints(null, null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            corrector.setListener(this);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            corrector.setPointsAndFundamentalMatrix(null, null, null);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            corrector.setProgressDelta(0.5f);
        }catch(LockedException e){}
        try{
            corrector.setFallbackToSampsonEnabled(true);
            fail("LockedException expected but not thrown");
        }catch(LockedException e){}
        try{
            corrector.correct();
            fail("LockedException expected but not thrown");
        }catch(LockedException e){
        }catch(Exception e){
            fail("LockedException expected but not thrown");
        }
        assertTrue(corrector.isLocked());
    }    
}
