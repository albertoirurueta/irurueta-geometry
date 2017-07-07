/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.CameraCalibratorSample
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 7, 2015
 */
package com.irurueta.geometry.calib3d;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
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

public class CameraCalibratorSampleTest {
    
    public static final double MIN_RANDOM_VALUE = -1.0;
    public static final double MAX_RANDOM_VALUE = 1.0;
    
    public static final int MIN_NUM_MARKERS = 4;
    public static final int MAX_NUM_MARKERS = 50;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-4;
    public static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;
    public static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;
    
    public static final double MIN_FOCAL_LENGTH = 3.0;
    public static final double MAX_FOCAL_LENGTH = 10.0;
    
    public static final double MIN_ANGLE_DEGREES = -10.0;
    public static final double MAX_ANGLE_DEGREES = 10.0;   
    
    public static final int INHOM_3D_COORDS = 3;  
    
    public static final int TIMES = 1000;
    
    public CameraCalibratorSampleTest() {}
    
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
        //test constructor without arguments
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check correctness        
        assertNull(sample.getPattern());
        assertNull(sample.getSampledMarkers());
        assertNull(sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());
        
        //test constructor with sampled markers
        List<Point2D> sampledMarkers = new ArrayList<Point2D>();
        for(int i = 0; i < 4; i++){
            sampledMarkers.add(Point2D.create());
        }
        sample = new CameraCalibratorSample(sampledMarkers);
        
        //check correctness
        assertNull(sample.getPattern());
        assertSame(sample.getSampledMarkers(), sampledMarkers);
        assertNull(sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());
        
        //Force IllegalArgumentException
        List<Point2D> emptyMarkers = new ArrayList<Point2D>();
        sample = null;
        try{
            sample = new CameraCalibratorSample(emptyMarkers);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(sample);
        
        //test constructor with sampled markers and quality scores
        double[] qualityScores = new double[4];
        sample = new CameraCalibratorSample(sampledMarkers, qualityScores);
        
        //check correctness
        assertNull(sample.getPattern());
        assertSame(sample.getSampledMarkers(), sampledMarkers);
        assertSame(sample.getSampledMarkersQualityScores(), qualityScores);
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());
        
        //Force IllegalArgumentException
        double[] shortQualityScores = new double[1];
        sample = null;
        try{
            sample = new CameraCalibratorSample(sampledMarkers, 
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(sample);
        
        //test constructor with pattern and sampled markers
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        sample = new CameraCalibratorSample(pattern, sampledMarkers);
        
        //check correctness
        assertSame(sample.getPattern(), pattern);
        assertSame(sample.getSampledMarkers(), sampledMarkers);
        assertNull(sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());
        
        //Force IllegalArgumentException
        sample = null;
        try{
            sample = new CameraCalibratorSample(pattern, emptyMarkers);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(sample);
        
        //test constructor with sampled markers and quality scores
        sample = new CameraCalibratorSample(pattern, sampledMarkers, 
                qualityScores);
        
        //check correctness
        assertSame(sample.getPattern(), pattern);
        assertSame(sample.getSampledMarkers(), sampledMarkers);
        assertSame(sample.getSampledMarkersQualityScores(), qualityScores);
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());
        
        //Force IllegalArgumentException
        sample = null;
        try{
            sample = new CameraCalibratorSample(pattern, sampledMarkers, 
                    shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(sample);        
    }
    
    @Test
    public void testGetSetPattern(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        assertNull(sample.getPattern());
        
        //set new value
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        sample.setPattern(pattern);
        
        //check correctness
        assertSame(sample.getPattern(), pattern);
    }
    
    @Test
    public void testGetSetSampledMarkers(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check default value
        assertNull(sample.getSampledMarkers());
        
        //set new value
        List<Point2D> sampledMarkers = new ArrayList<Point2D>();
        for(int i = 0; i < 4; i++){
            sampledMarkers.add(Point2D.create());
        }
        sample.setSampledMarkers(sampledMarkers);
        
        //check correctness
        assertSame(sample.getSampledMarkers(), sampledMarkers);
        
        //Force IllegalArgumentException
        List<Point2D> emptyMarkers = new ArrayList<Point2D>();
        try{
            sample.setSampledMarkers(emptyMarkers);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetSampledMarkersQualityScores(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check default value
        assertNull(sample.getSampledMarkersQualityScores());
        
        //set new value
        double[] qualityScores = new double[4];
        sample.setSampledMarkersQualityScores(qualityScores);
        
        //check correctness
        assertSame(sample.getSampledMarkersQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] shortScores = new double[1];
        try{
            sample.setSampledMarkersQualityScores(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testComputeSampledMarkersQualityScores(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        int numMarkers = randomizer.nextInt(MIN_NUM_MARKERS, MAX_NUM_MARKERS);
        
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        List<Point2D> sampledMarkers = new ArrayList<Point2D>();
        Point2D marker;
        double[] scoresWithCenter = new double[numMarkers];
        double[] scoresNoCenter = new double[numMarkers];
        double distance;
        for(int i = 0; i < numMarkers; i++){
            marker = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            sampledMarkers.add(marker);
            
            //distance with center at origin of coordinates
            distance = Math.sqrt(Math.pow(marker.getInhomX(), 2.0) +
                    Math.pow(marker.getInhomY(), 2.0));
            scoresNoCenter[i] = 1.0 / (1.0 + distance);
            
            //distance respect to center
            distance = Math.sqrt(Math.pow(marker.getInhomX() - center.getInhomX(), 2.0) +
                    Math.pow(marker.getInhomY() - center.getInhomY(), 2.0));
            scoresWithCenter[i] = 1.0 / (1.0 + distance);
        }
        
        //check correctness
        assertArrayEquals(scoresNoCenter, 
                CameraCalibratorSample.computeSampledMarkersQualityScores(
                sampledMarkers), ABSOLUTE_ERROR);
            
        assertArrayEquals(scoresWithCenter,
                CameraCalibratorSample.computeSampledMarkersQualityScores(
                sampledMarkers, center), ABSOLUTE_ERROR);
    }
        
    @Test
    public void testGetSetUndistortedMarkers(){
        List<Point2D> sampledMarkers = new ArrayList<Point2D>();
        for(int i = 0; i < 4; i++){
            sampledMarkers.add(Point2D.create());
        }        
        CameraCalibratorSample sample = new CameraCalibratorSample(
                sampledMarkers);
        
        //check default value
        assertNull(sample.getUndistortedMarkers());
        
        //set new value
        List<Point2D> undistortedMarkers = new ArrayList<Point2D>();
        for(int i = 0; i < 4; i++){
            undistortedMarkers.add(Point2D.create());
        }
        sample.setUndistortedMarkers(undistortedMarkers);
        
        //check correctness
        assertSame(sample.getUndistortedMarkers(), undistortedMarkers);        
    }
    
    @Test
    public void testGetSetHomography(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check default value
        assertNull(sample.getHomography());
        
        //set new value
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        sample.setHomography(homography);
        
        //check correctness
        assertSame(sample.getHomography(), homography);
    }
    
    @Test
    public void testGetSetRotation(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check default value
        assertNull(sample.getRotation());
        
        //set new value
        Rotation3D r = Rotation3D.create();
        sample.setRotation(r);
        
        //check correctness
        assertSame(sample.getRotation(), r);
    }
    
    @Test
    public void testGetSetCameraCenter(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check default value
        assertNull(sample.getCameraCenter());
        
        //set new value
        Point3D center = Point3D.create();
        sample.setCameraCenter(center);
        
        //check correctness
        assertSame(sample.getCameraCenter(), center);
    }
    
    @Test
    public void testGetSetCamera(){
        CameraCalibratorSample sample = new CameraCalibratorSample();
        
        //check default value
        assertNull(sample.getCamera());
        
        //set new valeu
        PinholeCamera camera = new PinholeCamera();
        sample.setCamera(camera);
        
        //check correctness
        assertSame(sample.getCamera(), camera);
    }
    
    @Test
    public void testEstimateHomographyCirclesPattern() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            CoincidentPointsException{
        
        int totalPoints = 0;
        double avgTotalError = 0.0;
        for(int j = 0; j < TIMES; j++){
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<Point3D>();
            for(Point2D patternPoint : patternPoints){
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                    patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create random camera to project 3D points
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);

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


            //create sample with projected pattern markers
            CameraCalibratorSample sample = new CameraCalibratorSample(
                    projectedPatternPoints, CameraCalibratorSample.
                    computeSampledMarkersQualityScores(projectedPatternPoints));


            //estimate homography using ideal markers as reference
            PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create();
            Transformation2D homography = sample.estimateHomography(estimator, 
                    patternPoints);

            //check that points are properly transformed
            double distance;
            for(int i = 0; i < patternPoints.size(); i++){
                distance = projectedPatternPoints.get(i).distanceTo(
                        homography.transformAndReturnNew(patternPoints.get(i)));
                avgTotalError += distance;
                totalPoints++;
            }            
        }
        
        avgTotalError /= totalPoints;
        assertEquals(avgTotalError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
    }
    
    @Test
    public void testEstimateHomographyQRPattern() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            CoincidentPointsException{
        
        int totalPoints = 0;
        double avgTotalError = 0.0;        
        for(int j = 0; j < 2*TIMES; j++){
            Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<Point3D>();
            for(Point2D patternPoint : patternPoints){
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                    patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create random camera to project 3D points
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);

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


            //create sample with projected pattern markers
            CameraCalibratorSample sample = new CameraCalibratorSample(
                    projectedPatternPoints, CameraCalibratorSample.
                    computeSampledMarkersQualityScores(projectedPatternPoints));


            //estimate homography using ideal markers as reference
            PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create();          
            Transformation2D homography;
            try {
                homography = sample.estimateHomography(estimator, patternPoints);
            } catch (CoincidentPointsException e) {
                continue;
            }

            //check that points are properly transformed
            double distance;
            for(int i = 0; i < patternPoints.size(); i++){
                distance = projectedPatternPoints.get(i).distanceTo(
                        homography.transformAndReturnNew(patternPoints.get(i)));
                avgTotalError += distance;
                totalPoints++;                
            }            
        }
        
        avgTotalError /= totalPoints;
        assertEquals(avgTotalError, 0.0, LARGE_ABSOLUTE_ERROR);
    }    
    
    @Test
    public void testComputeCameraPose() throws LockedException, 
            NotReadyException, RobustEstimatorException, 
            CoincidentPointsException, CalibrationException, RotationException, 
            NotAvailableException, WrongSizeException{
        
        int totalPoints = 0;        
        double avgProjectionError = 0.0;
        double avgCenterError = 0.0;
        int numValid = 0;
        for(int j = 0; j < TIMES; j++){
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();
            
            //assume that pattern points are located on a 3D plane
            //(for instance Z = 0)
            List<Point3D> points3D = new ArrayList<Point3D>();
            for(Point2D patternPoint : patternPoints){
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                patternPoint.getInhomY(), 0.0, 1.0));
            }
            
            //create random camera to project 3D points
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;
            
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
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
            
            //create sample with projected pattern markers
            CameraCalibratorSample sample = new CameraCalibratorSample(
                    projectedPatternPoints, CameraCalibratorSample.
                    computeSampledMarkersQualityScores(projectedPatternPoints));


            //estimate homography using ideal markers as reference
            PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                    create();
            Transformation2D homography = sample.estimateHomography(estimator, 
                    patternPoints);

            //set homography
            sample.setHomography(homography);
            
            //initially rotation, camera center and camera are null
            assertNull(sample.getRotation());
            assertNull(sample.getCameraCenter());
            assertNull(sample.getCamera());
            
            //compute camera pose
            sample.computeCameraPose(intrinsic);
            
            //check correctness
            
            //compare rotation
            Matrix sRotMat = sample.getRotation().asInhomogeneousMatrix();
            Matrix rotMat = rotation.asInhomogeneousMatrix();
            Matrix rotDiff = new Matrix(3, 3);
            for(int r = 0; r < 3; r++){
                for(int c = 0; c < 3; c++){
                    //signs of columns 1,2 and column 3 of rotation might
                    //be reversed and rotation would still be equal
                    rotDiff.setElementAt(r, c, 
                            Math.abs(sRotMat.getElementAt(r, c)) -
                            Math.abs(rotMat.getElementAt(r, c)));
                }
            }
            assertEquals(Utils.normF(rotDiff), 0.0, LARGE_ABSOLUTE_ERROR);
            
            //compare center
            avgCenterError += sample.getCameraCenter().distanceTo(cameraCenter);

            //compare camera parameters
            assertSame(sample.getCamera().getIntrinsicParameters(), intrinsic);
            assertSame(sample.getCamera().getCameraRotation(), 
                    sample.getRotation());
            assertEquals(sample.getCamera().getCameraCenter().distanceTo(
                    sample.getCameraCenter()), 0.0, ABSOLUTE_ERROR);
            
            //project ideal pattern points using estimated camera and
            //compare against sampled points
            List<Point2D> projectedPatternPoints2 = sample.getCamera().project(
                    points3D);
            double distance;
            for(int i = 0; i < patternPoints.size(); i++){
                distance = projectedPatternPoints.get(i).distanceTo(
                        projectedPatternPoints2.get(i));
                avgProjectionError += distance;
                totalPoints++;
            }
        }
        
        avgProjectionError /= totalPoints;
        avgCenterError /= TIMES;
        assertEquals(avgProjectionError, 0.0, ULTRA_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgCenterError, 0.0, ULTRA_LARGE_ABSOLUTE_ERROR);
    }
}
