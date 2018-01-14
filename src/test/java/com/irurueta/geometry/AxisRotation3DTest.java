/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.AxisRotation3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 11, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.LockedException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class AxisRotation3DTest {
    public static final int ROTATION_COLS = 3;    
    public static final int INHOM_COORDS = 3;
    public static final int HOM_COORDS = 4;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double MIN_RANDOM_SCALE = 2.0;
    public static final double MAX_RANDOM_SCALE = 3.0;
    
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;
    
    public static final double ABSOLUTE_ERROR = 5e-6;    
    public static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    public static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-4;
    
    public static final int TIMES = 10;
    
    public AxisRotation3DTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    @Test
    public void testConstructor() throws RotationException, WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException{
        //Test empty constructor
        AxisRotation3D rotation = new AxisRotation3D();
        assertNotNull(rotation);
        
        assertEquals(rotation.getAxisX(), 0.0, 0.0);
        assertEquals(rotation.getAxisY(), 0.0, 0.0);
        assertEquals(rotation.getAxisZ(), 1.0, 0.0);
        assertEquals(rotation.getRotationAngle(), 0.0, 0.0);
        
        //check that empty constructor creates a rotation without effect (its
        //matrix representation is equal to the identity
        assertTrue(rotation.asInhomogeneousMatrix().equals( 
                Matrix.identity(INHOM_COORDS, INHOM_COORDS), ABSOLUTE_ERROR));        
        
        
        //test constructor using axis and angle
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;        
        
            
        //Force WrongSizeException
        double[] axis = new double[INHOM_COORDS + 1];
            
        rotation = null;
        try{
            rotation = new AxisRotation3D(axis, theta);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(rotation);
            
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] vAxis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);
        Matrix mAxis = Matrix.newFromArray(vAxis, true);
            
        //inhomogeneous coordinates of point laying on rotation plane
        double [] vPoint = vMatrix.getSubmatrixAsArray(0, 1, 2, 1);
            
        rotation = new AxisRotation3D(vAxis, theta);
            
        //To test correctness of rotation, axis should remain equal
        Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
            
        Matrix mAxis2 = rotationMatrix.multiplyAndReturnNew(mAxis);
            
        assertEquals(mAxis.getElementAtIndex(0), 
                mAxis2.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(mAxis.getElementAtIndex(1), 
                mAxis2.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(mAxis.getElementAtIndex(2), 
                mAxis2.getElementAtIndex(2), ABSOLUTE_ERROR);
            
        axis = rotation.getRotationAxis();
            
        double scaleX = axis[0] / vAxis[0];
        double scaleY = axis[0] / vAxis[0];
        double scaleZ = axis[0] / vAxis[0];
            
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);        
        
        //and point on rotated plance should be rotated exactly theta 
        //radians
        Matrix mPoint = Matrix.newFromArray(vPoint);
        //below: rotated point
        Matrix mPoint2 = rotationMatrix.multiplyAndReturnNew(mPoint);
            
        //because vPoint is already normalized (from SVD decomposition)
        //we only need to normalize rotated point to compute the rotation
        //angle as the arcosine of their dot product
        double norm2 = Utils.normF(mPoint2);
        mPoint2.multiplyByScalar(1.0 / norm2);
            
        double dotProduct = mPoint.transposeAndReturnNew().
                multiplyAndReturnNew(mPoint2).getElementAtIndex(0);
            
        double theta2 = Math.acos(dotProduct);
        double theta2b = rotation.getRotationAngle();
            
        //check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);
            
        //check correctness of angles (including sign) for method in class
        if(scaleX > 0.0){
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        }else{
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }    
        
        //Test copy constructor
        //    rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);        
        AxisRotation3D rotation2 = new AxisRotation3D(
                rotation);
        assertEquals(rotation2.asInhomogeneousMatrix(), 
                rotation.asInhomogeneousMatrix());        
    }
    
    @Test
    public void testGetSetAxisAndRotation() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException, RotationException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        AxisRotation3D rotation = new AxisRotation3D();
        
        //Force IllegalArgumentException
        double[] axis = new double[INHOM_COORDS + 1];
        
        try{
            rotation.setAxisAndRotation(axis, theta);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        //remaining two will lie on the rotation plane and will be used to test
        //theta angle
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix v = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
        axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);
        
        //To test correctness of rotation, axis should remain equal
        Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        
        Matrix axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);
        
        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));
        
        double[] axis2 = rotation.getRotationAxis();
        
        double scaleX = axis[0] / axis2[0];
        double scaleY = axis[0] / axis2[0];
        double scaleZ = axis[0] / axis2[0];
            
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);
            
        //and point on rotated plance should be rotated exactly theta 
        //radians
        Matrix pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);
            
        //because vPoint is already normalized (from SVD decomposition)
        //we only need to normalize rotated point to compute the rotation
        //angle as the arcosine of their dot product
        double norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);
            
        double dotProduct = pointMatrix.transposeAndReturnNew().
                multiplyAndReturnNew(pointMatrix2).getElementAtIndex(0);
            
        double theta2 = Math.acos(dotProduct);
        double theta2b = rotation.getRotationAngle();
            
        //check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);
            
        //check correctness of angles (including sign) for method in class
        if(scaleX > 0.0){
            assertEquals(theta, theta2b, ABSOLUTE_ERROR);
        }else{
            assertEquals(theta, -theta2b, ABSOLUTE_ERROR);
        }                
    }    
    
    @Test
    public void testIsValidRotationMatrix() 
            throws WrongSizeException, NotReadyException, LockedException, 
            DecomposerException, NotAvailableException{
        
        Matrix a = Matrix.createWithUniformRandomValues(INHOM_COORDS, 
                INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //Use SVD to obtain an orthonormal matrix from V matrix
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix rotationMatrix = decomposer.getV();
        
        //rotation matrix shouldn't be valid for tiny thresholds
        assertFalse(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix, 0.0));
        
        //for reasonable threhsolds, matrix should be valid because it is
        //orthonormal with some imprecisions due to machine precision
        assertTrue(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix, ABSOLUTE_ERROR));
        assertTrue(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix));
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_SCALE, 
                MAX_RANDOM_SCALE);
        
        //change scale of matrix to make it orthogonal instead of orthonormal
        rotationMatrix.multiplyByScalar(scale);
        
        //now matrix shouldn't be valid even for reasonable thresholds
        assertFalse(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix, ABSOLUTE_ERROR));
        assertFalse(AxisRotation3D.isValidRotationMatrix(
                rotationMatrix));
    }    
    
    @Test
    public void testInverseRotation() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException, InvalidRotationMatrixException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix v = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
        double[] axis = axisMatrix.toArray();

        AxisRotation3D rotation = new AxisRotation3D(axis, 
                theta);
        
        Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        
        //compute inverse rotation
        AxisRotation3D invRotation = 
                rotation.inverseRotationAndReturnNew();
        AxisRotation3D invRotation2 = new AxisRotation3D();
        rotation.inverseRotation(invRotation2);
        
        //check correctness
        Matrix invRotMatrix = invRotation.asInhomogeneousMatrix();
        Matrix invRotMatrix2 = invRotation2.asInhomogeneousMatrix();
        
        Matrix identity = Matrix.identity(INHOM_COORDS, INHOM_COORDS);
        
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix).equals(
                identity, ABSOLUTE_ERROR));
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix2).equals(
                identity, ABSOLUTE_ERROR));
        
        //we can also inverse the original rotation
        rotation.inverseRotation();
        assertTrue(invRotMatrix.equals(rotation.asInhomogeneousMatrix(), 
                ABSOLUTE_ERROR));        
    } 
    
    @Test
    public void testAsInhomogeneousMatrix() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException, InvalidRotationMatrixException{

        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
            //To find 3 orthogonal vectors, we use V matrix of a singular value
            //decomposition of any Nx3 matrix
            Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
            decomposer.decompose();
        
            Matrix v = decomposer.getV();
        
            //axis of rotation
            Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
            //inhomogeneous coordinates of point laying on rotation plane
            Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
            double[] axis = axisMatrix.toArray();

            AxisRotation3D rotation = new AxisRotation3D(
                    axis, theta);
            MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
            
                        
            Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
            Matrix rotationMatrix2 = rotation2.getInternalMatrix();
            
            assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
            
            rotationMatrix2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
            rotation.asInhomogeneousMatrix(rotationMatrix2);
            assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
        
            //Force IllegalArgumentException
            try{
                rotation.asInhomogeneousMatrix(new Matrix(HOM_COORDS, 
                        HOM_COORDS));
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
        }
    }
    
    @Test
    public void testAsHomogeneousMatrix() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException, InvalidRotationMatrixException{
        
        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
            //To find 3 orthogonal vectors, we use V matrix of a singular value
            //decomposition of any Nx3 matrix
            Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
            decomposer.decompose();
        
            Matrix v = decomposer.getV();
        
            //axis of rotation
            Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
            //inhomogeneous coordinates of point laying on rotation plane
            Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
            double[] axis = axisMatrix.toArray();

            AxisRotation3D rotation = new AxisRotation3D(
                    axis, theta);
            MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);

            
            Matrix rotationMatrix = rotation.asHomogeneousMatrix();
            Matrix rotationMatrix2 = rotation2.asHomogeneousMatrix();
            
            assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
        
            
            Matrix homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //set top-left 3x3 submatrix
            homRotationMatrix.setSubmatrix(0, 0, 2, 2, 
                    rotation.asInhomogeneousMatrix());
            
            
            Matrix rotationMatrix3 = new Matrix(HOM_COORDS, HOM_COORDS);
            rotation.asHomogeneousMatrix(rotationMatrix3);
            assertTrue(homRotationMatrix.equals(rotationMatrix3, 
                    ABSOLUTE_ERROR));
        
            //Force IllegalArgumentException
            try{
                rotation.asHomogeneousMatrix(new Matrix(INHOM_COORDS, 
                        INHOM_COORDS));
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
        }
    } 
    
    @Test
    public void testFromMatrix() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, NotAvailableException, 
        InvalidRotationMatrixException{

        for(int t = 0; t < TIMES; t++){
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());       
            double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
            //To find 3 orthogonal vectors, we use V matrix of a singular value
            //decomposition of any Nx3 matrix
            Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                    0.0, MAX_RANDOM_VALUE);
            a.setElementAt(0, 2, MAX_RANDOM_VALUE);
            //normalize matrix
            double norm = Utils.normF(a);
            a.multiplyByScalar(1.0 / norm);
            double[] axis = a.toArray();
            
            double[] coords = new double[INHOM_COORDS];
            InhomogeneousPoint3D inputPoint = new InhomogeneousPoint3D(coords);

            MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
            Matrix rotationMatrix = rotation2.getInternalMatrix();
            
            Point3D outputPoint2 = rotation2.rotate(inputPoint);
       
            Matrix homRotationMatrix = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //set top-left 3x3 submatrix
            homRotationMatrix.setSubmatrix(0, 0, 2, 2, rotationMatrix);
        
        
            AxisRotation3D rotation = new AxisRotation3D();
            //test with inhomogeneous matrix
            rotation.fromMatrix(rotationMatrix, LARGE_ABSOLUTE_ERROR);
                        
            Point3D outputPoint = rotation.rotate(inputPoint);
            
            assertTrue(outputPoint.equals(outputPoint2, ABSOLUTE_ERROR));
                
        
            //test with homogeneous matrix
            rotation.fromMatrix(homRotationMatrix);
            
            outputPoint = rotation.rotate(inputPoint);
            
            assertTrue(outputPoint.equals(outputPoint2, ABSOLUTE_ERROR));            
            
            
            //Attempt to force InvalidRotationMatrixException (using a tiny 
            //threshold). This exception might not always be thrown even for
            //the smallest threshold
            try{
                rotation.fromMatrix(rotationMatrix, 0.0);
            }catch(InvalidRotationMatrixException e){}
            
            //Force IllegalArugmentException
            try{
                rotation.fromMatrix(rotationMatrix, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
        
            //or using a non orthonormal matrix
            homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
            try{
                rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR);
                fail("InvalidRotationMatrixException expected but not thrown");
            }catch(InvalidRotationMatrixException e){}        
        }
    }
    
    @Test
    public void testFromInhomogeneousMatrix() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException, InvalidRotationMatrixException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix v = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
        double[] axis = axisMatrix.toArray();
        MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
        Matrix rotationMatrix = rotation2.getInternalMatrix();
               
        
        AxisRotation3D rotation = new AxisRotation3D();
        //test with inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(rotationMatrix);
        //Check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), 
                LARGE_ABSOLUTE_ERROR));
        
        rotation.fromMatrix(rotationMatrix, ABSOLUTE_ERROR);
        //Check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), 
                ABSOLUTE_ERROR));

        int numValid = 0;
        for (int t = 0; t < 10*TIMES; t++) {
            //Force InvalidRotationMatrixException (using a tiny threshold)
            try {
                rotation.fromInhomogeneousMatrix(rotationMatrix, 0.0);
                continue;
            } catch (InvalidRotationMatrixException e) { }

            //or using a non orthonormal matrix
            //because matrix is orthonormal, it's enough to scale it to some value
            double scale = randomizer.nextDouble(MIN_RANDOM_SCALE,
                    MAX_RANDOM_SCALE);
            rotationMatrix.multiplyByScalar(scale);
            try {
                rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR);
                continue;
            } catch (InvalidRotationMatrixException e) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }  
    
    @Test
    public void testFromHomogeneousMatrix() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException, InvalidRotationMatrixException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix v = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
        double[] axis = axisMatrix.toArray();
        MatrixRotation3D rotation2 = new MatrixRotation3D(axis, theta);
        Matrix rotationMatrix = rotation2.getInternalMatrix();
        
        Matrix homRotationMatrix = Matrix.identity(HOM_COORDS, 
                HOM_COORDS);
        //set top-left 3x3 submatrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2, rotationMatrix);
        
        
        AxisRotation3D rotation = new AxisRotation3D();
        //test with homogeneous matrix
        rotation.fromHomogeneousMatrix(homRotationMatrix);
        //check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(), 
                ABSOLUTE_ERROR));
        assertTrue(homRotationMatrix.equals(rotation.asHomogeneousMatrix(),
                ABSOLUTE_ERROR));

        rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
        //check correctness
        assertTrue(rotationMatrix.equals(rotation.asInhomogeneousMatrix(),
                ABSOLUTE_ERROR));
        assertTrue(homRotationMatrix.equals(rotation.asHomogeneousMatrix(),
                ABSOLUTE_ERROR));
        
        
        //Force InvalidRotationMatrixException (using a tiny threshold)
        try{
            rotation.fromHomogeneousMatrix(rotationMatrix, 0.0);
            fail("InvalidRotationMatrixException expected but not thrown");
        }catch(InvalidRotationMatrixException e){}
        
        //or using a non orthonormal matrix
        homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
        try{
            rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        }catch(InvalidRotationMatrixException e){}
    }        
    
    @Test
    public void testRotate() throws WrongSizeException, ColinearPointsException, 
        NotReadyException, LockedException, DecomposerException, 
        NotAvailableException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        
        decomposer.decompose();
        
        Matrix v = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix = v.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix = v.getSubmatrix(0, 1, 2, 1);
        
        double[] axis = axisMatrix.toArray();
        
        AxisRotation3D rotation = new AxisRotation3D(axis, 
                theta);
        
        //create 3 random points
        HomogeneousPoint3D point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point1.normalize(); //to increase accuracy
        
        HomogeneousPoint3D point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.normalize(); //to increase accuracy
        
        HomogeneousPoint3D point3 = new HomogeneousPoint3D();
        point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3.normalize(); //to increase accuracy
        
        //ensure that points are not colinear
        
        while(Plane.areColinearPoints(point1, point2, point3)){
            point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            point2.normalize(); //to increase accuracy
            point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));        
            point3.normalize(); //to increase accuracy
        }
        
        //create plane passing through all three points
        Plane plane = new Plane(point1, point2, point3);
        plane.normalize(); //to increase accuracy
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));
        
        //now rotate points and plane
        Point3D rotPoint1A = rotation.rotate(point1);
        Point3D rotPoint1B = Point3D.create();
        rotation.rotate(point1, rotPoint1B);        
        
        Point3D rotPoint2A = rotation.rotate(point2);
        Point3D rotPoint2B = Point3D.create();
        rotation.rotate(point2, rotPoint2B);
        
        Point3D rotPoint3A = rotation.rotate(point3);
        Point3D rotPoint3B = Point3D.create();
        rotation.rotate(point3, rotPoint3B);
        
        
        //check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPoint3A.equals(rotPoint3B, ABSOLUTE_ERROR));
        
        //check points have been correctly rotated   
        Matrix point1Mat = Matrix.newFromArray(point1.asArray(), true);
        Matrix point2Mat = Matrix.newFromArray(point2.asArray(), true);
        Matrix point3Mat = Matrix.newFromArray(point3.asArray(), true);
        Matrix R = rotation.asHomogeneousMatrix();
        Matrix rotPoint1Mat = R.multiplyAndReturnNew(point1Mat);
        Matrix rotPoint2Mat = R.multiplyAndReturnNew(point2Mat);
        Matrix rotPoint3Mat = R.multiplyAndReturnNew(point3Mat);
        
        //check correctness
        double scaleX = rotPoint1A.getHomX() / 
                rotPoint1Mat.getElementAtIndex(0);
        double scaleY = rotPoint1A.getHomY() / 
                rotPoint1Mat.getElementAtIndex(1);
        double scaleZ = rotPoint1A.getHomZ() / 
                rotPoint1Mat.getElementAtIndex(2);
        double scaleW = rotPoint1A.getHomW() / 
                rotPoint1Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint2A.getHomX() / rotPoint2Mat.getElementAtIndex(0);
        scaleY = rotPoint2A.getHomY() / rotPoint2Mat.getElementAtIndex(1);
        scaleZ = rotPoint2A.getHomZ() / rotPoint2Mat.getElementAtIndex(2);
        scaleW = rotPoint2A.getHomW() / rotPoint2Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
        
        //ensure that points where correctly rotated using inhomogeneous 
        //coordinates
        Matrix inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        Matrix inhomR = rotation.asInhomogeneousMatrix();
        Matrix inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        Point3D rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));
        
        
        
        Plane rotPlaneA = rotation.rotate(plane);
        Plane rotPlaneB = new Plane();
        rotation.rotate(plane, rotPlaneB);
        
        //check both rotated lines are equal
        double scaleA = rotPlaneA.getA() / rotPlaneB.getA();
        double scaleB = rotPlaneA.getB() / rotPlaneB.getB();
        double scaleC = rotPlaneA.getC() / rotPlaneB.getC();
        double scaleD = rotPlaneA.getD() / rotPlaneB.getD();
        
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
        
        //check plane has been correctly rotated by ensuring that rotated points
        //belong into rotated line
        assertTrue(rotPlaneA.isLocus(rotPoint1A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3B, ABSOLUTE_ERROR));        
        
        //and by ensuring that rotated plane follow appropriate equation
        Matrix planeMat = Matrix.newFromArray(plane.asArray(), true);
        Matrix rotPlaneMat = R.multiplyAndReturnNew(planeMat);
        
        scaleA = rotPlaneA.getA() / rotPlaneMat.getElementAtIndex(0);
        scaleB = rotPlaneA.getB() / rotPlaneMat.getElementAtIndex(1);
        scaleC = rotPlaneA.getC() / rotPlaneMat.getElementAtIndex(2);
        scaleD = rotPlaneA.getD() / rotPlaneMat.getElementAtIndex(3);
        
        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);        
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);                
    }    
    
    @Test
    public void testCombine() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, NotAvailableException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
        //To find 3 orthogonal vectors, we use V matrix of a singular value
        //decomposition of any Nx3 matrix
        Matrix a1 = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Matrix a2 = Matrix.createWithUniformRandomValues(1, INHOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a1);        
        decomposer.decompose();        
        Matrix v1 = decomposer.getV();
        
        decomposer.setInputMatrix(a2);
        decomposer.decompose();
        Matrix v2 = decomposer.getV();
        
        //axis of rotation
        Matrix axisMatrix1 = v1.getSubmatrix(0, 0, 2, 0);
        Matrix axisMatrix2 = v2.getSubmatrix(0, 0, 2, 0);
        
        //inhomogeneous coordinates of point laying on rotation plane
        Matrix pointMatrix1 = v1.getSubmatrix(0, 1, 2, 1);
        Matrix pointMatrix2 = v2.getSubmatrix(0, 1, 2, 1);
        
        double[] axis1 = axisMatrix1.toArray();
        double[] axis2 = axisMatrix2.toArray();

                
        AxisRotation3D rot1 = new AxisRotation3D(axis1, 
                theta1);
        AxisRotation3D rot2 = new AxisRotation3D(axis2, 
                theta2);
        
        AxisRotation3D rot = new AxisRotation3D();
        AxisRotation3D.combine(rot1, rot2, rot);
        
        assertTrue(rot.asInhomogeneousMatrix().equals(
                rot1.asInhomogeneousMatrix().multiplyAndReturnNew(
                rot2.asInhomogeneousMatrix()), ABSOLUTE_ERROR));
        
        AxisRotation3D rot3 = rot.combineAndReturnNew(rot2);
        assertTrue(rot3.asInhomogeneousMatrix().equals(
                rot.asInhomogeneousMatrix().multiplyAndReturnNew(
                rot2.asInhomogeneousMatrix()), ABSOLUTE_ERROR));
        
        Matrix rotationMatrix = rot.asInhomogeneousMatrix();
        rot.combine(rot1);
        assertTrue(rot.asInhomogeneousMatrix().equals(rotationMatrix.
                multiplyAndReturnNew(rot1.asInhomogeneousMatrix()), 
                5.0 * LARGE_ABSOLUTE_ERROR));
    }    
    
    @Test
    public void testType(){
        AxisRotation3D rotation = new AxisRotation3D();
        assertEquals(rotation.getType(), Rotation3DType.AXIS_ROTATION3D);
    }
    
    @Test
    public void testFromRotation() throws AlgebraException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);
        
        MatrixRotation3D matrixRotation = new MatrixRotation3D();
        AxisRotation3D axisRotation = new AxisRotation3D();
        Quaternion quaternion = new Quaternion();

        matrixRotation.setAxisAndRotation(axis, theta);
        axisRotation.setAxisAndRotation(axis, theta);
        quaternion.setAxisAndRotation(axis, theta);                
        
        //test from matrix rotation
        AxisRotation3D rotation1 = new AxisRotation3D();
        rotation1.fromRotation(matrixRotation);
        
        //check correctness
        assertEquals(matrixRotation, rotation1);
        
        //test from axis rotation
        AxisRotation3D rotation2 = new AxisRotation3D();
        rotation2.fromRotation(axisRotation);
        
        //check correctness
        assertEquals(axisRotation, rotation2);
        
        //test from quaternion
        AxisRotation3D rotation3 = new AxisRotation3D();
        rotation3.fromRotation(quaternion);
        
        //check correctness
        assertEquals(quaternion, rotation3);
    }
    
    @Test
    public void testToMatrixRotation() throws AlgebraException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);
        
        AxisRotation3D rotation = new AxisRotation3D(axis, theta);
        
        //to matrix rotation
        MatrixRotation3D matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        MatrixRotation3D matrixRotation2 = rotation.toMatrixRotation();
        
        //check correctness
        assertEquals(rotation, matrixRotation1);
        assertEquals(rotation, matrixRotation2);
    }
    
    @Test
    public void testToAxisRotation() throws AlgebraException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);
        
        AxisRotation3D rotation = new AxisRotation3D(axis, theta);
        
        //to axis rotation
        AxisRotation3D axisRotation1 = new AxisRotation3D();
        rotation.toAxisRotation(axisRotation1);
        AxisRotation3D axisRotation2 = rotation.toAxisRotation();
        
        //check correctness
        assertEquals(rotation, axisRotation1);
        assertEquals(rotation, axisRotation2);
    }

    @Test
    public void testToQuaternion() throws AlgebraException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        //Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        //the remaining two will lie on the rotation plane and will be used
        //to test theta angle
            
        //To find 3 orthogonal vectors, we use V matrix of a singular
        //decomposition of any Nx3 matrix
        Matrix a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
        decomposer.decompose();
            
        Matrix vMatrix = decomposer.getV();
            
        //axis of rotation
        double [] axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);
        
        AxisRotation3D rotation = new AxisRotation3D(axis, theta);
                
        //to quaternion
        Quaternion quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        Quaternion quaternion2 = rotation.toQuaternion();
        
        //check correctness
        assertEquals(rotation, quaternion1);
        assertEquals(rotation, quaternion2);
    }    
}
