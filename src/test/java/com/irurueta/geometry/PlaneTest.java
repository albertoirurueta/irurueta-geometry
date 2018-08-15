/*
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.Plane
 * 
 * @author Alberto Irurueta (alberto@irureta.com)
 * @date August 10, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.*;
import static org.junit.Assert.*;

public class PlaneTest {
    
    private static final int HOM_COORDS = 4;
    private static final int INHOM_COORDS = 3;
    
    private static final double PRECISION_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    private static final double MIN_RANDOM_DISTANCE = -100.0;
    private static final double MAX_RANDOM_DISTANCE = 100.0;
    
    private static final double ABSOLUTE_ERROR = 1e-8;
    
    public PlaneTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException, ColinearPointsException, 
        IllegalArgumentException, ParallelVectorsException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane = new Plane();
        assertEquals(plane.getA(), 0.0, 0.0);
        assertEquals(plane.getB(), 0.0, 0.0);
        assertEquals(plane.getC(), 0.0, 0.0);
        assertEquals(plane.getD(), 0.0, 0.0);
        assertFalse(plane.isNormalized());
        
        plane = new Plane(a, b, c, d);
        assertEquals(plane.getA(), a, 0.0);
        assertEquals(plane.getB(), b, 0.0);
        assertEquals(plane.getC(), c, 0.0);
        assertEquals(plane.getD(), d, 0.0);
        assertFalse(plane.isNormalized());
        
        //instantiate plane using array
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        plane = new Plane(array);
        assertEquals(plane.getA(), array[0], 0.0);
        assertEquals(plane.getB(), array[1], 0.0);
        assertEquals(plane.getC(), array[2], 0.0);
        
        //Force IllegalArgumentException
        plane = null;
        try {
            plane = new Plane(new double[HOM_COORDS + 1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(plane);
        
        //find 1st point
        double[] array1 = new double[HOM_COORDS];
        randomizer.fill(array1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //find 2nd point
        double[] array2 = new double[HOM_COORDS];
        randomizer.fill(array2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double lambda = randomizer.nextDouble();
        
        //find 3rd point as a colinear point of 1st and 2nd point
        double[] array3 = new double[HOM_COORDS];
        //array3 = lamda * array2
        ArrayUtils.multiplyByScalar(array2, lambda, array3);
        //array3 = array1 + array3 = array1 + lambda * array2
        ArrayUtils.sum(array1, array3, array3);
        
        Point3D point1 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array1);
        Point3D point2 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array2);
        Point3D point3 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array3);
        
        //Force ColinearPointsException
        assertTrue(Plane.areColinearPoints(point1, point2, point3));
        plane = null;
        try {
            plane = new Plane(point1, point2, point3);
        } catch (ColinearPointsException ignore) { }
        assertNull(plane);
        
        //try with 3 non-colinear points
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        //ensure we create a matrix with 3 non linear dependent rows
        while(decomposer.getRank() < 3){
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        //V contains the right nullspace of m in the last column, which is the 
        //plane where the points belong to in the rows of m
        Matrix V = decomposer.getV();
        
        point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1), 
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));
        
        point1.normalize();
        point2.normalize();
        point3.normalize();
        
        double[] arrayPlane = new double[HOM_COORDS];
        arrayPlane[0] = V.getElementAt(0, 3);
        arrayPlane[1] = V.getElementAt(1, 3);
        arrayPlane[2] = V.getElementAt(2, 3);
        arrayPlane[3] = V.getElementAt(3, 3);
        
        assertFalse(Plane.areColinearPoints(point1, point2, point3));
        plane = new Plane(point1, point2, point3);
        
        //check correctness of obtained plane
        double scaleA = plane.getA() / arrayPlane[0];
        double scaleB = plane.getB() / arrayPlane[1];
        double scaleC = plane.getC() / arrayPlane[2];
        double scaleD = plane.getD() / arrayPlane[3];
        
        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleD, PRECISION_ERROR);
        assertEquals(scaleD, scaleA, PRECISION_ERROR);
        
        //create plane passing through a point and laying on provided direction 
        //vectors
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point3D point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array);
        
        double[] direction1 = new double[INHOM_COORDS];
        randomizer.fill(direction1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double[] direction2 = new double[INHOM_COORDS];
        randomizer.fill(direction2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //normalize to compare with greater accuracy
        double norm1 = com.irurueta.algebra.Utils.normF(direction1);
        double[] direction1b = ArrayUtils.multiplyByScalarAndReturnNew(
                direction1, 1.0 / norm1);
        double norm2 = com.irurueta.algebra.Utils.normF(direction2);
        double[] direction2b = ArrayUtils.multiplyByScalarAndReturnNew(
                direction2, 1.0 / norm2);
        
        //director vector will already be normalized because direction vectors
        //are normalized
        double[] directorVector = com.irurueta.algebra.Utils.crossProduct(
                direction1b, direction2b);

        //create plane using point and directions
        Plane plane1 = new Plane(point, direction1, direction2);
        //and create plane using point and director vector
        Plane plane2 = new Plane(point, directorVector);
        
        //check correctness of instantiated planes
        assertTrue(plane1.isLocus(point));
        assertTrue(plane2.isLocus(point));
        
        double[] directorVector1 = plane1.getDirectorVector();
        assertEquals(directorVector1[0], plane1.getA(), 0.0);
        assertEquals(directorVector1[1], plane1.getB(), 0.0);
        assertEquals(directorVector1[2], plane1.getC(), 0.0);
        double[] directorVector2 = plane2.getDirectorVector();        
        assertEquals(directorVector2[0], plane2.getA(), 0.0);
        assertEquals(directorVector2[1], plane2.getB(), 0.0);
        assertEquals(directorVector2[2], plane2.getC(), 0.0);
        
        //check that both director vectors are equal
        assertEquals(directorVector1[0], directorVector2[0], 
                PRECISION_ERROR);
        assertEquals(directorVector1[1], directorVector2[1], 
                PRECISION_ERROR);
        assertEquals(directorVector1[2], directorVector2[2], 
                PRECISION_ERROR);
        
        //and equal to the estimated director vector
        double scale1 = directorVector1[0] / directorVector[0];
        double scale2 = directorVector1[1] / directorVector[1];
        double scale3 = directorVector1[2] / directorVector[2];
        
        assertEquals(scale1, scale2, PRECISION_ERROR);
        assertEquals(scale2, scale3, PRECISION_ERROR);
        assertEquals(scale3, scale1, PRECISION_ERROR);        

        //both planes are equal
        assertEquals(plane1.getA(), plane2.getA(), PRECISION_ERROR);
        assertEquals(plane1.getB(), plane2.getB(), PRECISION_ERROR);
        assertEquals(plane1.getC(), plane2.getC(), PRECISION_ERROR);
        assertEquals(plane1.getD(), plane2.getD(), PRECISION_ERROR);
        
        
        //Force ParallelVectorsException
        plane1 = null;
        try {
            plane1 = new Plane(point, direction1, direction1);
            fail("ParallelsVectorsException expected but not thrown");
        } catch (ParallelVectorsException ignore) { }
        assertNull(plane1);
        
        //Force IllegalArgumentExeption
        double[] wrongArray = new double[HOM_COORDS];
        plane2 = null;
        try {
            plane1 = new Plane(point, wrongArray, direction2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            plane1 = new Plane(point, direction1, wrongArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            plane2 = new Plane(point, wrongArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(plane1);
        assertNull(plane2);
        
    }
    
    @Test
    public void testGettersAndSetters() {
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane = new Plane();
        
        //set parameters using array
        plane.setParameters(array);
        assertEquals(plane.getA(), array[0], 0.0);
        assertEquals(plane.getB(), array[1], 0.0);
        assertEquals(plane.getC(), array[2], 0.0);
        assertEquals(plane.getD(), array[3], 0.0);
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        plane.setA(a);
        plane.setB(b);
        plane.setC(c);
        plane.setD(d);
        
        assertEquals(plane.getA(), a, 0.0);
        assertEquals(plane.getB(), b, 0.0);
        assertEquals(plane.getC(), c, 0.0);
        assertEquals(plane.getD(), d, 0.0);
        
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        plane.setParameters(a, b, c, d);
        assertEquals(plane.getA(), a, 0.0);
        assertEquals(plane.getB(), b, 0.0);
        assertEquals(plane.getC(), c, 0.0);
        assertEquals(plane.getD(), d, 0.0);                
    }
    
    @Test
    public void testSetParametersFromThreePoints() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException, ColinearPointsException {
        
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        //the right nullspace of m (last column of V) contains the plane where
        //the 3 points belong to
        Matrix V = decomposer.getV();
        
        Point3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        Point3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        Point3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));
        
        point1.normalize();
        point2.normalize();
        point3.normalize();
        
        double[] planeArray = new double[HOM_COORDS];
        planeArray[0] = V.getElementAt(0, 3);
        planeArray[1] = V.getElementAt(1, 3);
        planeArray[2] = V.getElementAt(2, 3);
        planeArray[3] = V.getElementAt(3, 3);
        
        Plane plane = new Plane();
        assertFalse(Plane.areColinearPoints(point1, point2, point3));
        plane.setParametersFromThreePoints(point1, point2, point3);
        
        //compare plane parameters respect to computed plane array parameters
        //They will be equal up to scale
        double scaleA = plane.getA() / planeArray[0];
        double scaleB = plane.getB() / planeArray[1];
        double scaleC = plane.getC() / planeArray[2];
        double scaleD = plane.getD() / planeArray[3];
        
        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleD, PRECISION_ERROR);
        assertEquals(scaleD, scaleA, PRECISION_ERROR);
        
        //all three points belong to the plane
        plane.normalize(); //to increase accuracy
        assertTrue(plane.isLocus(point1, PRECISION_ERROR));
        assertTrue(plane.isLocus(point2, PRECISION_ERROR));
        assertTrue(plane.isLocus(point3, PRECISION_ERROR));
        
        //Force ColinearPointsException
        //find colinear points using a third point being a linear combination
        //of two other points so that a plane cannot be defined
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //find 1st point
        double[] array1 = new double[HOM_COORDS];
        randomizer.fill(array1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //find 2nd point
        double[] array2 = new double[HOM_COORDS];
        randomizer.fill(array2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double lambda = randomizer.nextDouble();
        
        //find 3rd point as a colinear point of 1st and 2nd point
        double[] array3 = new double[HOM_COORDS];
        //array3 = lamda * array2
        ArrayUtils.multiplyByScalar(array2, lambda, array3);
        //array3 = array1 + array3 = array1 + lambda * array2
        ArrayUtils.sum(array1, array3, array3);
        
        point1 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array1);
        point2 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array2);
        point3 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array3);
        
        //Force ColinearPointsException
        plane = null;
        assertTrue(Plane.areColinearPoints(point1, point2, point3));
        try {
            plane = new Plane(point1, point2, point3);
        } catch (ColinearPointsException ignore) { }
        assertNull(plane);
    }
    
    @Test
    public void testSetParametersFrom1PointAnd2Vectors() 
            throws WrongSizeException, IllegalArgumentException, 
            ParallelVectorsException {
        //create plane passing through a point and laying on provided direction 
        //vectors
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point3D point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array);
        
        double[] direction1 = new double[INHOM_COORDS];
        randomizer.fill(direction1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double[] direction2 = new double[INHOM_COORDS];
        randomizer.fill(direction2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //normalize to compare with greater accuracy
        double norm1 = com.irurueta.algebra.Utils.normF(direction1);
        double[] direction1b = ArrayUtils.multiplyByScalarAndReturnNew(
                direction1, 1.0 / norm1);
        double norm2 = com.irurueta.algebra.Utils.normF(direction2);
        double[] direction2b = ArrayUtils.multiplyByScalarAndReturnNew(
                direction2, 1.0 / norm2);
        
        //director vector will already be normalized because direction vectors
        //are normalized
        double[] directorVector = com.irurueta.algebra.Utils.crossProduct(
                direction1b, direction2b);

        Plane plane = new Plane();
        
        //set parameters using point and directions
        plane.setParametersFrom1PointAnd2Vectors(point, direction1, direction2);
        
        //check correctness of plane
        assertTrue(plane.isLocus(point));
        
        double[] directorVectorB = plane.getDirectorVector();
        assertEquals(directorVectorB[0], plane.getA(), 0.0);
        assertEquals(directorVectorB[1], plane.getB(), 0.0);
        assertEquals(directorVectorB[2], plane.getC(), 0.0);
                
        //and equal to the estimated director vector
        double scale1 = directorVectorB[0] / directorVector[0];
        double scale2 = directorVectorB[1] / directorVector[1];
        double scale3 = directorVectorB[2] / directorVector[2];
        
        assertEquals(scale1, scale2, PRECISION_ERROR);
        assertEquals(scale2, scale3, PRECISION_ERROR);
        assertEquals(scale3, scale1, PRECISION_ERROR);        
        
        
        //Force ParallelVectorsException
        try {
            plane.setParametersFrom1PointAnd2Vectors(point, direction1, 
                    direction1);
            fail("ParallelsVectorsException expected but not thrown");
        } catch (ParallelVectorsException ignore) { }
        
        //Force IllegalArgumentExeption
        double[] wrongArray = new double[HOM_COORDS];
        try {
            plane.setParametersFrom1PointAnd2Vectors(point, wrongArray, 
                    direction2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            plane.setParametersFrom1PointAnd2Vectors(point, direction1, 
                    wrongArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testSetParametersFrom1PointAndDirectorVector() 
            throws WrongSizeException {
        //create plane passing through a point and laying on provided direction 
        //vectors
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Point3D point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, 
                array);
        
        double[] direction1 = new double[INHOM_COORDS];
        randomizer.fill(direction1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double[] direction2 = new double[INHOM_COORDS];
        randomizer.fill(direction2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //normalize to compare with greater accuracy
        double norm1 = com.irurueta.algebra.Utils.normF(direction1);
        double[] direction1b = ArrayUtils.multiplyByScalarAndReturnNew(
                direction1, 1.0 / norm1);
        double norm2 = com.irurueta.algebra.Utils.normF(direction2);
        double[] direction2b = ArrayUtils.multiplyByScalarAndReturnNew(
                direction2, 1.0 / norm2);
        
        //director vector will already be normalized because direction vectors
        //are normalized
        double[] directorVector = com.irurueta.algebra.Utils.crossProduct(
                direction1b, direction2b);

        Plane plane = new Plane();
        
        //create plane using point and directions
        //and create plane using point and director vector
        plane.setParametersFromPointAndDirectorVector(point, directorVector);
        
        //check correctness of plane
        assertTrue(plane.isLocus(point));
        
        double[] directorVectorB = plane.getDirectorVector();
        assertEquals(directorVectorB[0], plane.getA(), 0.0);
        assertEquals(directorVectorB[1], plane.getB(), 0.0);
        assertEquals(directorVectorB[2], plane.getC(), 0.0);
                
        //and equal to the estimated director vector
        double scale1 = directorVectorB[0] / directorVector[0];
        double scale2 = directorVectorB[1] / directorVector[1];
        double scale3 = directorVectorB[2] / directorVector[2];
        
        assertEquals(scale1, scale2, PRECISION_ERROR);
        assertEquals(scale2, scale3, PRECISION_ERROR);
        assertEquals(scale3, scale1, PRECISION_ERROR);        
                
        //Force IllegalArgumentExeption
        double[] wrongArray = new double[HOM_COORDS];
        try {
            plane.setParametersFromPointAndDirectorVector(point, wrongArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testIsLocus() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException {
        
        //randomly choose 3 points to find their corresponding plane
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while(decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Matrix V = decomposer.getV();
        
        Plane plane = new Plane(V.getElementAt(0, 3),
                V.getElementAt(1, 3),
                V.getElementAt(2, 3),
                V.getElementAt(3, 3));
        
        Point3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        
        Point3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        
        Point3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));
        
        point1.normalize();
        point2.normalize();
        point3.normalize();
        
        //points belong to plane
        assertTrue(plane.isLocus(point1, PRECISION_ERROR));
        assertTrue(plane.isLocus(point2, PRECISION_ERROR));
        assertTrue(plane.isLocus(point3, PRECISION_ERROR));
        
        //use plane director vector to find another point outside of plane
        HomogeneousPoint3D point4 = new HomogeneousPoint3D(point1);
        
        double normDirectorVector = Math.sqrt(plane.getA() * plane.getA() +
                plane.getB() * plane.getB() + 
                plane.getC() * plane.getC());        
        point4.setInhomogeneousCoordinates(point4.getInhomX() +
                plane.getA(), point4.getInhomY() + plane.getB(),
                point4.getInhomZ() + plane.getC());
        
        assertFalse(plane.isLocus(point4, PRECISION_ERROR));
        
        //indeed point4 is at normDirectorVector distanfe from plane
        assertEquals(plane.signedDistance(point4), normDirectorVector, 
                PRECISION_ERROR);
    }
    
    @Test
    public void testSignedDistance() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException {
        
        //randomly choose 3 points to find their corresponding plane
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        while(decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Matrix V = decomposer.getV();
        
        Plane plane = new Plane(V.getElementAt(0, 3),
                V.getElementAt(1, 3),
                V.getElementAt(2, 3),
                V.getElementAt(3, 3));
        
        Point3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        
        Point3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        
        Point3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));
        
        point1.normalize();
        point2.normalize();
        point3.normalize();
        
        //points belong to plance, hence their distance is zero up to machine 
        //precision
        assertEquals(plane.signedDistance(point1), 0.0, PRECISION_ERROR);
        assertEquals(plane.signedDistance(point2), 0.0, PRECISION_ERROR);
        assertEquals(plane.signedDistance(point3), 0.0, PRECISION_ERROR);
        
        //use plane director vector to find another point at desired signed
        //distance
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE,
                MAX_RANDOM_DISTANCE);
        
        HomogeneousPoint3D point4 = new HomogeneousPoint3D(point1);
        
        double normDirectorVector = com.irurueta.algebra.Utils.normF(
                plane.getDirectorVector());
        point4.setInhomogeneousCoordinates(point4.getInhomX() +
                signedDistance * plane.getA() / normDirectorVector, 
                point4.getInhomY() + signedDistance * plane.getB() / 
                normDirectorVector, point4.getInhomZ() + 
                signedDistance * plane.getC() / normDirectorVector);
        
        assertEquals(plane.signedDistance(point4), signedDistance, 
                PRECISION_ERROR);
    }
    
    @Test
    public void testAsArray() {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane = new Plane(a, b, c, d);
        double[] array = plane.asArray();
        
        assertEquals(array[0], a, 0.0);
        assertEquals(array[1], b, 0.0);
        assertEquals(array[2], c, 0.0);
        assertEquals(array[3], d, 0.0);
        
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        plane.setParameters(a, b, c, d);
        plane.asArray(array);
        
        assertEquals(array[0], a, 0.0);
        assertEquals(array[1], b, 0.0);
        assertEquals(array[2], c, 0.0);
        assertEquals(array[3], d, 0.0);

        //Force IllegalArgumentException
        try {
            plane.asArray(new double[HOM_COORDS + 1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testNormalize() {
        
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane = new Plane(array);
        assertFalse(plane.isNormalized());
        
        //normalize plane
        plane.normalize();
        assertTrue(plane.isNormalized());
        
        //return plane as array
        double[] array2 = plane.asArray();
        
        //compare that both arrays are equal up to scale
        //check correctess of obtained plane
        double scaleA = array[0] / array2[0];
        double scaleB = array[1] / array2[1];
        double scaleC = array[2] / array2[2];
        double scaleD = array[3] / array2[3];
        
        assertEquals(scaleA, scaleB, PRECISION_ERROR);
        assertEquals(scaleB, scaleC, PRECISION_ERROR);
        assertEquals(scaleC, scaleD, PRECISION_ERROR);
        assertEquals(scaleD, scaleA, PRECISION_ERROR);
        
        //if we provide zero values, then normalization has no effect
        plane = new Plane(0.0, 0.0, 0.0, 0.0);
        
        assertFalse(plane.isNormalized());
        
        plane.normalize();
        
        assertFalse(plane.isNormalized());
    }
    
    @Test
    public void testDirectorVector() {
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane = new Plane(array);
        
        double[] n = plane.getDirectorVector();
        
        assertEquals(n.length, 3);
        assertEquals(n[0], array[0], 0.0);
        assertEquals(n[1], array[1], 0.0);
        assertEquals(n[2], array[2], 0.0);
        
        //try again
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        plane.setParameters(array);
        
        plane.directorVector(n);
        
        assertEquals(n.length, 3);
        assertEquals(n[0], array[0], 0.0);
        assertEquals(n[1], array[1], 0.0);
        assertEquals(n[2], array[2], 0.0);
    }
    
    @Test
    public void testIntersection() throws WrongSizeException, NotReadyException, 
        com.irurueta.algebra.NotAvailableException, LockedException, 
        DecomposerException, NoIntersectionException, ColinearPointsException {
        
        //Create random homogeneous coordinates for a point
        Matrix m = Matrix.createWithUniformRandomValues(1, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //M is a 1x4 matrix having rank 1, hence its right null-space will have
        //dimension 3. Each vector of the right null-space will follow equation:
        //m * P = 0, hence each of those vectors will be a plane where the point
        //will be locus, and hence the point will be the intersection of those
        //3 planes, which will be perpendicular among them
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        Matrix V = decomposer.getV();
        
        HomogeneousPoint3D point = new HomogeneousPoint3D(m.toArray());
        
        Plane plane1 = new Plane(V.getSubmatrixAsArray(0, 1, 3, 1));
        Plane plane2 = new Plane(V.getSubmatrixAsArray(0, 2, 3, 2));
        Plane plane3 = new Plane(V.getSubmatrixAsArray(0, 3, 3, 3));
        
        assertTrue(plane1.getIntersection(plane2, plane3).equals(point, 
                ABSOLUTE_ERROR));
        
        Point3D intersection = Point3D.create();
        plane1.intersection(plane2, plane3, intersection);
        assertTrue(intersection.equals(point, ABSOLUTE_ERROR));
        
        assertTrue(plane2.getIntersection(plane1, plane3).equals(point,
                ABSOLUTE_ERROR));
        
        plane2.intersection(plane1, plane3, intersection);
        assertTrue(intersection.equals(point, ABSOLUTE_ERROR));
        
        assertTrue(plane3.getIntersection(plane1, plane2).equals(point,
                ABSOLUTE_ERROR));
        
        plane3.intersection(plane1, plane2, intersection);
        assertTrue(intersection.equals(point, ABSOLUTE_ERROR));
        
        //Force NoIntersectionException by using two coincident or parallel 
        //planes
        try {
            plane1.getIntersection(plane1, plane2);
            fail("NoIntersectionException expected but not thrown");
        } catch (NoIntersectionException ignore) { }
        try {
            plane1.intersection(plane1, plane2, intersection);
            fail("NoIntersectionException expected but not thrown");
        } catch (NoIntersectionException ignore) { }
        
        
        //we could also find 7 random points to find 3 planes having one point 
        //in common, which will be their intersection
        Point3D point1 = new HomogeneousPoint3D();
        Point3D point2 = new HomogeneousPoint3D();
        Point3D point3 = new HomogeneousPoint3D();
        Point3D point4 = new HomogeneousPoint3D();
        Point3D point5 = new HomogeneousPoint3D();
        Point3D point6 = new HomogeneousPoint3D();
        Point3D point7 = new HomogeneousPoint3D();
        
        //create three intersecting planes using all points (each point is
        //defined as one row of the matrix)
        Matrix m1 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Matrix m2 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Matrix m3 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        
        //set 1st point in common in all three matrices
        m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1, 
                m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
        m3.setSubmatrix(0, 0, 0, HOM_COORDS - 1, 
                m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
        
        //ensure that all matrices have rank 3 (points are not colinear)
        while(com.irurueta.algebra.Utils.rank(m1) < 3 || 
                com.irurueta.algebra.Utils.rank(m2) < 3 || 
                com.irurueta.algebra.Utils.rank(m3) < 3){
            //create random matrices again
            m1 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            m2 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            m3 = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        
            //set 1st point in common in all three matrices
            m2.setSubmatrix(0, 0, 0, HOM_COORDS - 1, 
                    m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));
            m3.setSubmatrix(0, 0, 0, HOM_COORDS - 1, 
                    m1.getSubmatrix(0, 0, 0, HOM_COORDS - 1));            
        }
        
        point1.setCoordinates(m1.getSubmatrixAsArray(0, 0, 0, HOM_COORDS - 1));
        point2.setCoordinates(m1.getSubmatrixAsArray(1, 0, 1, HOM_COORDS - 1));
        point3.setCoordinates(m1.getSubmatrixAsArray(2, 0, 2, HOM_COORDS - 1));
        
        point4.setCoordinates(m2.getSubmatrixAsArray(1, 0, 1, HOM_COORDS - 1));
        point5.setCoordinates(m2.getSubmatrixAsArray(2, 0, 2, HOM_COORDS - 1));
        
        point6.setCoordinates(m3.getSubmatrixAsArray(1, 0, 1, HOM_COORDS - 1));
        point7.setCoordinates(m3.getSubmatrixAsArray(2, 0, 2, HOM_COORDS - 1));        
        
        //Create three planes between point1-point2-point3, 
        //point1-point4-point5 and point1-point6-point7
        plane1 = new Plane(point1, point2, point3);
        plane2 = new Plane(point1, point4, point5);
        plane3 = new Plane(point1, point6, point7);
        
        //because all three planes have in common point1, then their 
        //intersection must be point1
        assertTrue(plane1.getIntersection(plane2, plane3).equals(point1, 
                ABSOLUTE_ERROR));        
        plane1.intersection(plane2, plane3, intersection);
        assertTrue(intersection.equals(point1, ABSOLUTE_ERROR));
        
        assertTrue(plane2.getIntersection(plane1, plane3).equals(point1,
                ABSOLUTE_ERROR));
        plane2.intersection(plane1, plane3, intersection);
        assertTrue(intersection.equals(point1, ABSOLUTE_ERROR));
        
        assertTrue(plane3.getIntersection(plane1, plane2).equals(point1,
                ABSOLUTE_ERROR));
        plane3.intersection(plane1, plane2, intersection);
        assertTrue(intersection.equals(point1, ABSOLUTE_ERROR));        
    }
    
    @Test
    public void testClosestPoint() throws WrongSizeException, NotReadyException,
        LockedException, DecomposerException, 
        com.irurueta.algebra.NotAvailableException, ColinearPointsException {
        
        //randomly choose 3 points to find their corresponding plane.
        //Each point is represented as one row of matrix below
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        
        //ensure points are not colinear
        while(decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }
        
        Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0,
                HOM_COORDS - 1));
        Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1, 
                HOM_COORDS - 1));
        Point3D point3 = new HomogeneousPoint3D(m.getSubmatrixAsArray(2, 0, 2, 
                HOM_COORDS - 1));
        
        Plane plane = new Plane(point1, point2, point3);
        
        //point1, point2 and point3 belong to plane, hence their distance to the
        //plane is zero up to machine precision
        assertEquals(plane.signedDistance(point1), 0.0, PRECISION_ERROR);
        assertEquals(plane.signedDistance(point2), 0.0, PRECISION_ERROR);
        assertEquals(plane.signedDistance(point3), 0.0, PRECISION_ERROR);
        
        //because they belong to plane, their closest point to plane is 
        //themselves
        Point3D closestPoint1 = plane.getClosestPoint(point1);
        Point3D closestPoint1b = plane.getClosestPoint(point1, PRECISION_ERROR);
        Point3D closestPoint1c = Point3D.create();
        plane.closestPoint(point1, closestPoint1c);
        Point3D closestPoint1d = Point3D.create();
        plane.closestPoint(point1, closestPoint1d, ABSOLUTE_ERROR);
        
        assertTrue(closestPoint1.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1b.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1c.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPoint1d.equals(point1, ABSOLUTE_ERROR));


        Point3D closestPoint2 = plane.getClosestPoint(point2);
        Point3D closestPoint2b = plane.getClosestPoint(point2, PRECISION_ERROR);
        Point3D closestPoint2c = Point3D.create();
        plane.closestPoint(point2, closestPoint2c);
        Point3D closestPoint2d = Point3D.create();
        plane.closestPoint(point2, closestPoint2d, ABSOLUTE_ERROR);

        assertTrue(closestPoint2.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2b.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2c.equals(point2, ABSOLUTE_ERROR));
        assertTrue(closestPoint2d.equals(point2, ABSOLUTE_ERROR));


        Point3D closestPoint3 = plane.getClosestPoint(point3);
        Point3D closestPoint3b = plane.getClosestPoint(point3, PRECISION_ERROR);
        Point3D closestPoint3c = Point3D.create();
        plane.closestPoint(point3, closestPoint3c);
        Point3D closestPoint3d = Point3D.create();
        plane.closestPoint(point3, closestPoint3d, ABSOLUTE_ERROR);

        assertTrue(closestPoint3.equals(point3, ABSOLUTE_ERROR));
        assertTrue(closestPoint3b.equals(point3, ABSOLUTE_ERROR));
        assertTrue(closestPoint3c.equals(point3, ABSOLUTE_ERROR));
        assertTrue(closestPoint3d.equals(point3, ABSOLUTE_ERROR));
        
        
        //use plane director vector to find another point at desired signed
        //distance
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double signedDistance = randomizer.nextDouble(MIN_RANDOM_DISTANCE,
                MAX_RANDOM_DISTANCE);
        
        HomogeneousPoint3D point4 = new HomogeneousPoint3D();
        
        double normDirectorVector = Math.sqrt(plane.getA() * plane.getA() +
                plane.getB() * plane.getB() + plane.getC() * plane.getC());
        point4.setInhomogeneousCoordinates(point1.getInhomX() +
                signedDistance * plane.getA() / normDirectorVector,
                point1.getInhomY() + signedDistance * plane.getB() /
                normDirectorVector, point1.getInhomZ() + 
                signedDistance * plane.getC() / normDirectorVector);
        
        assertEquals(plane.signedDistance(point4), signedDistance, 
                PRECISION_ERROR);
        
        //because point4 goes in plane's perpendicular direction from point1, 
        //its closest point belonging to the plane will be point1
        Point3D closestPoint = plane.getClosestPoint(point4);
        Point3D closestPointB = plane.getClosestPoint(point4, PRECISION_ERROR);
        Point3D closestPointC = Point3D.create();
        plane.closestPoint(point4, closestPointC);
        Point3D closestPointD = Point3D.create();
        plane.closestPoint(point4, closestPointD, ABSOLUTE_ERROR);
        
        assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointB.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointC.equals(point1, ABSOLUTE_ERROR));
        assertTrue(closestPointD.equals(point1, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        try {
            plane.getClosestPoint(point4, -PRECISION_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            plane.closestPoint(point4, closestPointD, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testDotProduct() {
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        //a random line
        Plane plane1 = new Plane(array);
        
        //oposed sign line
        Plane plane2 = new Plane(-plane1.getA(), -plane1.getB(), 
                -plane1.getC(), -plane1.getD());
        
        //another random line
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Plane plane3 = new Plane(array);
        
        //test for equal lines
        assertEquals(plane1.dotProduct(plane1), 1.0, ABSOLUTE_ERROR);
        //test for oposed sign lines
        assertEquals(plane1.dotProduct(plane2), -1.0, ABSOLUTE_ERROR);
        //test for 2 random lines
        assertEquals(plane1.dotProduct(plane3), plane1.getA() * plane3.getA() +
                plane1.getB() * plane3.getB() + plane1.getC() * plane3.getC() +
                plane1.getD() * plane3.getD(), 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testEquals() {
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Plane plane1 = new Plane(array);
        Plane plane2 = new Plane(array);
        assertTrue(plane1.equals(plane2, ABSOLUTE_ERROR));
        assertTrue(plane1.equals(plane2));
        //noinspection all
        assertTrue(plane1.equals((Object)plane2));
        
        array[0] = plane1.getA() + randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        array[1] = plane1.getB();
        array[2] = plane1.getC();
        array[3] = plane1.getD();
        
        plane2 = new Plane(array);
        assertFalse(plane1.equals(plane2, 0.0));
        
        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        plane1 = new Plane(array);
        array[0] *= 2.0;
        plane2 = new Plane(array);
        assertTrue(plane1.equals(plane2, 2.0));
        assertFalse(plane1.equals(plane2, 0.0));
        
        //Force IllegalArgumentException
        try {
            //noinspection all
            plane1.equals(plane1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testCreateCanonicalPlaneAtInfinity() {
        Plane plane = Plane.createCanonicalPlaneAtInfinity();
        
        assertEquals(plane.getA(), 0.0, 0.0);
        assertEquals(plane.getB(), 0.0, 0.0);
        assertEquals(plane.getC(), 0.0, 0.0);
        assertEquals(plane.getD(), 1.0, 0.0);
        
        //create a point at infinity
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);
        
        //check that point at infinity is locus of plane
        assertTrue(plane.isLocus(point));
    }
    
    @Test
    public void testSetAsCanonicalPlaneAtInfinity() {
        //create a point at infinity
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D point = new HomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 0.0);
        
        Plane plane = new Plane(1.0, 1.0, 1.0, 1.0);
        
        //initially plane is not at infinity
        assertFalse(plane.isLocus(point));
        
        //set plane at infinity
        Plane.setAsCanonicalPlaneAtInfinity(plane);
        
        //check correctness
        assertEquals(plane.getA(), 0.0, 0.0);
        assertEquals(plane.getB(), 0.0, 0.0);
        assertEquals(plane.getC(), 0.0, 0.0);
        assertEquals(plane.getD(), 1.0, 0.0);
        
        //check that point at infinity is now locus of plane
        assertTrue(plane.isLocus(point));
    }
}
