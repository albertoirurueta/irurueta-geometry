/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.Line3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 15, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.ArrayUtils;
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
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class Line3DTest {
    
    public static final double MIN_RANDOM_VALUE = -10.0;
    public static final double MAX_RANDOM_VALUE = 10.0;
    
    public static final int INHOM_COORDS = 3;
    public static final int HOM_COORDS = 4;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    
    public Line3DTest() {
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
    public void testConstructor() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, NotAvailableException, 
        CoincidentPlanesException, NoIntersectionException, 
        CoincidentPointsException{
        
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
        
        //ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));
        
        //test constructor from 2 planes
        Line3D line = new Line3D(plane1, plane2);
        
        assertEquals(line.getPlane1(), plane1);
        assertEquals(line.getPlane2(), plane2);
        
        assertTrue(line.isLocus(point));
        
        //Force ParallelPlanesException (by using the same plane)
        line = null;
        try{
            line = new Line3D(plane1, plane1);
            fail("CoincidentPlanesException expected but not thrown");
        }catch(CoincidentPlanesException e){}
        assertNull(line);
        
        
        //test constructor from 2 non-coincident points
        m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while(Utils.rank(m) < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }
        
        Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0, 
                HOM_COORDS - 1));
        Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1, 
                HOM_COORDS - 1));
        
        line = new Line3D(point1, point2);
        
        //ensure that both point1 and point2 belong to line 3D
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));
        
        //planes get defined but we don't know their value, just that point1
        //and point2 belong to those planes too
        assertNotNull(line.getPlane1());
        assertNotNull(line.getPlane2());
        assertTrue(line.getPlane1().isLocus(point1));
        assertTrue(line.getPlane1().isLocus(point2));
        assertTrue(line.getPlane2().isLocus(point1));
        assertTrue(line.getPlane2().isLocus(point2));
        
        
        //Force CoincidentPointsException (by using the same point)
        line = null;
        try{
            line = new Line3D(point1, point1);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        assertNull(line);
    }
    
    @Test
    public void testAreParallelPlanes() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException,
        NotAvailableException, NoIntersectionException{
        
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
        
        //ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));

        //plane1, plane2 and plane3 are perpendicular
        assertFalse(Line3D.areCoincidentPlanes(plane1, plane2));
        assertFalse(Line3D.areCoincidentPlanes(plane1, plane3));
        assertFalse(Line3D.areCoincidentPlanes(plane2, plane3));
        
        //but using the same plane...
        assertTrue(Line3D.areCoincidentPlanes(plane1, plane1));
        assertTrue(Line3D.areCoincidentPlanes(plane2, plane2));
        assertTrue(Line3D.areCoincidentPlanes(plane3, plane3));
    }
    
    @Test
    public void testGetSetPlanes() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, NotAvailableException, 
        CoincidentPlanesException{
        
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

        Line3D line = new Line3D(plane1, plane2);
        assertEquals(line.getPlane1(), plane1);
        assertEquals(line.getPlane2(), plane2);
        
        //set new planes
        line.setPlanes(plane2, plane3);
        assertEquals(line.getPlane1(), plane2);
        assertEquals(line.getPlane2(), plane3);
        
        //Force ParallelPlanesException
        try{
            line.setPlanes(plane1, plane1);
            fail("CoincidentPlanesException expected but not thrown");
        }catch(CoincidentPlanesException e){}
    }
    
    @Test
    public void testSetPlanesFromPoints() throws WrongSizeException, 
        DecomposerException, CoincidentPointsException{
        
        //test constructor from 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while(Utils.rank(m) < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }
        
        Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0, 
                HOM_COORDS - 1));
        Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1, 
                HOM_COORDS - 1));
                
        
        Line3D line = new Line3D(point1, point2);
        //ensure that both point1 and point2 belong to line 3D
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));
        
        //planes get defined but we don't know their value, just that point1
        //and point2 belong to those planes too
        assertNotNull(line.getPlane1());
        assertNotNull(line.getPlane2());
        assertTrue(line.getPlane1().isLocus(point1));
        assertTrue(line.getPlane1().isLocus(point2));
        assertTrue(line.getPlane2().isLocus(point1));
        assertTrue(line.getPlane2().isLocus(point2));
        
        
        //now build another set of points        
        m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while(Utils.rank(m) < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }
        
        Point3D point3 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0, 
                HOM_COORDS - 1));
        Point3D point4 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1, 
                HOM_COORDS - 1));        
        
        line.setPlanesFromPoints(point3, point4);
        //ensure that both point3 and point4 belong to line 3D
        assertTrue(line.isLocus(point3));
        assertTrue(line.isLocus(point4));
        
        //planes get defined but we don't know their value, just that point1
        //and point2 belong to those planes too
        assertNotNull(line.getPlane1());
        assertNotNull(line.getPlane2());
        assertTrue(line.getPlane1().isLocus(point3));
        assertTrue(line.getPlane1().isLocus(point4));
        assertTrue(line.getPlane2().isLocus(point3));
        assertTrue(line.getPlane2().isLocus(point4));        
    }
    
    @Test
    public void testIsLocusDistanceAndClosestPoint() throws WrongSizeException, 
        NotReadyException, LockedException, DecomposerException,
        NotAvailableException, NoIntersectionException, 
        CoincidentPlanesException{
        
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
        
        //ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));
        
        //test constructor from 2 planes
        Line3D line = new Line3D(plane1, plane2);

        //test point is locus of line
        assertTrue(line.isLocus(point));
        assertTrue(line.isLocus(point, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try{
            line.isLocus(point, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //build another point at a given distance in perpendicular direction of
        //line.
        m = new Matrix(1, INHOM_COORDS);
        m.setSubmatrix(0, 0, 0, INHOM_COORDS - 1, line.getDirection());
        
        decomposer.setInputMatrix(m);
        decomposer.decompose();
        
        V = decomposer.getV();
        
        //because m has rank 1, the null space of m has rank 2, which are the 
        //last 2 columns of V, which are perpendicular to m
        Matrix perpendicular = V.getSubmatrix(0, 2, 2, 2); //pick last column 
                                                            //of V
        
        //determine amount of distance
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double dist = randomizer.nextDouble(ABSOLUTE_ERROR, MAX_RANDOM_VALUE);
        
        Point3D distPoint = Point3D.create();
        distPoint.setInhomogeneousCoordinates(
                point.getInhomX() + dist * perpendicular.getElementAtIndex(0), 
                point.getInhomY() + dist * perpendicular.getElementAtIndex(1), 
                point.getInhomZ() + dist * perpendicular.getElementAtIndex(2));
        
        //check that distPoint is indeed at distance dist
        assertEquals(line.getDistance(distPoint), dist, ABSOLUTE_ERROR);
        //check that distPoint is no longer locus of line 3D
        assertFalse(line.isLocus(distPoint));
        //but increasing enough the threshold, it becomes locus
        assertTrue(line.isLocus(distPoint, dist));
        
        //because distPoint has moved in perpendicular direction from point,
        //the closest point will be point
        assertTrue(line.getClosestPoint(distPoint).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));
        assertTrue(line.getClosestPoint(distPoint, ABSOLUTE_ERROR).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));
        
        Point3D closestPoint = Point3D.create();
        line.closestPoint(distPoint, closestPoint);
        assertTrue(closestPoint.equals(point.toInhomogeneous(), 
                ABSOLUTE_ERROR));
        line.closestPoint(distPoint, closestPoint, ABSOLUTE_ERROR);
        assertTrue(closestPoint.equals(point.toInhomogeneous(), 
                ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try{
            line.getClosestPoint(distPoint, -dist);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            line.closestPoint(distPoint, closestPoint, -dist);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testNormalizeAndIsNormalized() throws WrongSizeException, 
        DecomposerException, CoincidentPointsException{
        
        //test constructor from 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while(Utils.rank(m) < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }
        
        Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0, 
                HOM_COORDS - 1));
        Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1, 
                HOM_COORDS - 1));
        
        Line3D line = new Line3D(point1, point2);
        
        assertFalse(line.getPlane1().isNormalized());
        assertFalse(line.getPlane2().isNormalized());
        assertFalse(line.isNormalized());
        
        //normalize
        line.normalize();
        
        //check correctness
        assertTrue(line.getPlane1().isNormalized());
        assertTrue(line.getPlane2().isNormalized());
        assertTrue(line.isNormalized());
    }
    
    @Test
    public void testGetDirection() throws WrongSizeException, 
        DecomposerException,
        CoincidentPointsException{
        
        //test constructor from 2 non-coincident points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while(Utils.rank(m) < 2){
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }
        
        Point3D point1 = new HomogeneousPoint3D(m.getSubmatrixAsArray(0, 0, 0, 
                HOM_COORDS - 1));
        Point3D point2 = new HomogeneousPoint3D(m.getSubmatrixAsArray(1, 0, 1, 
                HOM_COORDS - 1));
        
        Line3D line = new Line3D(point1, point2);        
        
        double[] expectedDirection = Utils.crossProduct(
                line.getPlane1().getDirectorVector(), 
                line.getPlane2().getDirectorVector());

        double[] direction = line.getDirection();
        
        //check that direction is equal up to scale
        double norm1 = Utils.normF(expectedDirection);
        double norm2 = Utils.normF(direction);
        
        ArrayUtils.multiplyByScalar(expectedDirection, 1.0 / norm1, 
                expectedDirection);
        ArrayUtils.multiplyByScalar(direction, 1.0 / norm2, direction);
        
        double[] diff = ArrayUtils.subtractAndReturnNew(direction, 
                expectedDirection);
        
        double normDiff = Utils.normF(diff);
        assertTrue(normDiff < LARGE_ABSOLUTE_ERROR);
    }
    
    @Test
    public void testIntersection() throws WrongSizeException, NotReadyException, 
        LockedException, DecomposerException, NotAvailableException, 
        NoIntersectionException, CoincidentPlanesException{
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
        
        //ensure that point is the intersection of all 3 points
        assertTrue(plane1.getIntersection(plane2, plane3).equals(
                point.toInhomogeneous(), ABSOLUTE_ERROR));
        
        //test constructor from 2 planes
        Line3D line = new Line3D(plane1, plane2);
        
        //because line is made of plane1 and plane2, the intersection with
        //plane3 will be point
        assertTrue(line.getIntersection(plane3).equals(point.toInhomogeneous(),
                ABSOLUTE_ERROR));

        Point3D intersection = Point3D.create();
        line.intersection(plane3, intersection);
        assertTrue(intersection.equals(point.toInhomogeneous(), 
                ABSOLUTE_ERROR));
    }
}
