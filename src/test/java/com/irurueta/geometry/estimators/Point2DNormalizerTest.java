/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.Point2DNormalizer
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 9, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
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

public class Point2DNormalizerTest {
    public static final int MIN_POINTS = 2;
    public static final int MAX_POINTS = 10;
    
    public static final double MIN_VALUE = -10.0;
    public static final double MAX_VALUE = 10.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 100;
    
    public Point2DNormalizerTest() {}
    
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        List<Point2D> points = new ArrayList<Point2D>();
        InhomogeneousPoint2D point;
        for(int i = 0; i < nPoints; i++){
            double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint2D(x, y);
            points.add(point);
        }
        
        Point2DNormalizer normalizer = new Point2DNormalizer(points);
        
        //check default values
        assertSame(normalizer.getPoints(), points);
        assertTrue(normalizer.isReady());
        assertFalse(normalizer.isLocked());
        assertEquals(normalizer.getMinInhomX(), Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMinInhomY(), Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMaxInhomX(), -Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMaxInhomY(), -Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getScaleX(), 1.0, 0.0);
        assertEquals(normalizer.getScaleY(), 1.0, 0.0);
        assertEquals(normalizer.getCentroidX(), 0.0, 0.0);
        assertEquals(normalizer.getCentroidY(), 0.0, 0.0);
        assertNull(normalizer.getTransformation());
        assertNull(normalizer.getInverseTransformation());
        assertFalse(normalizer.isResultAvailable());
        
        //Force IllegalArgumentException
        points.clear();
        assertFalse(normalizer.isReady());
        
        normalizer = null;
        try{
            normalizer = new Point2DNormalizer(points);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(normalizer);
    }
    
    @Test
    public void testGetSetPoints() throws LockedException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        int nPoints1 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        List<Point2D> points1 = new ArrayList<Point2D>();
        InhomogeneousPoint2D point;
        for(int i = 0; i < nPoints1; i++){
            double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint2D(x, y);
            points1.add(point);
        }
        
        int nPoints2 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        List<Point2D> points2 = new ArrayList<Point2D>();
        for(int i = 0; i < nPoints2; i++){
            double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint2D(x, y);
            points2.add(point);
        }
        
        Point2DNormalizer normalizer = new Point2DNormalizer(points1);
        
        //check default value
        assertSame(normalizer.getPoints(), points1);
        
        //set new value
        normalizer.setPoints(points2);
        
        //check correctness
        assertSame(normalizer.getPoints(), points2);
    }
    
    @Test
    public void testCompute() throws NotReadyException, LockedException, 
            WrongSizeException, NormalizerException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for(int t = 0; t < TIMES; t++){
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point2D> points = new ArrayList<Point2D>();
            InhomogeneousPoint2D point;
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, 
                    maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            for(int i = 0; i < nPoints; i++){
                double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                point = new InhomogeneousPoint2D(x, y);
                points.add(point);

                if(x < minX) minX = x;
                if(y < minY) minY = y;
                if(x > maxX) maxX = x;
                if(y > maxY) maxY = y;
            }

            double width = maxX - minX;
            double height = maxY - minY;
            double scaleX = 1.0 / width;
            double scaleY = 1.0 / height;
            double centroidX = (minX + maxX) / 2.0;
            double centroidY = (minY + maxY) / 2.0;

            Point2DNormalizer normalizer = new Point2DNormalizer(points);

            assertTrue(normalizer.isReady());
            assertFalse(normalizer.isLocked());

            normalizer.compute();

            assertFalse(normalizer.isLocked());

            assertEquals(normalizer.getMinInhomX(), minX, 0.0);
            assertEquals(normalizer.getMinInhomY(), minY, 0.0);
            assertEquals(normalizer.getMaxInhomX(), maxX, 0.0);
            assertEquals(normalizer.getMaxInhomY(), maxY, 0.0);
            assertEquals(normalizer.getScaleX(), scaleX, 0.0);
            assertEquals(normalizer.getScaleY(), scaleY, 0.0);
            assertEquals(normalizer.getCentroidX(), centroidX, 0.0);
            assertEquals(normalizer.getCentroidY(), centroidY, 0.0);

            ProjectiveTransformation2D transformation = 
                    normalizer.getTransformation();

            ProjectiveTransformation2D invTransformation = 
                    normalizer.getInverseTransformation();

            //test that invTransformation is indeed the inverse transformation
            Matrix T = transformation.asMatrix();
            Matrix invT = invTransformation.asMatrix();

            Matrix identity = invT.multiplyAndReturnNew(T);
            double norm = identity.getElementAt(0, 0);
            identity.multiplyByScalar(1.0 / norm); //normalize

            assertTrue(identity.equals(
                    Matrix.identity(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH), 
                    ABSOLUTE_ERROR));

            //normalize points
            List<Point2D> normPoints = transformation.transformPointsAndReturnNew(points);

            //compute centroid and scales
            minX = minY = Double.MAX_VALUE;
            maxX = maxY = -Double.MAX_VALUE;        
            for(Point2D normPoint : normPoints){
                double x = normPoint.getInhomX();
                double y = normPoint.getInhomY();

                if(x < minX) minX = x;
                if(y < minY) minY = y;
                if(x > maxX) maxX = x;
                if(y > maxY) maxY = y;
            }

            width = maxX - minX;
            height = maxY - minY;
            scaleX = 1.0 / width;
            scaleY = 1.0 / height;
            centroidX = (minX + maxX) / 2.0;
            centroidY = (minY + maxY) / 2.0;

            //check that points have been correctly normalized (scales = 1 and
            //centroid = [0, 0])
            assertEquals(width, 1.0, ABSOLUTE_ERROR);
            assertEquals(height, 1.0, ABSOLUTE_ERROR);
            assertEquals(scaleX, 1.0, ABSOLUTE_ERROR);
            assertEquals(scaleY, 1.0, ABSOLUTE_ERROR);
            assertEquals(centroidX, 0.0, ABSOLUTE_ERROR);
            assertEquals(centroidY, 0.0, ABSOLUTE_ERROR);

            //denormalize points and check that are equal to the original ones
            List<Point2D> denomPoints = invTransformation.transformPointsAndReturnNew(
                    normPoints);

            for(int i = 0; i < nPoints; i++){
                assertEquals(points.get(i).distanceTo(denomPoints.get(i)), 0.0, 
                        ABSOLUTE_ERROR);
            }
        }
    }
}
