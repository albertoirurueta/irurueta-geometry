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
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class Point3DNormalizerTest {
    private static final int MIN_POINTS = 2;
    private static final int MAX_POINTS = 10;
    
    private static final double MIN_VALUE = -10.0;
    private static final double MAX_VALUE = 10.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final int TIMES = 100;
    
    public Point3DNormalizerTest() { }
    
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        List<Point3D> points = new ArrayList<>();
        InhomogeneousPoint3D point;
        for (int i = 0; i < nPoints; i++) {
            double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint3D(x, y, z);
            points.add(point);
        }
        
        Point3DNormalizer normalizer = new Point3DNormalizer(points);
        
        //check default values
        assertSame(normalizer.getPoints(), points);
        assertTrue(normalizer.isReady());
        assertFalse(normalizer.isLocked());
        assertEquals(normalizer.getMinInhomX(), Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMinInhomY(), Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMinInhomZ(), Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMaxInhomX(), -Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMaxInhomY(), -Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getMaxInhomZ(), -Double.MAX_VALUE, 0.0);
        assertEquals(normalizer.getScaleX(), 1.0, 0.0);
        assertEquals(normalizer.getScaleY(), 1.0, 0.0);
        assertEquals(normalizer.getScaleZ(), 1.0, 0.0);
        assertEquals(normalizer.getCentroidX(), 0.0, 0.0);
        assertEquals(normalizer.getCentroidY(), 0.0, 0.0);
        assertNull(normalizer.getTransformation());
        assertNull(normalizer.getInverseTransformation());
        assertFalse(normalizer.isResultAvailable());
        
        //Force IllegalArgumentException
        points.clear();
        assertFalse(normalizer.isReady());
        
        normalizer = null;
        try {
            normalizer = new Point3DNormalizer(points);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(normalizer);
    }
    
    @Test
    public void testGetSetPoints() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        int nPoints1 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        List<Point3D> points1 = new ArrayList<>();
        InhomogeneousPoint3D point;
        for (int i = 0; i < nPoints1; i++) {
            double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint3D(x, y, z);
            points1.add(point);
        }
        
        int nPoints2 = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        List<Point3D> points2 = new ArrayList<>();
        for (int i = 0; i < nPoints2; i++) {
            double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            point = new InhomogeneousPoint3D(x, y, z);
            points2.add(point);
        }
        
        Point3DNormalizer normalizer = new Point3DNormalizer(points1);
        
        //check default value
        assertSame(normalizer.getPoints(), points1);
        
        //set new value
        normalizer.setPoints(points2);
        
        //check correctness
        assertSame(normalizer.getPoints(), points2);
    }    
    
    @Test
    public void testCompute() throws NotReadyException, LockedException, 
            WrongSizeException, NormalizerException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int t = 0; t < TIMES; t++) {
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> points = new ArrayList<>();
            InhomogeneousPoint3D point;
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE,
                    minZ = Double.MAX_VALUE, maxX = -Double.MAX_VALUE,
                    maxY = -Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;
            for (int i = 0; i < nPoints; i++) {
                double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                point = new InhomogeneousPoint3D(x, y, z);
                points.add(point);
                
                if(x < minX) minX = x;
                if(y < minY) minY = y;
                if(z < minZ) minZ = z;
                if(x > maxX) maxX = x;
                if(y > maxY) maxY = y;
                if(z > maxZ) maxZ = z;
            }
            
            double width = maxX - minX;
            double height = maxY - minY;
            double depth = maxZ - minZ;
            double scaleX = 1.0 / width;
            double scaleY = 1.0 / height;
            double scaleZ = 1.0 / depth;
            double centroidX = (minX + maxX) / 2.0;
            double centroidY = (minY + maxY) / 2.0;
            double centroidZ = (minZ + maxZ) / 2.0;
            
            Point3DNormalizer normalizer = new Point3DNormalizer(points);
            
            assertTrue(normalizer.isReady());
            assertFalse(normalizer.isLocked());
            
            normalizer.compute();
            
            assertFalse(normalizer.isLocked());
            
            assertEquals(normalizer.getMinInhomX(), minX, 0.0);
            assertEquals(normalizer.getMinInhomY(), minY, 0.0);
            assertEquals(normalizer.getMinInhomZ(), minZ, 0.0);
            assertEquals(normalizer.getMaxInhomX(), maxX, 0.0);
            assertEquals(normalizer.getMaxInhomY(), maxY, 0.0);
            assertEquals(normalizer.getMaxInhomZ(), maxZ, 0.0);
            assertEquals(normalizer.getScaleX(), scaleX, 0.0);
            assertEquals(normalizer.getScaleY(), scaleY, 0.0);
            assertEquals(normalizer.getScaleZ(), scaleZ, 0.0);
            assertEquals(normalizer.getCentroidX(), centroidX, 0.0);
            assertEquals(normalizer.getCentroidY(), centroidY, 0.0);
            assertEquals(normalizer.getCentroidZ(), centroidZ, 0.0);
            
            ProjectiveTransformation3D transformation =
                    normalizer.getTransformation();
            
            ProjectiveTransformation3D invTransformation =
                    normalizer.getInverseTransformation();
            
            //test that invTransformation is indeed the inverse transformation
            Matrix T = transformation.asMatrix();
            Matrix invT = invTransformation.asMatrix();
            
            Matrix identity = invT.multiplyAndReturnNew(T);
            double norm = identity.getElementAt(0, 0);
            identity.multiplyByScalar(1.0 / norm); //normalize
            
            assertTrue(identity.equals(Matrix.identity(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH), 
                    ABSOLUTE_ERROR));
            
            //normalize points
            List<Point3D> normPoints = transformation.transformPointsAndReturnNew(points);
            
            //compute centroid and scales
            minX = minY = minZ = Double.MAX_VALUE;
            maxX = maxY = maxZ = -Double.MAX_VALUE;
            for (Point3D normPoint : normPoints) {
                double x = normPoint.getInhomX();
                double y = normPoint.getInhomY();
                double z = normPoint.getInhomZ();
                
                if(x < minX) minX = x;
                if(y < minY) minY = y;
                if(z < minZ) minZ = z;
                if(x > maxX) maxX = x;
                if(y > maxY) maxY = y;
                if(z > maxZ) maxZ = z;
            }
            
            width = maxX - minX;
            height = maxY - minY;
            depth = maxZ - minZ;
            scaleX = 1.0 / width;
            scaleY = 1.0 / height;
            scaleZ = 1.0 / depth;
            centroidX = (minX + maxX) / 2.0;
            centroidY = (minY + maxY) / 2.0;
            centroidZ = (minZ + maxZ) / 2.0;
            
            //check that points have been correctly normalized (scales = 1 and
            //centroid = [0,0,0])
            assertEquals(width, 1.0, ABSOLUTE_ERROR);
            assertEquals(height, 1.0, ABSOLUTE_ERROR);
            assertEquals(depth, 1.0, ABSOLUTE_ERROR);
            assertEquals(scaleX, 1.0, ABSOLUTE_ERROR);
            assertEquals(scaleY, 1.0, ABSOLUTE_ERROR);
            assertEquals(scaleZ, 1.0, ABSOLUTE_ERROR);
            assertEquals(centroidX, 0.0, ABSOLUTE_ERROR);
            assertEquals(centroidY, 0.0, ABSOLUTE_ERROR);
            assertEquals(centroidZ, 0.0, ABSOLUTE_ERROR);
            
            //denormalize points and check that are equal to the original ones
            List<Point3D> denomPoints = invTransformation.transformPointsAndReturnNew(
                    normPoints);
            
            for (int i = 0; i < nPoints; i++) {
                assertEquals(points.get(i).distanceTo(denomPoints.get(i)), 0.0, 
                        ABSOLUTE_ERROR);
            }
        }
    }
}
