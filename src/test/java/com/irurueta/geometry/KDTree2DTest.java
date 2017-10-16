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

package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class KDTree2DTest {

    public static final double ABSOLUTE_ERROR = 1e-9;
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;

    public static final int MIN_POINTS = 50;
    public static final int MAX_POINTS = 500;

    public KDTree2DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConsructor() {
        KDTree2D tree = null;

        //test empty list
        List<Point2D> points = new ArrayList<>();

        //check
        try {
            tree = new KDTree2D(points);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(tree);

        //one point list
        points.add(Point2D.create());

        //check
        try {
            tree = new KDTree2D(points);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(tree);

        //two point list
        points.add(Point2D.create());

        //check
        try {
            tree = new KDTree2D(points);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(tree);

        //three point list
        points.add(Point2D.create());
        tree = new KDTree2D(points);

        assertNotNull(tree);

        //random list of points
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        tree = new KDTree2D(points);

        assertNotNull(tree);

        assertEquals(KDTree2D.MIN_PTS, 3);
    }

    @Test
    public void testDistance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        Point2D pi, pj;
        for (int i = 0; i < n; i++) {
            pi = points.get(i);

            for (int j = 0; j < n; j++) {

                if (i == j) {
                    //for equal indices distance is BIG
                    assertEquals(tree.distance(i, j), KDTree.BIG, 0.0);
                } else {
                    pj = points.get(j);
                    assertEquals(tree.distance(i, j), pi.distanceTo(pj), ABSOLUTE_ERROR);
                }
            }
        }
    }

    @Test
    public void testLocateBoxIndex() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        for (int i = 0; i < n; i++) {
            Point2D p = points.get(i);
            int boxIndex = tree.locateBoxIndex(p);

            KDTree2D.BoxNode<Point2D> box = tree.mBoxes[boxIndex];

            //point is inside box, so its distance is zero
            assertEquals(box.getDistance(p), 0.0, 0.0);
        }
    }

    @Test
    public void testLocateBox() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        for (int i = 0; i < n; i++) {
            Point2D p = points.get(i);
            KDTree2D.BoxNode<Point2D> box = tree.locateBox(p);

            //point is inside box, so its distance is zero
            assertEquals(box.getDistance(p), 0.0, 0.0);
        }
    }

    @Test
    public void testNearestIndex() {
        //test with a point inside the collection
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        int bestIndex = 0;
        for (int i = 0; i < n; i++) {
            Point2D pi = points.get(i);

            //find nearest
            bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {

                Point2D pj = points.get(j);
                dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                    bestIndex = j;
                }
            }

            int nearestIndex = tree.nearestIndex(pi);

            assertSame(points.get(nearestIndex), bestP);
            assertEquals(nearestIndex, bestIndex);
        }
    }

    @Test
    public void testNearestIndex2() {
        //test with a point not contained in the collection
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint2D p = new InhomogeneousPoint2D(x, y);

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        int bestIndex = 0;
        //find nearest
        bestDist = Double.MAX_VALUE;
        for (int j = 0; j < n; j++) {

            Point2D pj = points.get(j);
            dist = p.distanceTo(pj);
            if (dist < bestDist) {
                bestDist = dist;
                bestP = pj;
                bestIndex = j;
            }
        }

        int nearestIndex = tree.nearestIndex(p);

        assertSame(points.get(nearestIndex), bestP);
        assertEquals(nearestIndex, bestIndex);
    }

    @Test
    public void testNearestPoint() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        for (int i = 0; i < n; i++) {
            Point2D pi = points.get(i);

            //find nearest
            bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {

                Point2D pj = points.get(j);
                dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                }
            }

            Point2D nearestPoint = tree.nearestPoint(pi);

            assertSame(nearestPoint, bestP);
        }
    }

    @Test
    public void testNearestPoint2() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint2D p = new InhomogeneousPoint2D(x, y);

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        //find nearest
        bestDist = Double.MAX_VALUE;
        for (int j = 0; j < n; j++) {

            Point2D pj = points.get(j);
            dist = p.distanceTo(pj);
            if (dist < bestDist) {
                bestDist = dist;
                bestP = pj;
            }
        }

        Point2D nearestPoint = tree.nearestPoint(p);

        assertSame(nearestPoint, bestP);
    }

    @Test
    public void testNNearest() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        int bestIndex = 0;
        int[] nn = new int[1];
        double[] dn = new double[1];
        for (int i = 0; i < n; i++) {
            Point2D pi = points.get(i);

            //find nearest
            bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                Point2D pj = points.get(j);
                dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                    bestIndex = j;
                }
            }

            tree.nNearest(i, nn, dn, 1);

            assertEquals(nn[0], bestIndex);
            assertSame(bestP, points.get(nn[0]));
        }

        //Force IllegalArgumentException
        try {
            tree.nNearest(0, nn, dn, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(0, nn, dn, n);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(0, nn, dn, n-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testNNearest2() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        int numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double dist;
        double[] bestDistances = new double[numberNearest];
        int[] bestIndices = new int[numberNearest];
        int[] nn = new int[numberNearest];
        double[] dn = new double[numberNearest];
        for (int i = 0; i < numberPoints; i++) {
            Point2D pi = points.get(i);

            for (int k = 0; k < numberNearest; k++) {
                //find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (int j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    Point2D pj = points.get(j);
                    dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestIndices[k] = j;
                    }
                }
            }

            tree.nNearest(i, nn, dn, numberNearest);

            //check that result contains all nearest points
            for (int k = 0; k < numberNearest; k++) {
                boolean found = false;
                for(int m = 0; m < numberNearest; m++) {
                    if (bestIndices[k] == nn[m]) {
                        found = true;
                        break;
                    }
                }

                assertTrue(found);
            }
        }
    }

    @Test
    public void testNNearest3() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        int bestIndex = 0;
        int[] nn = new int[1];
        double[] dn = new double[1];
        for (int i = 0; i < n; i++) {
            Point2D pi = points.get(i);

            //find nearest
            bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                Point2D pj = points.get(j);
                dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                    bestIndex = j;
                }
            }

            tree.nNearest(pi, nn, dn, 1);

            assertEquals(nn[0], bestIndex);
            assertSame(bestP, points.get(nn[0]));
        }

        //Force IllegalArgumentException
        try {
            tree.nNearest(points.get(0), nn, dn, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(points.get(0), nn, dn, n);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(points.get(0), nn, dn, n-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testNNearest4() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        int numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double dist;
        double[] bestDistances = new double[numberNearest];
        int[] bestIndices = new int[numberNearest];
        int[] nn = new int[numberNearest];
        double[] dn = new double[numberNearest];
        for (int i = 0; i < numberPoints; i++) {
            Point2D pi = points.get(i);

            for (int k = 0; k < numberNearest; k++) {
                //find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (int j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    Point2D pj = points.get(j);
                    dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestIndices[k] = j;
                    }
                }
            }

            tree.nNearest(pi, nn, dn, numberNearest);

            //check that result contains all nearest points
            for (int k = 0; k < numberNearest; k++) {
                boolean found = false;
                for(int m = 0; m < numberNearest; m++) {
                    if (bestIndices[k] == nn[m]) {
                        found = true;
                        break;
                    }
                }

                assertTrue(found);
            }
        }
    }

    @Test
    public void testNNearest5() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        Point2D[] pn = new Point2D[1];
        double[] dn = new double[1];
        for (int i = 0; i < n; i++) {
            Point2D pi = points.get(i);

            //find nearest
            bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                Point2D pj = points.get(j);
                dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                }
            }

            tree.nNearest(i, pn, dn, 1);

            assertSame(bestP, pn[0]);
        }

        //Force IllegalArgumentException
        try {
            tree.nNearest(0, pn, dn, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(0, pn, dn, n);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(0, pn, dn, n-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testNNearest6() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        int numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double dist;
        double[] bestDistances = new double[numberNearest];
        Point2D[] bestPoints = new Point2D[numberNearest];
        Point2D[] pn = new Point2D[numberNearest];
        double[] dn = new double[numberNearest];
        for (int i = 0; i < numberPoints; i++) {
            Point2D pi = points.get(i);

            for (int k = 0; k < numberNearest; k++) {
                //find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (int j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    Point2D pj = points.get(j);
                    dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestPoints[k] = pj;
                    }
                }
            }

            tree.nNearest(i, pn, dn, numberNearest);

            //check that result contains all nearest points
            for (int k = 0; k < numberNearest; k++) {
                boolean found = false;
                for(int m = 0; m < numberNearest; m++) {
                    if (bestPoints[k] == pn[m]) {
                        found = true;
                        break;
                    }
                }

                assertTrue(found);
            }
        }
    }

    @Test
    public void testNNearest7() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double bestDist, dist;
        Point2D bestP = null;
        Point2D[] pn = new Point2D[1];
        double[] dn = new double[1];
        for (int i = 0; i < n; i++) {
            Point2D pi = points.get(i);

            //find nearest
            bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                Point2D pj = points.get(j);
                dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                }
            }

            tree.nNearest(pi, pn, dn, 1);

            assertSame(bestP, pn[0]);
        }

        //Force IllegalArgumentException
        try {
            tree.nNearest(points.get(0), pn, dn, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(points.get(0), pn, dn, n);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.nNearest(points.get(0), pn, dn, n-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testNNearest8() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        int numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));
        }

        KDTree2D tree = new KDTree2D(points);

        double dist;
        double[] bestDistances = new double[numberNearest];
        Point2D[] bestPoints = new Point2D[numberNearest];
        Point2D[] pn = new Point2D[numberNearest];
        double[] dn = new double[numberNearest];
        for (int i = 0; i < numberPoints; i++) {
            Point2D pi = points.get(i);

            for (int k = 0; k < numberNearest; k++) {
                //find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (int j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    Point2D pj = points.get(j);
                    dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestPoints[k] = pj;
                    }
                }
            }

            tree.nNearest(pi, pn, dn, numberNearest);

            //check that result contains all nearest points
            for (int k = 0; k < numberNearest; k++) {
                boolean found = false;
                for(int m = 0; m < numberNearest; m++) {
                    if (bestPoints[k] == pn[m]) {
                        found = true;
                        break;
                    }
                }

                assertTrue(found);
            }
        }
    }

    @Test
    public void testLocateNear() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));

            if (x < minX) {
                minX = x;
            }
            if (y < minY) {
                minY = y;
            }
            if (x > maxX) {
                maxX = x;
            }
            if (y > maxY) {
                maxY = y;
            }
        }

        Point2D lo = new InhomogeneousPoint2D(minX, minY);
        Point2D hi = new InhomogeneousPoint2D(maxX, maxY);
        double maxDist = lo.distanceTo(hi);

        double r = maxDist * randomizer.nextDouble(0.25, 0.5);

        KDTree2D tree = new KDTree2D(points);

        List<Point2D> expected = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            Point2D pi = points.get(i);

            expected.clear();

            for (int j = 0; j < numberPoints; j++) {
                Point2D pj = points.get(j);

                if (pi.distanceTo(pj) <= r) {
                    expected.add(pj);
                }
            }

            int numExpected = expected.size();
            int[] list = new int[numExpected];
            int result = tree.locateNear(pi, r, list, numExpected);

            //check
            assertEquals(result, numExpected);

            for (int j = 0; j < numExpected; j++) {
                Point2D pj = points.get(list[j]);

                assertTrue(expected.contains(pj));
                assertTrue(pj.distanceTo(pi) <= r);
            }
        }

        //Force IllegalArgumentException
        int[] list = new int[numberPoints];
        try {
            tree.locateNear(points.get(0), -1.0, list, numberPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.locateNear(points.get(0), 1.0, list, 0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        list = new int[numberPoints - 1];
        try {
            tree.locateNear(points.get(0), 1.0, list, numberPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testLocateNear2() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        List<Point2D> points = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint2D(x, y));

            if (x < minX) {
                minX = x;
            }
            if (y < minY) {
                minY = y;
            }
            if (x > maxX) {
                maxX = x;
            }
            if (y > maxY) {
                maxY = y;
            }
        }

        Point2D lo = new InhomogeneousPoint2D(minX, minY);
        Point2D hi = new InhomogeneousPoint2D(maxX, maxY);
        double maxDist = lo.distanceTo(hi);

        double r = maxDist * randomizer.nextDouble(0.25, 0.5);

        KDTree2D tree = new KDTree2D(points);

        List<Point2D> expected = new ArrayList<>();
        for (int i = 0; i < numberPoints; i++) {
            Point2D pi = points.get(i);

            expected.clear();

            for (int j = 0; j < numberPoints; j++) {
                Point2D pj = points.get(j);

                if (pi.distanceTo(pj) <= r) {
                    expected.add(pj);
                }
            }

            int numExpected = expected.size();
            Point2D[] plist = new Point2D[numExpected];
            int result = tree.locateNear(pi, r, plist, numExpected);

            //check
            assertEquals(result, numExpected);

            for (int j = 0; j < numExpected; j++) {
                Point2D pj = plist[j];

                assertTrue(expected.contains(pj));
                assertTrue(pj.distanceTo(pi) <= r);
            }
        }

        //Force IllegalArgumentException
        Point2D[] plist = new Point2D[numberPoints];
        try {
            tree.locateNear(points.get(0), -1.0, plist, numberPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            tree.locateNear(points.get(0), 1.0, plist, 0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        plist = new Point2D[numberPoints - 1];
        try {
            tree.locateNear(points.get(0), 1.0, plist, numberPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testBoxNode() {

        Point2D lo = Point2D.create();
        Point2D hi = Point2D.create();
        int mom = 1;
        int d1 = 2;
        int d2 = 3;
        int ptLo = 4;
        int ptHi = 5;

        KDTree.BoxNode<Point2D> node = new KDTree.BoxNode<>(lo, hi, mom, d1, d2, ptLo, ptHi);

        //check
        assertSame(node.getLo(), lo);
        assertSame(node.getHi(), hi);
        assertEquals(node.getMom(), mom);
        assertEquals(node.getDau1(), d1);
        assertEquals(node.getDau2(), d2);
        assertEquals(node.getPtLo(), ptLo);
        assertEquals(node.getPtHi(), ptHi);

        //Force IllegalArgumentException
        try {
            node.setLo(hi);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            node.setHi(lo);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            node.setBounds(hi, lo);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
