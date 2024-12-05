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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class KDTree3DTest {

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int MIN_POINTS = 50;
    private static final int MAX_POINTS = 500;

    @Test
    void testConstructor() {
        // test empty list
        final var points = new ArrayList<Point3D>();

        // check
        assertThrows(IllegalArgumentException.class, () -> new KDTree3D(points));

        // one point list
        points.add(Point3D.create());

        // check
        assertThrows(IllegalArgumentException.class, () -> new KDTree3D(points));

        // two point list
        points.add(Point3D.create());

        // check
        assertThrows(IllegalArgumentException.class, () -> new KDTree3D(points));

        // three point list
        points.add(Point3D.create());
        var tree = new KDTree3D(points);

        assertNotNull(tree);

        // random list of points
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points2 = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points2.add(new InhomogeneousPoint3D(x, y, z));
        }

        tree = new KDTree3D(points2);

        assertNotNull(tree);

        assertEquals(3, KDTree3D.MIN_PTS);
    }

    @Test
    void testDistance() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);
            for (var j = 0; j < n; j++) {
                if (i == j) {
                    // for equal indices distance is BIG
                    assertEquals(KDTree.BIG, tree.distance(i, j), 0.0);
                } else {
                    final var pj = points.get(j);
                    assertEquals(tree.distance(i, j), pi.distanceTo(pj), ABSOLUTE_ERROR);
                }
            }
        }
    }

    @Test
    void testLocateBoxIndex() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        for (var i = 0; i < n; i++) {
            final var p = points.get(i);
            final var boxIndex = tree.locateBoxIndex(p);

            final var box = tree.boxes[boxIndex];

            // point is inside box, so its distance is zero
            assertEquals(0.0, box.getDistance(p), 0.0);
        }
    }

    @Test
    void testLocateBox() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        for (var i = 0; i < n; i++) {
            final var p = points.get(i);
            final var box = tree.locateBox(p);

            // point is inside box, so its distance is zero
            assertEquals(0.0, box.getDistance(p), 0.0);
        }
    }

    @Test
    void testNearestIndex() {
        // test with a point inside the collection
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        var bestIndex = 0;
        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);

            // find nearest
            var bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                final var pj = points.get(j);
                final var dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                    bestIndex = j;
                }
            }

            final var nearestIndex = tree.nearestIndex(pi);

            assertSame(bestP, points.get(nearestIndex));
            assertEquals(bestIndex, nearestIndex);
        }
    }

    @Test
    void testNearestIndex2() {
        // test with a point not contained in the collection
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var p = new InhomogeneousPoint3D(x, y, z);

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        var bestIndex = 0;
        // find nearest
        var bestDist = Double.MAX_VALUE;
        for (var j = 0; j < n; j++) {
            final var pj = points.get(j);
            final var dist = p.distanceTo(pj);
            if (dist < bestDist) {
                bestDist = dist;
                bestP = pj;
                bestIndex = j;
            }
        }

        final var nearestIndex = tree.nearestIndex(p);

        assertSame(bestP, points.get(nearestIndex));
        assertEquals(bestIndex, nearestIndex);
    }

    @Test
    void testNearestPoint() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);

            // find nearest
            var bestDist = Double.MAX_VALUE;
            for (var j = 0; j < n; j++) {
                final var pj = points.get(j);
                final var dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                }
            }

            final var nearestPoint = tree.nearestPoint(pi);

            assertSame(bestP, nearestPoint);
        }
    }

    @Test
    void testNearestPoint2() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var p = new InhomogeneousPoint3D(x, y, z);

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        // find nearest
        var bestDist = Double.MAX_VALUE;
        for (int j = 0; j < n; j++) {
            final var pj = points.get(j);
            final var dist = p.distanceTo(pj);
            if (dist < bestDist) {
                bestDist = dist;
                bestP = pj;
            }
        }

        final var nearestPoint = tree.nearestPoint(p);
        assertSame(bestP, nearestPoint);
    }

    @Test
    void testNNearest() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        var bestIndex = 0;
        final var nn = new int[1];
        final var dn = new double[1];
        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);

            // find nearest
            var bestDist = Double.MAX_VALUE;
            for (int j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                final var pj = points.get(j);
                final var dist = pi.distanceTo(pj);
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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(0, nn, dn, -1));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(0, nn, dn, n));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(0, nn, dn, n - 1));
    }

    @Test
    void testNNearest2() {
        final var randomizer = new UniformRandomizer();
        final var numberPoints = randomizer.nextInt(MIN_POINTS + 1, MAX_POINTS);

        final var numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        final var bestDistances = new double[numberNearest];
        final var bestIndices = new int[numberNearest];
        final var nn = new int[numberNearest];
        final var dn = new double[numberNearest];
        for (var i = 0; i < numberPoints; i++) {
            final var pi = points.get(i);

            for (var k = 0; k < numberNearest; k++) {
                // find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (var j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    final var pj = points.get(j);
                    final var dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestIndices[k] = j;
                    }
                }
            }

            tree.nNearest(i, nn, dn, numberNearest);

            // check that result contains all nearest points
            for (var k = 0; k < numberNearest; k++) {
                var found = false;
                for (var m = 0; m < numberNearest; m++) {
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
    void testNNearest3() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        var bestIndex = 0;
        final var nn = new int[1];
        final var dn = new double[1];
        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);

            // find nearest
            var bestDist = Double.MAX_VALUE;
            for (var j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                final var pj = points.get(j);
                final var dist = pi.distanceTo(pj);
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

        // Force IllegalArgumentException
        final var p0 = points.get(0);
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(p0, nn, dn, -1));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(p0, nn, dn, n));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(p0, nn, dn, n - 1));
    }

    @Test
    void testNNearest4() {
        final var randomizer = new UniformRandomizer();
        final var numberPoints = randomizer.nextInt(MIN_POINTS + 1, MAX_POINTS);

        final var numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        final var bestDistances = new double[numberNearest];
        final var bestIndices = new int[numberNearest];
        final var nn = new int[numberNearest];
        final var dn = new double[numberNearest];
        for (var i = 0; i < numberPoints; i++) {
            final var pi = points.get(i);

            for (var k = 0; k < numberNearest; k++) {
                // find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (var j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    final var pj = points.get(j);
                    final var dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestIndices[k] = j;
                    }
                }
            }

            tree.nNearest(pi, nn, dn, numberNearest);

            // check that result contains all nearest points
            for (var k = 0; k < numberNearest; k++) {
                var found = false;
                for (var m = 0; m < numberNearest; m++) {
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
    void testNNearest5() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        final var pn = new Point3D[1];
        final var dn = new double[1];
        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);

            // find nearest
            var bestDist = Double.MAX_VALUE;
            for (var j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                final var pj = points.get(j);
                final var dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                }
            }

            tree.nNearest(i, pn, dn, 1);

            assertSame(bestP, pn[0]);
        }

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(0, pn, dn, -1));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(0, pn, dn, n));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(0, pn, dn, n - 1));
    }

    @Test
    void testNNearest6() {
        final var randomizer = new UniformRandomizer();
        final var numberPoints = randomizer.nextInt(MIN_POINTS + 1, MAX_POINTS);

        final var numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        final var bestDistances = new double[numberNearest];
        final var bestPoints = new Point3D[numberNearest];
        final var pn = new Point3D[numberNearest];
        final var dn = new double[numberNearest];
        for (var i = 0; i < numberPoints; i++) {
            final var pi = points.get(i);

            for (var k = 0; k < numberNearest; k++) {
                // find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (var j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    final var pj = points.get(j);
                    final var dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestPoints[k] = pj;
                    }
                }
            }

            tree.nNearest(i, pn, dn, numberNearest);

            // check that result contains all nearest points
            for (var k = 0; k < numberNearest; k++) {
                var found = false;
                for (var m = 0; m < numberNearest; m++) {
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
    void testNNearest7() {
        final var randomizer = new UniformRandomizer();
        final var n = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < n; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        Point3D bestP = null;
        final var pn = new Point3D[1];
        final var dn = new double[1];
        for (var i = 0; i < n; i++) {
            final var pi = points.get(i);

            // find nearest
            var bestDist = Double.MAX_VALUE;
            for (var j = 0; j < n; j++) {
                if (i == j) {
                    continue;
                }

                final var pj = points.get(j);
                final var dist = pi.distanceTo(pj);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestP = pj;
                }
            }

            tree.nNearest(pi, pn, dn, 1);

            assertSame(bestP, pn[0]);
        }

        // Force IllegalArgumentException
        final var p0 = points.get(0);
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(p0, pn, dn, -1));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(p0, pn, dn, n));
        assertThrows(IllegalArgumentException.class, () -> tree.nNearest(p0, pn, dn, n - 1));
    }

    @Test
    void testNNearest8() {
        final var randomizer = new UniformRandomizer();
        final var numberPoints = randomizer.nextInt(MIN_POINTS + 1, MAX_POINTS);

        final var numberNearest = randomizer.nextInt(MIN_POINTS, numberPoints);

        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));
        }

        final var tree = new KDTree3D(points);

        final var bestDistances = new double[numberNearest];
        final var bestPoints = new Point3D[numberNearest];
        final var pn = new Point3D[numberNearest];
        final var dn = new double[numberNearest];
        for (var i = 0; i < numberPoints; i++) {
            final var pi = points.get(i);

            for (var k = 0; k < numberNearest; k++) {
                // find nearest
                bestDistances[k] = Double.MAX_VALUE;
                for (var j = 0; j < numberPoints; j++) {
                    if (i == j) {
                        continue;
                    }

                    final var pj = points.get(j);
                    final var dist = pi.distanceTo(pj);
                    if (dist < bestDistances[k]) {
                        bestDistances[k] = dist;
                        bestPoints[k] = pj;
                    }
                }
            }

            tree.nNearest(pi, pn, dn, numberNearest);

            // check that result contains all nearest points
            for (var k = 0; k < numberNearest; k++) {
                var found = false;
                for (var m = 0; m < numberNearest; m++) {
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
    void testLocateNear() {
        final var randomizer = new UniformRandomizer();
        final var numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        var minX = Double.MAX_VALUE;
        var minY = Double.MAX_VALUE;
        var minZ = Double.MAX_VALUE;
        var maxX = -Double.MAX_VALUE;
        var maxY = -Double.MAX_VALUE;
        var maxZ = -Double.MAX_VALUE;
        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));

            if (x < minX) {
                minX = x;
            }
            if (y < minY) {
                minY = y;
            }
            if (z < minZ) {
                minZ = z;
            }
            if (x > maxX) {
                maxX = x;
            }
            if (y > maxY) {
                maxY = y;
            }
            if (z > maxZ) {
                maxZ = z;
            }
        }

        final var lo = new InhomogeneousPoint3D(minX, minY, minZ);
        final var hi = new InhomogeneousPoint3D(maxX, maxY, maxZ);
        final var maxDist = lo.distanceTo(hi);

        final var r = maxDist * randomizer.nextDouble(0.25, 0.5);

        final var tree = new KDTree3D(points);

        final var expected = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var pi = points.get(i);

            expected.clear();

            for (var j = 0; j < numberPoints; j++) {
                final var pj = points.get(j);

                if (pi.distanceTo(pj) <= r) {
                    expected.add(pj);
                }
            }

            final var numExpected = expected.size();
            final var list = new int[numExpected];
            final var result = tree.locateNear(pi, r, list, numExpected);

            // check
            assertEquals(result, numExpected);

            for (var j = 0; j < numExpected; j++) {
                final var pj = points.get(list[j]);

                assertTrue(expected.contains(pj));
                assertTrue(pj.distanceTo(pi) <= r);
            }
        }

        // Force IllegalArgumentException
        final var list = new int[numberPoints];
        final var p0 = points.get(0);
        assertThrows(IllegalArgumentException.class, () -> tree.locateNear(p0, -1.0, list, numberPoints));
        assertThrows(IllegalArgumentException.class, () -> tree.locateNear(p0, 1.0, list, 0));

        final var list2 = new int[numberPoints - 1];
        assertThrows(IllegalArgumentException.class, () -> tree.locateNear(p0, 1.0, list2, numberPoints));
    }

    @Test
    void testLocateNear2() {
        final var randomizer = new UniformRandomizer();
        final var numberPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        var minX = Double.MAX_VALUE;
        var minY = Double.MAX_VALUE;
        var minZ = Double.MAX_VALUE;
        var maxX = -Double.MAX_VALUE;
        var maxY = -Double.MAX_VALUE;
        var maxZ = -Double.MAX_VALUE;
        final var points = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            points.add(new InhomogeneousPoint3D(x, y, z));

            if (x < minX) {
                minX = x;
            }
            if (y < minY) {
                minY = y;
            }
            if (z < minZ) {
                minZ = z;
            }
            if (x > maxX) {
                maxX = x;
            }
            if (y > maxY) {
                maxY = y;
            }
            if (z > maxZ) {
                maxZ = z;
            }
        }

        final var lo = new InhomogeneousPoint3D(minX, minY, minZ);
        final var hi = new InhomogeneousPoint3D(maxX, maxY, maxZ);
        final var maxDist = lo.distanceTo(hi);

        final var r = maxDist * randomizer.nextDouble(0.25, 0.5);

        final var tree = new KDTree3D(points);

        final var expected = new ArrayList<Point3D>();
        for (var i = 0; i < numberPoints; i++) {
            final var pi = points.get(i);

            expected.clear();

            for (var j = 0; j < numberPoints; j++) {
                final var pj = points.get(j);

                if (pi.distanceTo(pj) <= r) {
                    expected.add(pj);
                }
            }

            final var numExpected = expected.size();
            final var plist = new Point3D[numExpected];
            final var result = tree.locateNear(pi, r, plist, numExpected);

            // check
            assertEquals(result, numExpected);

            for (var j = 0; j < numExpected; j++) {
                final var pj = plist[j];

                assertTrue(expected.contains(pj));
                assertTrue(pj.distanceTo(pi) <= r);
            }
        }

        // Force IllegalArgumentException
        final var plist = new Point3D[numberPoints];
        final var p0 = points.get(0);
        assertThrows(IllegalArgumentException.class, () -> tree.locateNear(p0, -1.0, plist, numberPoints));
        assertThrows(IllegalArgumentException.class, () -> tree.locateNear(p0, 1.0, plist, 0));

        final var plist2 = new Point3D[numberPoints - 1];
        assertThrows(IllegalArgumentException.class, () -> tree.locateNear(p0, 1.0, plist2, numberPoints));
    }

    @Test
    void testBoxNode() {

        final var lo = Point3D.create();
        final var hi = Point3D.create();
        final var mom = 1;
        final var d1 = 2;
        final var d2 = 3;
        final var ptLo = 4;
        final var ptHi = 5;

        final var node = new KDTree.BoxNode<>(lo, hi, mom, d1, d2, ptLo, ptHi);

        // check
        assertSame(node.getLo(), lo);
        assertSame(node.getHi(), hi);
        assertEquals(mom, node.getMom());
        assertEquals(d1, node.getDau1());
        assertEquals(d2, node.getDau2());
        assertEquals(ptLo, node.getPtLo());
        assertEquals(ptHi, node.getPtHi());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> node.setLo(hi));
        assertThrows(IllegalArgumentException.class, () -> node.setHi(lo));
        assertThrows(IllegalArgumentException.class, () -> node.setBounds(hi, lo));
    }
}
