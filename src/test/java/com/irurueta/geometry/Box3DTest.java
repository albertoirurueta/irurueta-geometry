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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

public class Box3DTest {

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    public void testConstructor() {
        // test empty constructor
        Box3D box = new Box3D();

        // check default values
        assertEquals(box.getLo(), new InhomogeneousPoint3D(-0.5, -0.5, -0.5));
        assertEquals(box.getHi(), new InhomogeneousPoint3D(0.5, 0.5, 0.5));

        // test constructor with lo and hi
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double loZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final double hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);
        final double hiZ = randomizer.nextDouble(loZ, MAX_RANDOM_VALUE);

        final InhomogeneousPoint3D lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final InhomogeneousPoint3D hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);
        box = new Box3D(lo, hi);

        // check
        assertSame(box.getLo(), lo);
        assertSame(box.getHi(), hi);
    }

    @Test
    public void testGetSetLo() {
        final Box3D box = new Box3D();

        // check default value
        assertEquals(box.getLo(), new InhomogeneousPoint3D(-0.5, -0.5, -0.5));

        // set new value
        final InhomogeneousPoint3D lo = new InhomogeneousPoint3D();
        box.setLo(lo);

        // check
        assertSame(box.getLo(), lo);
    }

    @Test
    public void testGetSetHi() {
        final Box3D box = new Box3D();

        // check default value
        assertEquals(box.getHi(), new InhomogeneousPoint3D(0.5, 0.5, 0.5));

        // set new value
        final InhomogeneousPoint3D hi = new InhomogeneousPoint3D();
        box.setHi(hi);

        // check
        assertSame(box.getHi(), hi);
    }

    @Test
    public void testSetBounds() {
        final Box3D box = new Box3D();

        // check default values
        assertEquals(box.getLo(), new InhomogeneousPoint3D(-0.5, -0.5, -0.5));
        assertEquals(box.getHi(), new InhomogeneousPoint3D(0.5, 0.5, 0.5));

        // set bounds
        InhomogeneousPoint3D lo = new InhomogeneousPoint3D();
        InhomogeneousPoint3D hi = new InhomogeneousPoint3D();
        box.setBounds(lo, hi);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double loZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final double hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);
        final double hiZ = randomizer.nextDouble(loZ, MAX_RANDOM_VALUE);

        lo = new InhomogeneousPoint3D(loX, loY, loZ);
        hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);
        box.setBounds(loX, loY, loZ, hiX, hiY, hiZ);

        // check
        assertEquals(box.getLo(), lo);
        assertEquals(box.getHi(), hi);
    }

    @Test
    public void testGetDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double centerX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double centerY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double centerZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double sizeX = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final double sizeY = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final double sizeZ = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final double loX = centerX - 0.5 * sizeX;
        final double loY = centerY - 0.5 * sizeY;
        final double loZ = centerZ - 0.5 * sizeZ;

        final double hiX = centerX + 0.5 * sizeX;
        final double hiY = centerY + 0.5 * sizeY;
        final double hiZ = centerZ + 0.5 * sizeZ;

        final InhomogeneousPoint3D lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final InhomogeneousPoint3D hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);

        final Point3D pointXLoSide = new InhomogeneousPoint3D(centerX -
                sizeX * randomizer.nextDouble(1.5, 2.5), centerY, centerZ);
        final Point3D pointYLoSide = new InhomogeneousPoint3D(centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5), centerZ);
        final Point3D pointZLoSide = new InhomogeneousPoint3D(centerX, centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXHiSide = new InhomogeneousPoint3D(centerX +
                sizeX * randomizer.nextDouble(1.5, 2.5), centerY, centerZ);
        final Point3D pointYHiSide = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5), centerZ);
        final Point3D pointZHiSide = new InhomogeneousPoint3D(centerX, centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointYZLoSide = new InhomogeneousPoint3D(centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointYZHiSide = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointXYMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointYZMixedSide1 = new InhomogeneousPoint3D(centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointYZMixedSide2 = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide2 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide3 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide4 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide5 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide6 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D center = new InhomogeneousPoint3D(centerX, centerY, centerZ);

        final Box3D box = new Box3D(lo, hi);
        assertEquals(box.getDistance(pointXLoSide),
                pointXLoSide.distanceTo(new InhomogeneousPoint3D(loX, centerY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointYLoSide),
                pointYLoSide.distanceTo(new InhomogeneousPoint3D(centerX, loY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointZLoSide),
                pointZLoSide.distanceTo(new InhomogeneousPoint3D(centerX, centerY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(pointXHiSide),
                pointXHiSide.distanceTo(new InhomogeneousPoint3D(hiX, centerY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointYHiSide),
                pointYHiSide.distanceTo(new InhomogeneousPoint3D(centerX, hiY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointZHiSide),
                pointZHiSide.distanceTo(new InhomogeneousPoint3D(centerX, centerY, hiZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(pointXYLoSide),
                pointXYLoSide.distanceTo(new InhomogeneousPoint3D(loX, loY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointYZLoSide),
                pointYZLoSide.distanceTo(new InhomogeneousPoint3D(centerX, loY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXZLoSide),
                pointXZLoSide.distanceTo(new InhomogeneousPoint3D(loX, centerY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(pointXYHiSide),
                pointXYHiSide.distanceTo(new InhomogeneousPoint3D(hiX, hiY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointYZHiSide),
                pointYZHiSide.distanceTo(new InhomogeneousPoint3D(centerX, hiY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXZHiSide),
                pointXZHiSide.distanceTo(new InhomogeneousPoint3D(hiX, centerY, hiZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(pointXYZLoSide),
                pointXYZLoSide.distanceTo(new InhomogeneousPoint3D(loX, loY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYZHiSide),
                pointXYZHiSide.distanceTo(new InhomogeneousPoint3D(hiX, hiY, hiZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(pointXYMixedSide1),
                pointXYMixedSide1.distanceTo(new InhomogeneousPoint3D(loX, hiY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYMixedSide2),
                pointXYMixedSide2.distanceTo(new InhomogeneousPoint3D(hiX, loY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointYZMixedSide1),
                pointYZMixedSide1.distanceTo(new InhomogeneousPoint3D(centerX, loY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointYZMixedSide2),
                pointYZMixedSide2.distanceTo(new InhomogeneousPoint3D(centerX, hiY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXZMixedSide1),
                pointXZMixedSide1.distanceTo(new InhomogeneousPoint3D(loX, centerY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXZMixedSide2),
                pointXZMixedSide2.distanceTo(new InhomogeneousPoint3D(hiX, centerY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(pointXYZMixedSide1),
                pointXYZMixedSide1.distanceTo(new InhomogeneousPoint3D(loX, loY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYZMixedSide2),
                pointXYZMixedSide2.distanceTo(new InhomogeneousPoint3D(loX, hiY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYZMixedSide3),
                pointXYZMixedSide3.distanceTo(new InhomogeneousPoint3D(loX, hiY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYZMixedSide4),
                pointXYZMixedSide4.distanceTo(new InhomogeneousPoint3D(hiX, loY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYZMixedSide5),
                pointXYZMixedSide5.distanceTo(new InhomogeneousPoint3D(hiX, loY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointXYZMixedSide6),
                pointXYZMixedSide6.distanceTo(new InhomogeneousPoint3D(hiX, hiY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getDistance(center), 0.0, 0.0);
    }

    @Test
    public void testGetSqrDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double centerX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double centerY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double centerZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double sizeX = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final double sizeY = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final double sizeZ = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final double loX = centerX - 0.5 * sizeX;
        final double loY = centerY - 0.5 * sizeY;
        final double loZ = centerZ - 0.5 * sizeZ;

        final double hiX = centerX + 0.5 * sizeX;
        final double hiY = centerY + 0.5 * sizeY;
        final double hiZ = centerZ + 0.5 * sizeZ;

        final InhomogeneousPoint3D lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final InhomogeneousPoint3D hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);

        final Point3D pointXLoSide = new InhomogeneousPoint3D(centerX -
                sizeX * randomizer.nextDouble(1.5, 2.5), centerY, centerZ);
        final Point3D pointYLoSide = new InhomogeneousPoint3D(centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5), centerZ);
        final Point3D pointZLoSide = new InhomogeneousPoint3D(centerX, centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXHiSide = new InhomogeneousPoint3D(centerX +
                sizeX * randomizer.nextDouble(1.5, 2.5), centerY, centerZ);
        final Point3D pointYHiSide = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5), centerZ);
        final Point3D pointZHiSide = new InhomogeneousPoint3D(centerX, centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointYZLoSide = new InhomogeneousPoint3D(centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointYZHiSide = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointXYMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final Point3D pointYZMixedSide1 = new InhomogeneousPoint3D(centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointYZMixedSide2 = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXZMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D pointXYZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide2 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide3 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide4 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide5 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final Point3D pointXYZMixedSide6 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final Point3D center = new InhomogeneousPoint3D(centerX, centerY, centerZ);

        final Box3D box = new Box3D(lo, hi);
        assertEquals(box.getSqrDistance(pointXLoSide),
                pointXLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, centerY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointYLoSide),
                pointYLoSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, loY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointZLoSide),
                pointZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, centerY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(pointXHiSide),
                pointXHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, centerY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointYHiSide),
                pointYHiSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, hiY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointZHiSide),
                pointZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, centerY, hiZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(pointXYLoSide),
                pointXYLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, loY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointYZLoSide),
                pointYZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, loY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXZLoSide),
                pointXZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, centerY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(pointXYHiSide),
                pointXYHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, hiY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointYZHiSide),
                pointYZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, hiY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXZHiSide),
                pointXZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, centerY, hiZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(pointXYZLoSide),
                pointXYZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, loY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYZHiSide),
                pointXYZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, hiY, hiZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(pointXYMixedSide1),
                pointXYMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(loX, hiY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYMixedSide2),
                pointXYMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(hiX, loY, centerZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointYZMixedSide1),
                pointYZMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(centerX, loY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointYZMixedSide2),
                pointYZMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(centerX, hiY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXZMixedSide1),
                pointXZMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(loX, centerY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXZMixedSide2),
                pointXZMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(hiX, centerY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(pointXYZMixedSide1),
                pointXYZMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(loX, loY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYZMixedSide2),
                pointXYZMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(loX, hiY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYZMixedSide3),
                pointXYZMixedSide3.sqrDistanceTo(new InhomogeneousPoint3D(loX, hiY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYZMixedSide4),
                pointXYZMixedSide4.sqrDistanceTo(new InhomogeneousPoint3D(hiX, loY, loZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYZMixedSide5),
                pointXYZMixedSide5.sqrDistanceTo(new InhomogeneousPoint3D(hiX, loY, hiZ)),
                ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointXYZMixedSide6),
                pointXYZMixedSide6.sqrDistanceTo(new InhomogeneousPoint3D(hiX, hiY, loZ)),
                ABSOLUTE_ERROR);

        assertEquals(box.getSqrDistance(center), 0.0, 0.0);
    }
}
