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

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class Box3DTest {

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        // test empty constructor
        var box = new Box3D();

        // check default values
        assertEquals(new InhomogeneousPoint3D(-0.5, -0.5, -0.5), box.getLo());
        assertEquals(new InhomogeneousPoint3D(0.5, 0.5, 0.5), box.getHi());

        // test constructor with lo and hi
        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);
        final var hiZ = randomizer.nextDouble(loZ, MAX_RANDOM_VALUE);

        final var lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final var hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);
        box = new Box3D(lo, hi);

        // check
        assertSame(lo, box.getLo());
        assertSame(hi, box.getHi());
    }

    @Test
    void testGetSetLo() {
        final var box = new Box3D();

        // check default value
        assertEquals(new InhomogeneousPoint3D(-0.5, -0.5, -0.5), box.getLo());

        // set new value
        final var lo = new InhomogeneousPoint3D();
        box.setLo(lo);

        // check
        assertSame(lo, box.getLo());
    }

    @Test
    void testGetSetHi() {
        final var box = new Box3D();

        // check default value
        assertEquals(new InhomogeneousPoint3D(0.5, 0.5, 0.5), box.getHi());

        // set new value
        final var hi = new InhomogeneousPoint3D();
        box.setHi(hi);

        // check
        assertSame(hi, box.getHi());
    }

    @Test
    void testSetBounds() {
        final var box = new Box3D();

        // check default values
        assertEquals(new InhomogeneousPoint3D(-0.5, -0.5, -0.5), box.getLo());
        assertEquals(new InhomogeneousPoint3D(0.5, 0.5, 0.5), box.getHi());

        // set bounds
        var lo = new InhomogeneousPoint3D();
        var hi = new InhomogeneousPoint3D();
        box.setBounds(lo, hi);

        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);
        final var hiZ = randomizer.nextDouble(loZ, MAX_RANDOM_VALUE);

        lo = new InhomogeneousPoint3D(loX, loY, loZ);
        hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);
        box.setBounds(loX, loY, loZ, hiX, hiY, hiZ);

        // check
        assertEquals(lo, box.getLo());
        assertEquals(hi, box.getHi());
    }

    @Test
    void testGetDistance() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var sizeX = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var sizeY = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var sizeZ = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var loX = centerX - 0.5 * sizeX;
        final var loY = centerY - 0.5 * sizeY;
        final var loZ = centerZ - 0.5 * sizeZ;

        final var hiX = centerX + 0.5 * sizeX;
        final var hiY = centerY + 0.5 * sizeY;
        final var hiZ = centerZ + 0.5 * sizeZ;

        final var lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final var hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);

        final var pointXLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ);
        final var pointYLoSide = new InhomogeneousPoint3D(
                centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointZLoSide = new InhomogeneousPoint3D(
                centerX,
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ);
        final var pointYHiSide = new InhomogeneousPoint3D(
                centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointZHiSide = new InhomogeneousPoint3D(
                centerX,
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointYZLoSide = new InhomogeneousPoint3D(
                centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointYZHiSide = new InhomogeneousPoint3D(centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointXYMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointYZMixedSide1 = new InhomogeneousPoint3D(
                centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointYZMixedSide2 = new InhomogeneousPoint3D(
                centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide2 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide3 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide4 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide5 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide6 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var center = new InhomogeneousPoint3D(centerX, centerY, centerZ);

        final var box = new Box3D(lo, hi);
        assertEquals(pointXLoSide.distanceTo(new InhomogeneousPoint3D(loX, centerY, centerZ)),
                box.getDistance(pointXLoSide), ABSOLUTE_ERROR);
        assertEquals(pointYLoSide.distanceTo(new InhomogeneousPoint3D(centerX, loY, centerZ)),
                box.getDistance(pointYLoSide), ABSOLUTE_ERROR);
        assertEquals(pointZLoSide.distanceTo(new InhomogeneousPoint3D(centerX, centerY, loZ)),
                box.getDistance(pointZLoSide), ABSOLUTE_ERROR);

        assertEquals(pointXHiSide.distanceTo(new InhomogeneousPoint3D(hiX, centerY, centerZ)),
                box.getDistance(pointXHiSide), ABSOLUTE_ERROR);
        assertEquals(pointYHiSide.distanceTo(new InhomogeneousPoint3D(centerX, hiY, centerZ)),
                box.getDistance(pointYHiSide), ABSOLUTE_ERROR);
        assertEquals(pointZHiSide.distanceTo(new InhomogeneousPoint3D(centerX, centerY, hiZ)),
                box.getDistance(pointZHiSide), ABSOLUTE_ERROR);

        assertEquals(pointXYLoSide.distanceTo(new InhomogeneousPoint3D(loX, loY, centerZ)),
                box.getDistance(pointXYLoSide), ABSOLUTE_ERROR);
        assertEquals(pointYZLoSide.distanceTo(new InhomogeneousPoint3D(centerX, loY, loZ)),
                box.getDistance(pointYZLoSide), ABSOLUTE_ERROR);
        assertEquals(pointXZLoSide.distanceTo(new InhomogeneousPoint3D(loX, centerY, loZ)),
                box.getDistance(pointXZLoSide), ABSOLUTE_ERROR);

        assertEquals(pointXYHiSide.distanceTo(new InhomogeneousPoint3D(hiX, hiY, centerZ)),
                box.getDistance(pointXYHiSide), ABSOLUTE_ERROR);
        assertEquals(pointYZHiSide.distanceTo(new InhomogeneousPoint3D(centerX, hiY, hiZ)),
                box.getDistance(pointYZHiSide), ABSOLUTE_ERROR);
        assertEquals(pointXZHiSide.distanceTo(new InhomogeneousPoint3D(hiX, centerY, hiZ)),
                box.getDistance(pointXZHiSide), ABSOLUTE_ERROR);

        assertEquals(pointXYZLoSide.distanceTo(new InhomogeneousPoint3D(loX, loY, loZ)),
                box.getDistance(pointXYZLoSide), ABSOLUTE_ERROR);
        assertEquals(pointXYZHiSide.distanceTo(new InhomogeneousPoint3D(hiX, hiY, hiZ)),
                box.getDistance(pointXYZHiSide), ABSOLUTE_ERROR);

        assertEquals(pointXYMixedSide1.distanceTo(new InhomogeneousPoint3D(loX, hiY, centerZ)),
                box.getDistance(pointXYMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointXYMixedSide2.distanceTo(new InhomogeneousPoint3D(hiX, loY, centerZ)),
                box.getDistance(pointXYMixedSide2), ABSOLUTE_ERROR);
        assertEquals(pointYZMixedSide1.distanceTo(new InhomogeneousPoint3D(centerX, loY, hiZ)),
                box.getDistance(pointYZMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointYZMixedSide2.distanceTo(new InhomogeneousPoint3D(centerX, hiY, loZ)),
                box.getDistance(pointYZMixedSide2), ABSOLUTE_ERROR);
        assertEquals(pointXZMixedSide1.distanceTo(new InhomogeneousPoint3D(loX, centerY, hiZ)),
                box.getDistance(pointXZMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointXZMixedSide2.distanceTo(new InhomogeneousPoint3D(hiX, centerY, loZ)),
                box.getDistance(pointXZMixedSide2), ABSOLUTE_ERROR);

        assertEquals(pointXYZMixedSide1.distanceTo(new InhomogeneousPoint3D(loX, loY, hiZ)),
                box.getDistance(pointXYZMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide2.distanceTo(new InhomogeneousPoint3D(loX, hiY, loZ)),
                box.getDistance(pointXYZMixedSide2), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide3.distanceTo(new InhomogeneousPoint3D(loX, hiY, hiZ)),
                box.getDistance(pointXYZMixedSide3), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide4.distanceTo(new InhomogeneousPoint3D(hiX, loY, loZ)),
                box.getDistance(pointXYZMixedSide4), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide5.distanceTo(new InhomogeneousPoint3D(hiX, loY, hiZ)),
                box.getDistance(pointXYZMixedSide5), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide6.distanceTo(new InhomogeneousPoint3D(hiX, hiY, loZ)),
                box.getDistance(pointXYZMixedSide6), ABSOLUTE_ERROR);

        assertEquals(0.0, box.getDistance(center), 0.0);
    }

    @Test
    void testGetSqrDistance() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var sizeX = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var sizeY = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var sizeZ = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var loX = centerX - 0.5 * sizeX;
        final var loY = centerY - 0.5 * sizeY;
        final var loZ = centerZ - 0.5 * sizeZ;

        final var hiX = centerX + 0.5 * sizeX;
        final var hiY = centerY + 0.5 * sizeY;
        final var hiZ = centerZ + 0.5 * sizeZ;

        final var lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final var hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);

        final var pointXLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ);
        final var pointYLoSide = new InhomogeneousPoint3D(
                centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointZLoSide = new InhomogeneousPoint3D(
                centerX,
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ);
        final var pointYHiSide = new InhomogeneousPoint3D(
                centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointZHiSide = new InhomogeneousPoint3D(
                centerX,
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointYZLoSide = new InhomogeneousPoint3D(
                centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointYZHiSide = new InhomogeneousPoint3D(
                centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYZLoSide = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZHiSide = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointXYMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ);
        final var pointYZMixedSide1 = new InhomogeneousPoint3D(
                centerX,
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointYZMixedSide2 = new InhomogeneousPoint3D(
                centerX,
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXZMixedSide2 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY,
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var pointXYZMixedSide1 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide2 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide3 = new InhomogeneousPoint3D(
                centerX - sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide4 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide5 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY - sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ + sizeZ * randomizer.nextDouble(1.5, 2.5));
        final var pointXYZMixedSide6 = new InhomogeneousPoint3D(
                centerX + sizeX * randomizer.nextDouble(1.5, 2.5),
                centerY + sizeY * randomizer.nextDouble(1.5, 2.5),
                centerZ - sizeZ * randomizer.nextDouble(1.5, 2.5));

        final var center = new InhomogeneousPoint3D(centerX, centerY, centerZ);

        final var box = new Box3D(lo, hi);
        assertEquals(pointXLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, centerY, centerZ)),
                box.getSqrDistance(pointXLoSide), ABSOLUTE_ERROR);
        assertEquals(pointYLoSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, loY, centerZ)),
                box.getSqrDistance(pointYLoSide), ABSOLUTE_ERROR);
        assertEquals(pointZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, centerY, loZ)),
                box.getSqrDistance(pointZLoSide), ABSOLUTE_ERROR);

        assertEquals(pointXHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, centerY, centerZ)),
                box.getSqrDistance(pointXHiSide), ABSOLUTE_ERROR);
        assertEquals(pointYHiSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, hiY, centerZ)),
                box.getSqrDistance(pointYHiSide), ABSOLUTE_ERROR);
        assertEquals(pointZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, centerY, hiZ)),
                box.getSqrDistance(pointZHiSide), ABSOLUTE_ERROR);

        assertEquals(pointXYLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, loY, centerZ)),
                box.getSqrDistance(pointXYLoSide), ABSOLUTE_ERROR);
        assertEquals(pointYZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, loY, loZ)),
                box.getSqrDistance(pointYZLoSide), ABSOLUTE_ERROR);
        assertEquals(pointXZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, centerY, loZ)),
                box.getSqrDistance(pointXZLoSide), ABSOLUTE_ERROR);

        assertEquals(pointXYHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, hiY, centerZ)),
                box.getSqrDistance(pointXYHiSide), ABSOLUTE_ERROR);
        assertEquals(pointYZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(centerX, hiY, hiZ)),
                box.getSqrDistance(pointYZHiSide), ABSOLUTE_ERROR);
        assertEquals(pointXZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, centerY, hiZ)),
                box.getSqrDistance(pointXZHiSide), ABSOLUTE_ERROR);

        assertEquals(pointXYZLoSide.sqrDistanceTo(new InhomogeneousPoint3D(loX, loY, loZ)),
                box.getSqrDistance(pointXYZLoSide), ABSOLUTE_ERROR);
        assertEquals(pointXYZHiSide.sqrDistanceTo(new InhomogeneousPoint3D(hiX, hiY, hiZ)),
                box.getSqrDistance(pointXYZHiSide), ABSOLUTE_ERROR);

        assertEquals(pointXYMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(loX, hiY, centerZ)),
                box.getSqrDistance(pointXYMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointXYMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(hiX, loY, centerZ)),
                box.getSqrDistance(pointXYMixedSide2), ABSOLUTE_ERROR);
        assertEquals(pointYZMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(centerX, loY, hiZ)),
                box.getSqrDistance(pointYZMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointYZMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(centerX, hiY, loZ)),
                box.getSqrDistance(pointYZMixedSide2), ABSOLUTE_ERROR);
        assertEquals(pointXZMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(loX, centerY, hiZ)),
                box.getSqrDistance(pointXZMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointXZMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(hiX, centerY, loZ)),
                box.getSqrDistance(pointXZMixedSide2), ABSOLUTE_ERROR);

        assertEquals(pointXYZMixedSide1.sqrDistanceTo(new InhomogeneousPoint3D(loX, loY, hiZ)),
                box.getSqrDistance(pointXYZMixedSide1), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide2.sqrDistanceTo(new InhomogeneousPoint3D(loX, hiY, loZ)),
                box.getSqrDistance(pointXYZMixedSide2), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide3.sqrDistanceTo(new InhomogeneousPoint3D(loX, hiY, hiZ)),
                box.getSqrDistance(pointXYZMixedSide3), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide4.sqrDistanceTo(new InhomogeneousPoint3D(hiX, loY, loZ)),
                box.getSqrDistance(pointXYZMixedSide4), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide5.sqrDistanceTo(new InhomogeneousPoint3D(hiX, loY, hiZ)),
                box.getSqrDistance(pointXYZMixedSide5), ABSOLUTE_ERROR);
        assertEquals(pointXYZMixedSide6.sqrDistanceTo(new InhomogeneousPoint3D(hiX, hiY, loZ)),
                box.getSqrDistance(pointXYZMixedSide6), ABSOLUTE_ERROR);

        assertEquals(0.0, box.getSqrDistance(center), 0.0);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);
        final var hiZ = randomizer.nextDouble(loZ, MAX_RANDOM_VALUE);

        final var lo = new InhomogeneousPoint3D(loX, loY, loZ);
        final var hi = new InhomogeneousPoint3D(hiX, hiY, hiZ);
        final var box1 = new Box3D(lo, hi);

        // check
        assertSame(box1.getLo(), lo);
        assertSame(box1.getHi(), hi);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(box1);
        final var box2 = SerializationHelper.<Box3D>deserialize(bytes);

        // check
        assertNotSame(box1, box2);
        assertEquals(box1.getLo(), box2.getLo());
        assertNotSame(box1.getLo(), box2.getLo());
        assertEquals(box1.getHi(), box2.getHi());
        assertNotSame(box1.getHi(), box2.getHi());
    }
}
