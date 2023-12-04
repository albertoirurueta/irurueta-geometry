/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
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

public class UtilsTest {

    private static final double MIN_RADIANS = -Math.PI / 2.0;
    private static final double MAX_RADIANS = Math.PI / 2.0;

    private static final double MIN_DEGREES = -180.0;
    private static final double MAX_DEGREES = 180.0;

    private static final int MIN_TURNS = 500;
    private static final int MAX_TURNS = 10000;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testConstants() {
        assertEquals(180.0, Utils.HALF_TURN_DEGREES, 0.0);
        assertEquals(Math.PI, Utils.HALF_TURN_RADIANS, 0.0);
    }

    @Test
    public void testConvertToDegrees() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double radians = randomizer.nextDouble(MIN_RADIANS, MAX_RADIANS);
        final double degrees = radians * 180.0 / Math.PI;

        assertEquals(degrees, Utils.convertToDegrees(radians), ABSOLUTE_ERROR);
    }

    @Test
    public void testConvertToRadians() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double degrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
        final double radians = degrees * Math.PI / 180.0;

        assertEquals(radians, Utils.convertToRadians(degrees), ABSOLUTE_ERROR);
    }

    @Test
    public void testNormalizeAngleDegrees() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double degrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
        final int turns = (randomizer.nextBoolean() ? 1 : -1) *
                randomizer.nextInt(MIN_TURNS, MAX_TURNS);
        final double degrees2 = degrees +
                turns * 360.0;

        final double degrees3 = Utils.normalizeAngleDegrees(degrees2);
        final double degrees4 = Utils.normalizeAngleDegrees(degrees);

        // check correctness
        assertEquals(degrees, degrees3, ABSOLUTE_ERROR);
        assertEquals(degrees, degrees4, ABSOLUTE_ERROR);

        double degrees5 = degrees2;
        while (degrees5 <= -180.0) {
            degrees5 += 2.0 * 180.0;
        }
        while (degrees5 > 180.0) {
            degrees5 -= 2.0 * 180.0;
        }

        assertEquals(degrees, degrees5, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testNormalizeAngleRadians() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double radians = randomizer.nextDouble(MIN_RADIANS, MAX_RADIANS);
        final int turns = (randomizer.nextBoolean() ? 1 : -1) *
                randomizer.nextInt(MIN_TURNS, MAX_TURNS);
        final double radians2 = radians +
                turns * 2.0 * Math.PI;

        final double radians3 = Utils.normalizeAngleRadians(radians2);
        final double radians4 = Utils.normalizeAngleRadians(radians);

        // check correctness
        assertEquals(radians, radians3, ABSOLUTE_ERROR);
        assertEquals(radians, radians4, ABSOLUTE_ERROR);

        double radians5 = radians2;
        while (radians5 <= -Math.PI) {
            radians5 += 2.0 * Math.PI;
        }
        while (radians5 > Math.PI) {
            radians5 -= 2.0 * Math.PI;
        }

        assertEquals(radians, radians5, LARGE_ABSOLUTE_ERROR);
    }
}
