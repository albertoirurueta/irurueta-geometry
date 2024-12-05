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

class Box2DTest {

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        // test empty constructor
        var box = new Box2D();

        // check default values
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), box.getLo());
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), box.getHi());

        // test constructor with lo and hi
        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        final var lo = new InhomogeneousPoint2D(loX, loY);
        final var hi = new InhomogeneousPoint2D(hiX, hiY);
        box = new Box2D(lo, hi);

        // check
        assertSame(lo, box.getLo());
        assertSame(hi, box.getHi());

        // test constructor with rectangle
        final var topLeft = new InhomogeneousPoint2D(loX, hiY);
        final var bottomRight = new InhomogeneousPoint2D(hiX, loY);
        final var rectangle = new Rectangle(topLeft, bottomRight);
        box = new Box2D(rectangle);

        // check
        assertEquals(lo, box.getLo());
        assertEquals(hi, box.getHi());
    }

    @Test
    void testGetSetLo() {
        final var box = new Box2D();

        // check default value
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), box.getLo());

        // set new value
        final var lo = new InhomogeneousPoint2D();
        box.setLo(lo);

        // check
        assertSame(lo, box.getLo());
    }

    @Test
    void testGetSetHi() {
        final var box = new Box2D();

        // check default value
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), box.getHi());

        // set new value
        final var hi = new InhomogeneousPoint2D();
        box.setHi(hi);

        // check
        assertSame(hi, box.getHi());
    }

    @Test
    void testSetBounds() {
        final var box = new Box2D();

        // check default values
        assertEquals(new InhomogeneousPoint2D(-0.5, -0.5), box.getLo());
        assertEquals(new InhomogeneousPoint2D(0.5, 0.5), box.getHi());

        // set bounds
        var lo = new InhomogeneousPoint2D();
        var hi = new InhomogeneousPoint2D();
        box.setBounds(lo, hi);

        // check
        assertSame(lo, box.getLo());
        assertSame(hi, box.getHi());

        // random values
        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        lo = new InhomogeneousPoint2D(loX, loY);
        hi = new InhomogeneousPoint2D(hiX, hiY);
        box.setBounds(loX, loY, hiX, hiY);

        // check
        assertEquals(lo, box.getLo());
        assertEquals(hi, box.getHi());
    }

    @Test
    void testFromToRectangle() {
        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        final var lo = new InhomogeneousPoint2D(loX, loY);
        final var hi = new InhomogeneousPoint2D(hiX, hiY);

        final var topLeft = new InhomogeneousPoint2D(loX, hiY);
        final var bottomRight = new InhomogeneousPoint2D(hiX, loY);
        final var rectangle = new Rectangle(topLeft, bottomRight);

        final var box = new Box2D();

        // from rectangle
        box.fromRectangle(rectangle);

        // check
        assertEquals(lo, box.getLo());
        assertEquals(hi, box.getHi());

        // to rectangle
        final var rectangle2 = box.toRectangle();
        final var rectangle3 = new Rectangle();
        box.toRectangle(rectangle3);

        // check
        assertEquals(topLeft, rectangle2.getTopLeft());
        assertEquals(bottomRight, rectangle2.getBottomRight());
        assertEquals(topLeft, rectangle3.getTopLeft());
        assertEquals(bottomRight, rectangle3.getBottomRight());
    }

    @Test
    void testGetDistance() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var rectangle = new Rectangle(topLeft, bottomRight);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX,
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));


        final var box = new Box2D(rectangle);
        assertEquals(rectangle.getDistance(pointAtLeftSide), box.getDistance(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtTopLeftCorner), box.getDistance(pointAtTopLeftCorner),
                ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtTopSide), box.getDistance(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtTopRightCorner), box.getDistance(pointAtTopRightCorner),
                ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtRightSide), box.getDistance(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtBottomRightCorner), box.getDistance(pointAtBottomRightCorner),
                ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtBottomSide), box.getDistance(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(rectangle.getDistance(pointAtBottomLeftCorner), box.getDistance(pointAtBottomLeftCorner),
                ABSOLUTE_ERROR);
        assertEquals(0.0, box.getDistance(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSqrDistance() {
        final var randomizer = new UniformRandomizer();
        final var centerX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var centerY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        final var height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        final var left = centerX - 0.5 * width;
        final var right = centerX + 0.5 * width;
        final var top = centerY + 0.5 * height;
        final var bottom = centerY - 0.5 * height;

        final var topLeft = new InhomogeneousPoint2D(left, top);
        final var bottomRight = new InhomogeneousPoint2D(right, bottom);
        final var rectangle = new Rectangle(topLeft, bottomRight);

        final var pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopSide = new InhomogeneousPoint2D(
                centerX,
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY + height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY);
        final var pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomSide = new InhomogeneousPoint2D(
                centerX,
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width * randomizer.nextDouble(1.5, 2.5),
                centerY - height * randomizer.nextDouble(1.5, 2.5));
        final var insidePoint = new InhomogeneousPoint2D(
                centerX + width * randomizer.nextDouble(-0.5, 0.5),
                centerY + height * randomizer.nextDouble(-0.5, 0.5));

        final var box = new Box2D(rectangle);
        assertEquals(Math.pow(rectangle.getDistance(pointAtLeftSide), 2.0), box.getSqrDistance(pointAtLeftSide),
                ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtTopLeftCorner), 2.0),
                box.getSqrDistance(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtTopSide), 2.0), box.getSqrDistance(pointAtTopSide),
                ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtTopRightCorner), 2.0),
                box.getSqrDistance(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtRightSide), 2.0), box.getSqrDistance(pointAtRightSide),
                ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtBottomRightCorner), 2.0),
                box.getSqrDistance(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtBottomSide), 2.0), box.getSqrDistance(pointAtBottomSide),
                ABSOLUTE_ERROR);
        assertEquals(Math.pow(rectangle.getDistance(pointAtBottomLeftCorner), 2.0),
                box.getSqrDistance(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(0.0, box.getSqrDistance(insidePoint), ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        final var hiY = randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        final var lo = new InhomogeneousPoint2D(loX, loY);
        final var hi = new InhomogeneousPoint2D(hiX, hiY);
        final var box1 = new Box2D(lo, hi);

        // check
        assertSame(box1.getLo(), lo);
        assertSame(box1.getHi(), hi);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(box1);
        final var box2 = SerializationHelper.<Box2D>deserialize(bytes);

        // check
        assertNotSame(box1, box2);
        assertEquals(box1.getLo(), box2.getLo());
        assertNotSame(box1.getLo(), box2.getLo());
        assertEquals(box1.getHi(), box2.getHi());
        assertNotSame(box1.getHi(), box2.getHi());
    }
}
