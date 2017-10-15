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

import java.util.Random;

import static org.junit.Assert.*;

public class Box2DTest {

    public static final double ABSOLUTE_ERROR = 1e-9;
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;

    public Box2DTest() { }

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
        //test empty constructor
        Box2D box = new Box2D();

        //check default values
        assertEquals(box.getLo(), new InhomogeneousPoint2D(-0.5, -0.5));
        assertEquals(box.getHi(), new InhomogeneousPoint2D(0.5, 0.5));

        //test constructor with lo and hi
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        double hiY =  randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        InhomogeneousPoint2D lo = new InhomogeneousPoint2D(loX, loY);
        InhomogeneousPoint2D hi = new InhomogeneousPoint2D(hiX, hiY);
        box = new Box2D(lo, hi);

        //check
        assertSame(box.getLo(), lo);
        assertSame(box.getHi(), hi);

        //test constructor with rectangle
        InhomogeneousPoint2D topLeft = new InhomogeneousPoint2D(loX, hiY);
        InhomogeneousPoint2D bottomRight = new InhomogeneousPoint2D(hiX, loY);
        Rectangle rectangle = new Rectangle(topLeft, bottomRight);
        box = new Box2D(rectangle);

        //check
        assertEquals(box.getLo(), lo);
        assertEquals(box.getHi(), hi);
    }

    @Test
    public void testGetSetLo() {
        Box2D box = new Box2D();

        //check default value
        assertEquals(box.getLo(), new InhomogeneousPoint2D(-0.5, -0.5));

        //set new value
        InhomogeneousPoint2D lo = new InhomogeneousPoint2D();
        box.setLo(lo);

        //check
        assertSame(box.getLo(), lo);
    }

    @Test
    public void testGetSetHi() {
        Box2D box = new Box2D();

        //check default value
        assertEquals(box.getHi(), new InhomogeneousPoint2D(0.5, 0.5));

        //set new value
        InhomogeneousPoint2D hi = new InhomogeneousPoint2D();
        box.setHi(hi);

        //check
        assertSame(box.getHi(), hi);
    }

    @Test
    public void testSetBounds() {
        Box2D box = new Box2D();

        //check default values
        assertEquals(box.getLo(), new InhomogeneousPoint2D(-0.5, -0.5));
        assertEquals(box.getHi(), new InhomogeneousPoint2D(0.5, 0.5));

        //set bounds
        InhomogeneousPoint2D lo = new InhomogeneousPoint2D();
        InhomogeneousPoint2D hi = new InhomogeneousPoint2D();
        box.setBounds(lo, hi);

        //check
        assertSame(box.getLo(), lo);
        assertSame(box.getHi(), hi);

        //random values
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        double hiY =  randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        lo = new InhomogeneousPoint2D(loX, loY);
        hi = new InhomogeneousPoint2D(hiX, hiY);
        box.setBounds(loX, loY, hiX, hiY);

        //check
        assertEquals(box.getLo(), lo);
        assertEquals(box.getHi(), hi);
    }

    @Test
    public void testFromToRectangle() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double loX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double loY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double hiX = randomizer.nextDouble(loX, MAX_RANDOM_VALUE);
        double hiY =  randomizer.nextDouble(loY, MAX_RANDOM_VALUE);

        InhomogeneousPoint2D lo = new InhomogeneousPoint2D(loX, loY);
        InhomogeneousPoint2D hi = new InhomogeneousPoint2D(hiX, hiY);

        InhomogeneousPoint2D topLeft = new InhomogeneousPoint2D(loX, hiY);
        InhomogeneousPoint2D bottomRight = new InhomogeneousPoint2D(hiX, loY);
        Rectangle rectangle = new Rectangle(topLeft, bottomRight);

        Box2D box = new Box2D();

        //from rectangle
        box.fromRectangle(rectangle);

        //check
        assertEquals(box.getLo(), lo);
        assertEquals(box.getHi(), hi);

        //to rectangle
        Rectangle rectangle2 = box.toRectangle();
        Rectangle rectangle3 = new Rectangle();
        box.toRectangle(rectangle3);

        //check
        assertEquals(rectangle2.getTopLeft(), topLeft);
        assertEquals(rectangle2.getBottomRight(), bottomRight);
        assertEquals(rectangle3.getTopLeft(), topLeft);
        assertEquals(rectangle3.getBottomRight(), bottomRight);
    }

    @Test
    public void testGetDistance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double centerX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double centerY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        double height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        double left = centerX - 0.5 * width;
        double right = centerX + 0.5 * width;
        double top = centerY + 0.5 * height;
        double bottom = centerY - 0.5 * height;

        Point2D topLeft = new InhomogeneousPoint2D(left, top);
        Point2D bottomRight = new InhomogeneousPoint2D(right, bottom);
        Rectangle rectangle = new Rectangle(topLeft, bottomRight);

        Point2D pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width*randomizer.nextDouble(1.5, 2.5),
                centerY);
        Point2D pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width*randomizer.nextDouble(1.5, 2.5),
                centerY + height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtTopSide = new InhomogeneousPoint2D(
                centerX,
                centerY + height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(1.5, 2.5),
                centerY + height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(1.5, 2.5),
                centerY);
        Point2D pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(1.5, 2.5),
                centerY - height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtBottomSide = new InhomogeneousPoint2D(
                centerX,
                centerY - height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width*randomizer.nextDouble(1.5, 2.5),
                centerY - height*randomizer.nextDouble(1.5, 2.5));
        Point2D insidePoint = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(-0.5, 0.5),
                centerY + height*randomizer.nextDouble(-0.5, 0.5));


        Box2D box = new Box2D(rectangle);
        assertEquals(box.getDistance(pointAtLeftSide),
                rectangle.getDistance(pointAtLeftSide), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtTopLeftCorner),
                rectangle.getDistance(pointAtTopLeftCorner), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtTopSide),
                rectangle.getDistance(pointAtTopSide), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtTopRightCorner),
                rectangle.getDistance(pointAtTopRightCorner), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtRightSide),
                rectangle.getDistance(pointAtRightSide), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtBottomRightCorner),
                rectangle.getDistance(pointAtBottomRightCorner), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtBottomSide),
                rectangle.getDistance(pointAtBottomSide), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(pointAtBottomLeftCorner),
                rectangle.getDistance(pointAtBottomLeftCorner), ABSOLUTE_ERROR);
        assertEquals(box.getDistance(insidePoint), 0.0, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSqrDistance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double centerX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double centerY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        double width = randomizer.nextDouble(0, MAX_RANDOM_VALUE);
        double height = randomizer.nextDouble(0, MAX_RANDOM_VALUE);

        double left = centerX - 0.5 * width;
        double right = centerX + 0.5 * width;
        double top = centerY + 0.5 * height;
        double bottom = centerY - 0.5 * height;

        Point2D topLeft = new InhomogeneousPoint2D(left, top);
        Point2D bottomRight = new InhomogeneousPoint2D(right, bottom);
        Rectangle rectangle = new Rectangle(topLeft, bottomRight);

        Point2D pointAtLeftSide = new InhomogeneousPoint2D(
                centerX - width*randomizer.nextDouble(1.5, 2.5),
                centerY);
        Point2D pointAtTopLeftCorner = new InhomogeneousPoint2D(
                centerX - width*randomizer.nextDouble(1.5, 2.5),
                centerY + height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtTopSide = new InhomogeneousPoint2D(
                centerX,
                centerY + height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtTopRightCorner = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(1.5, 2.5),
                centerY + height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtRightSide = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(1.5, 2.5),
                centerY);
        Point2D pointAtBottomRightCorner = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(1.5, 2.5),
                centerY - height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtBottomSide = new InhomogeneousPoint2D(
                centerX,
                centerY - height*randomizer.nextDouble(1.5, 2.5));
        Point2D pointAtBottomLeftCorner = new InhomogeneousPoint2D(
                centerX - width*randomizer.nextDouble(1.5, 2.5),
                centerY - height*randomizer.nextDouble(1.5, 2.5));
        Point2D insidePoint = new InhomogeneousPoint2D(
                centerX + width*randomizer.nextDouble(-0.5, 0.5),
                centerY + height*randomizer.nextDouble(-0.5, 0.5));


        Box2D box = new Box2D(rectangle);
        assertEquals(box.getSqrDistance(pointAtLeftSide),
                Math.pow(rectangle.getDistance(pointAtLeftSide), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtTopLeftCorner),
                Math.pow(rectangle.getDistance(pointAtTopLeftCorner), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtTopSide),
                Math.pow(rectangle.getDistance(pointAtTopSide), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtTopRightCorner),
                Math.pow(rectangle.getDistance(pointAtTopRightCorner), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtRightSide),
                Math.pow(rectangle.getDistance(pointAtRightSide), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtBottomRightCorner),
                Math.pow(rectangle.getDistance(pointAtBottomRightCorner), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtBottomSide),
                Math.pow(rectangle.getDistance(pointAtBottomSide), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(pointAtBottomLeftCorner),
                Math.pow(rectangle.getDistance(pointAtBottomLeftCorner), 2.0), ABSOLUTE_ERROR);
        assertEquals(box.getSqrDistance(insidePoint), 0.0, ABSOLUTE_ERROR);
    }
}
