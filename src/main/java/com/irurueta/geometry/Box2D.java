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

/**
 * This class defines a 2D rectangular area aligned with the horizontal and vertical axes.
 * A box is defined by two corners, low, containing minimum coordinate values, and high,
 * containing maximum coordinate values.
 * This class is used internally by KD trees and quad trees.
 */
public class Box2D extends Box<Point2D> {

    /**
     * Empty constructor.
     * Creates a box centered at the origin and having unitary area.
     */
    public Box2D() {
        super();
        mLo = new InhomogeneousPoint2D(-0.5, -0.5);
        mHi = new InhomogeneousPoint2D(0.5, 0.5);
    }

    /**
     * Constructor.
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    public Box2D(Point2D lo, Point2D hi) {
        super(lo, hi);
    }

    /**
     * Constructor from rectangle.
     * @param rectangle a rectangle.
     */
    public Box2D(Rectangle rectangle) {
        fromRectangle(rectangle);
    }

    /**
     * Sets boundaries.
     * @param loX horizontal low coordinate.
     * @param loY vertical low coordinate.
     * @param hiX horizontal high coordinate.
     * @param hiY vertical high coordinate.
     */
    public final void setBounds(double loX, double loY, double hiX, double hiY) {
        setBounds(new InhomogeneousPoint2D(loX, loY),
                new InhomogeneousPoint2D(hiX, hiY));
    }

    /**
     * Sets values of this box from provided rectangle.
     * @param rectangle a rectangle containing boundaries.
     */
    public final void fromRectangle(Rectangle rectangle) {
        Point2D topLeft = rectangle.getTopLeft();
        Point2D bottomRight = rectangle.getBottomRight();

        double loX = topLeft.getInhomX();
        double loY = bottomRight.getInhomY();

        double hiX = bottomRight.getInhomX();
        double hiY = topLeft.getInhomY();

        mLo = new InhomogeneousPoint2D(loX, loY);
        mHi = new InhomogeneousPoint2D(hiX, hiY);
    }

    /**
     * Creates a rectangle equivalent to this box.
     * @return a rectangle equivalent to this box.
     */
    public Rectangle toRectangle() {
        Rectangle result = new Rectangle();
        toRectangle(result);
        return result;
    }

    /**
     * Sets values into provided rectangle to make it equivalent to this box.
     * @param result instance where values will be stored.
     */
    public void toRectangle(Rectangle result) {
        double left = mLo.getInhomX();
        double right = mHi.getInhomX();

        double bottom = mLo.getInhomY();
        double top = mHi.getInhomY();

        result.setBounds(left, top, right, bottom);
    }
}
