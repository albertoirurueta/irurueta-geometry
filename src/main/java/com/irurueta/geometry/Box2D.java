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
        lo = new InhomogeneousPoint2D(-0.5, -0.5);
        hi = new InhomogeneousPoint2D(0.5, 0.5);
    }

    /**
     * Constructor.
     *
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    public Box2D(final Point2D lo, final Point2D hi) {
        super(lo, hi);
    }

    /**
     * Constructor from rectangle.
     *
     * @param rectangle a rectangle.
     */
    public Box2D(final Rectangle rectangle) {
        fromRectangle(rectangle);
    }

    /**
     * Sets boundaries.
     *
     * @param loX horizontal low coordinate.
     * @param loY vertical low coordinate.
     * @param hiX horizontal high coordinate.
     * @param hiY vertical high coordinate.
     */
    public final void setBounds(final double loX, final double loY, final double hiX, final double hiY) {
        setBounds(new InhomogeneousPoint2D(loX, loY), new InhomogeneousPoint2D(hiX, hiY));
    }

    /**
     * Sets values of this box from provided rectangle.
     *
     * @param rectangle a rectangle containing boundaries.
     */
    public final void fromRectangle(final Rectangle rectangle) {
        final var topLeft = rectangle.getTopLeft();
        final var bottomRight = rectangle.getBottomRight();

        final var loX = topLeft.getInhomX();
        final var loY = bottomRight.getInhomY();

        final var hiX = bottomRight.getInhomX();
        final var hiY = topLeft.getInhomY();

        lo = new InhomogeneousPoint2D(loX, loY);
        hi = new InhomogeneousPoint2D(hiX, hiY);
    }

    /**
     * Creates a rectangle equivalent to this box.
     *
     * @return a rectangle equivalent to this box.
     */
    public Rectangle toRectangle() {
        final var result = new Rectangle();
        toRectangle(result);
        return result;
    }

    /**
     * Sets values into provided rectangle to make it equivalent to this box.
     *
     * @param result instance where values will be stored.
     */
    public void toRectangle(final Rectangle result) {
        final var left = lo.getInhomX();
        final var right = hi.getInhomX();

        final var bottom = lo.getInhomY();
        final var top = hi.getInhomY();

        result.setBounds(left, top, right, bottom);
    }
}
