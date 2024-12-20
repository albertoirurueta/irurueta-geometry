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

import java.io.Serializable;

/**
 * Abstract representation of a box for a point in an n-dimensional representation.
 *
 * @param <P> a point implementation.
 */
public abstract class Box<P extends Point<P>> implements Serializable {

    /**
     * Low coordinate values.
     */
    protected P lo;

    /**
     * High coordinate values.
     */
    protected P hi;

    /**
     * Empty constructor.
     */
    protected Box() {
    }

    /**
     * Constructor.
     *
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    protected Box(final P lo, final P hi) {
        internalSetBounds(lo, hi);
    }

    /**
     * Gets low coordinate values.
     *
     * @return low coordinate values.
     */
    public P getLo() {
        return lo;
    }

    /**
     * Sets low coordinate values.
     *
     * @param lo low coordinate values.
     */
    public void setLo(final P lo) {
        this.lo = lo;
    }

    /**
     * Gets high coordinate values.
     *
     * @return high coordinate values.
     */
    public P getHi() {
        return hi;
    }

    /**
     * Sets high coordinate values.
     *
     * @param hi high coordinate values.
     */
    public void setHi(final P hi) {
        this.hi = hi;
    }

    /**
     * Sets boundaries.
     *
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    public void setBounds(final P lo, final P hi) {
        internalSetBounds(lo, hi);
    }

    /**
     * Gets square distance of provided point to the boundaries of this box or zero if the point is
     * inside this box.
     *
     * @param point point to be checked.
     * @return distance of provided point.
     */
    public double getSqrDistance(final P point) {
        final var dim = point.getDimensions();

        var dd = 0.0;
        for (var i = 0; i < dim; i++) {
            final var pointValue = point.getInhomogeneousCoordinate(i);
            final var loValue = lo.getInhomogeneousCoordinate(i);
            final var hiValue = hi.getInhomogeneousCoordinate(i);

            if (pointValue < loValue) {
                dd += sqr(pointValue - loValue);
            }
            if (pointValue > hiValue) {
                dd += sqr(pointValue - hiValue);
            }
        }
        return dd;
    }

    /**
     * Gets distance of provided point to the boundaries of this box or zero if the point is
     * inside this box.
     *
     * @param point point to be checked.
     * @return distance of provided point.
     */
    public double getDistance(final P point) {
        return Math.sqrt(getSqrDistance(point));
    }

    /**
     * Returns the squared value.
     *
     * @param value value to be squared.
     * @return squared value.
     */
    private double sqr(final double value) {
        return value * value;
    }

    /**
     * Internally sets boundaries.
     *
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    private void internalSetBounds(final P lo, final P hi) {
        this.lo = lo;
        this.hi = hi;
    }
}
