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
 * @param <P> a point implementation.
 */
public abstract class Box<P extends Point> implements Serializable {

    /**
     * Low coordinate values.
     */
    protected P mLo;

    /**
     * High coordinate values.
     */
    protected P mHi;

    /**
     * Empty constructor.
     */
    protected Box() { }

    /**
     * Constructor.
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    public Box(P lo, P hi) {
        setBounds(lo, hi);
    }

    /**
     * Gets low coordinate values.
     * @return low coordinate values.
     */
    public P getLo() {
        return mLo;
    }

    /**
     * Sets low coordinate values.
     * @param lo low coordinate values.
     */
    public void setLo(P lo) {
        mLo = lo;
    }

    /**
     * Gets high coordinate values.
     * @return high coordinate values.
     */
    public P getHi() {
        return mHi;
    }

    /**
     * Sets high coordinate values.
     * @param hi high coordinate values.
     */
    public void setHi(P hi) {
        mHi = hi;
    }

    /**
     * Sets boundaries.
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    public final void setBounds(P lo, P hi) {
        mLo = lo;
        mHi = hi;
    }

    /**
     * Gets square distance of provided point to the boundaries of this box or zero if the point is
     * inside this box.
     * @param point point to be checked.
     * @return distance of provided point.
     */
    public double getSqrDistance(P point) {
        int dim = point.getDimensions();

        double dd = 0.0;
        double pointValue, loValue, hiValue;
        for (int i = 0; i < dim; i++) {
            pointValue = point.getInhomogeneousCoordinate(i);
            loValue = mLo.getInhomogeneousCoordinate(i);
            hiValue = mHi.getInhomogeneousCoordinate(i);

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
     * @param point point to be checked.
     * @return distance of provided point.
     */
    public double getDistance(P point) {
        return Math.sqrt(getSqrDistance(point));
    }

    /**
     * Returns the squared value.
     * @param value value to be squared.
     * @return squared value.
     */
    private double sqr(double value) {
        return value * value;
    }
}
