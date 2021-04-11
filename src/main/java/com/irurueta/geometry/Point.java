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
 * Interface defining a point (either in 2D or 3D, or any other further dimensions that might be defined).
 */
public interface Point<P extends Point<P>> extends Serializable {

    /**
     * Returns number of dimensions of this point implementation.
     *
     * @return number of dimensions.
     */
    int getDimensions();

    /**
     * Gets value of inhomogeneous coordinate for provided dimension.
     *
     * @param dim dimension to retrieve coordinate for (i.e. 0 means x, 1 means y, 2 means z, etc).
     * @return value of inhomogeneous coordinate.
     * @throws IllegalArgumentException if provided dimension value is negative or exceeds number of dimensions.
     */
    double getInhomogeneousCoordinate(final int dim);

    /**
     * Sets value of inhomogeneous coordinate for provided dimension.
     *
     * @param dim   dimension to set coordinate for (i.e. 0 means x, 1 means y, 2 means z, etc).
     * @param value value to be set.
     * @throws IllegalArgumentException if provided dimension value is negative or exceeds number of dimensions.
     */
    void setInhomogeneousCoordinate(final int dim, final double value);

    /**
     * Returns euclidean distance between this point and provided point.
     *
     * @param point point to compare.
     * @return euclidean distance between this point and provided point.
     */
    double distanceTo(final P point);

    /**
     * Returns squared euclidean distance between this point and provided point.
     *
     * @param point point to compare.
     * @return euclidean distance between this point and provided point.
     */
    double sqrDistanceTo(final P point);
}
