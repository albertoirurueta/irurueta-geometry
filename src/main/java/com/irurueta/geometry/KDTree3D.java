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

import java.util.Collection;

/**
 * Implementation of a k-D tree in 3D.
 * Once a K-D tree is built for a collection of points, it can later be used to efficiently do certain operations
 * such as point locaiton, nearest points searches, etc.
 */
public class KDTree3D extends KDTree<Point3D> {

    /**
     * Constructor.
     * @param pts collection of points to store in the tree.
     */
    public KDTree3D(final Collection<Point3D> pts) {
        super(pts, Point3D.class);
    }

    /**
     * Gets number of dimensions supported by this k-D tree implementation on provided list of points.
     * @return number of dimensions.
     */
    @Override
    public int getDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Creates a point.
     * @param value value to be set on point coordinates.
     * @return created point.
     */
    @Override
    protected Point3D createPoint(final double value) {
        return new InhomogeneousPoint3D(value, value, value);
    }

    /**
     * Copies a point.
     * @param point point to be copied.
     * @return copied point.
     */
    @Override
    protected Point3D copyPoint(final Point3D point) {
        return new InhomogeneousPoint3D(point);
    }
}
