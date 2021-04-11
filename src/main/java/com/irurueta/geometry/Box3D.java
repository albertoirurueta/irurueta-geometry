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
 * This class defines a 3D box area aligned with x, y, z axes.
 * A box is defined by two corners, low, containing minimum coordinate values, and high,
 * containing maximum coordinate values.
 * This class is used internally by KD trees and quad trees.
 */
public class Box3D extends Box<Point3D> {

    /**
     * Empty constructor.
     * Creates a box centered at the origin and having unitary volume.
     */
    public Box3D() {
        super();
        mLo = new InhomogeneousPoint3D(-0.5, -0.5, -0.5);
        mHi = new InhomogeneousPoint3D(0.5, 0.5, 0.5);
    }

    /**
     * Constructor.
     *
     * @param lo low coordinate values.
     * @param hi high coordinate values.
     */
    public Box3D(final Point3D lo, final Point3D hi) {
        super(lo, hi);
    }

    /**
     * Sets boundaries.
     *
     * @param loX x low coordinate.
     * @param loY y low coordinate
     * @param loZ z low coordinate.
     * @param hiX x high coordinate.
     * @param hiY y high coordinate.
     * @param hiZ z high coordinate.
     */
    public final void setBounds(final double loX, final double loY, final double loZ,
                                final double hiX, final double hiY, final double hiZ) {
        setBounds(new InhomogeneousPoint3D(loX, loY, loZ),
                new InhomogeneousPoint3D(hiX, hiY, hiZ));
    }
}
