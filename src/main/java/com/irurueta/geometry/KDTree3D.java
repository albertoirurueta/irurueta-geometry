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
    public KDTree3D(Collection<Point3D> pts) {
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
    protected Point3D createPoint(double value) {
        return new InhomogeneousPoint3D(value, value, value);
    }

    /**
     * Copies a point.
     * @param point point to be copied.
     * @return copied point.
     */
    @Override
    protected Point3D copyPoint(Point3D point) {
        return new InhomogeneousPoint3D(point);
    }
}
