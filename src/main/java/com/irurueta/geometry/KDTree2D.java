package com.irurueta.geometry;

import java.util.Collection;

/**
 * Implementation of a k-D tree in 2D.
 * Once a K-D tree is built for a collection of points, it can later be used to efficiently do certain operations
 * such as point locaiton, nearest points searches, etc.
 */
public class KDTree2D extends KDTree<Point2D> {

    /**
     * Constructor.
     * @param pts collection of points to store in the tree.
     */
    public KDTree2D(Collection<Point2D> pts) {
        super(pts, Point2D.class);
    }

    /**
     * Gets number of dimensions supported by this k-D tree implementation on provided list of points.
     * @return number of dimensions.
     */
    @Override
    public int getDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Creates a point.
     * @param value value to be set on point coordinates.
     * @return created point.
     */
    @Override
    protected Point2D createPoint(double value) {
        return new InhomogeneousPoint2D(value, value);
    }

    /**
     * Copies a point.
     * @param point point to be copied.
     * @return copied point.
     */
    @Override
    protected Point2D copyPoint(Point2D point) {
        return new InhomogeneousPoint2D(point);
    }
}
