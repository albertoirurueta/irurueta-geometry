package com.irurueta.geometry;

import java.util.Collection;

public class KDTree2D extends KDTree<Point2D> {

    public KDTree2D(Collection<Point2D> pts) {
        super(pts, Point2D.class);
    }

    @Override
    public int getDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    @Override
    protected Point2D createPoint(double value) {
        return new InhomogeneousPoint2D(value, value);
    }

    @Override
    protected Point2D copyPoint(Point2D point) {
        return new InhomogeneousPoint2D(point);
    }
}
