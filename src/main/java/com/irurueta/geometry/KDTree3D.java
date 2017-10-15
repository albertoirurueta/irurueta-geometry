package com.irurueta.geometry;

import java.util.Collection;

public class KDTree3D extends KDTree<Point3D> {

    public KDTree3D(Collection<Point3D> pts) {
        super(pts, Point3D.class);
    }

    @Override
    public int getDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    @Override
    protected Point3D createPoint(double value) {
        return new InhomogeneousPoint3D(value, value, value);
    }

    @Override
    protected Point3D copyPoint(Point3D point) {
        return new InhomogeneousPoint3D(point);
    }
}
