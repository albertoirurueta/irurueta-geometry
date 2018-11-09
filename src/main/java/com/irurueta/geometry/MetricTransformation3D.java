/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.MetricTransformation3DEstimator;
import com.irurueta.geometry.estimators.NotReadyException;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This class performs metric transformations on 3D space.
 * Metric transformations include transformations related to rotations,
 * translations and scale.
 */
public class MetricTransformation3D extends EuclideanTransformation3D 
        implements Serializable {
    
    /**
     * Scale factor. Negative values mean that objects get reversed. Values
     * greater than 1.0 means that objects get enlarged and values between 0.0
     * and 1.0 means that objects get reduced.
     */
    private double scale;
    
    /**
     * Default scale factor, which leaves objects with the same scale.
     */
    public static final double DEFAULT_SCALE = 1.0;
    
    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public MetricTransformation3D() {
        super();
        scale = DEFAULT_SCALE;
    }
    
    /**
     * Creates transformation with provided rotation.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public MetricTransformation3D(Rotation3D rotation)
            throws NullPointerException {
        super(rotation);
        scale = DEFAULT_SCALE;
    }
    
    /**
     * Creates transformation with provided 3D translation.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null.
     * @throws IllegalArgumentException raised if length of array is not equal 
     * to NUM_TRANSLATION_COORDS.
     */
    public MetricTransformation3D(double[] translation)
            throws NullPointerException, IllegalArgumentException {
        super(translation);
        scale = DEFAULT_SCALE;
    }
    
    /**
     * Creates transformation with provided scale value.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     */
    public MetricTransformation3D(double scale) {
        super();
        this.scale = scale;
    }
    
    /**
     * Creates transformation with provided rotation, translation and scale 
     * value.
     * @param rotation a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @throws NullPointerException raised if provided array is null or if
     * rotation is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     * to NUM_TRANSLATION_COORDS.
     */
    public MetricTransformation3D(Rotation3D rotation, double[] translation,
            double scale) throws NullPointerException, 
            IllegalArgumentException {
        super(rotation, translation);
        this.scale = scale;
    }
    
    /**
     * Creates transformation by estimating its internal values using provided 4
     * corresponding original and transformed points.
     * @param inputPoint1 1st input point.
     * @param inputPoint2 2nd input point.
     * @param inputPoint3 3rd input point.
     * @param inputPoint4 4th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input 
     * point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input 
     * point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input 
     * point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     * point.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate
     * points or numerical instabilities).
     */
    public MetricTransformation3D(Point3D inputPoint1, Point3D inputPoint2,
            Point3D inputPoint3, Point3D inputPoint4, Point3D outputPoint1,
            Point3D outputPoint2, Point3D outputPoint3, Point3D outputPoint4) 
            throws CoincidentPointsException {
        internalSetTransformationFromPoints(inputPoint1, inputPoint2, 
                inputPoint3, inputPoint4, outputPoint1, outputPoint2, 
                outputPoint3, outputPoint4);
    }
    
    /**
     * Returns scale of this transformation.
     * A value between 0.0 and 1.0 indicates that objects will be reduced,
     * a value greater than 1.0 indicates that objects will be enlarged, and a
     * negative value indicates that objects will be reversed.
     * @return scale.
     */
    public double getScale() {
        return scale;
    }
    
    /**
     * Sets scale of this transformation.
     * @param scale Scale value to be set. A value between 0.0 and 1.0 indicates
     * that objects will be reduced, a value greater than 1.0 indicates that 
     * objects will be enlarged, and a negative value indicates that objects 
     * will be reversed.
     */
    public void setScale(double scale) {
        this.scale = scale;
    }
    
    /**
     * Represents this transformation as a 4x4 matrix and stores the result in
     * provided instance.
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException Raised if provided instance is not a 4x4
     * matrix.
     */
    @Override
    public void asMatrix(Matrix m) throws IllegalArgumentException {
        if (m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        //set rotation        
        Matrix rot = getRotation().asInhomogeneousMatrix();
        rot.multiplyByScalar(scale);
        
        m.setSubmatrix(0, 0, 
                Rotation3D.INHOM_COORDS - 1, 
                Rotation3D.INHOM_COORDS - 1, rot);

        double[] translation = getTranslation();
        
        //set translation
        m.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1, 
                HOM_COORDS - 1, translation);
        
        //set last element
        m.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);        
    }
    
    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     * @param inputPoint point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    @Override
    public void transform(Point3D inputPoint, Point3D outputPoint) {
        inputPoint.normalize();
        getRotation().rotate(inputPoint, outputPoint);
        
        double[] translation = getTranslation();
        
        outputPoint.setInhomogeneousCoordinates(
                scale * outputPoint.getInhomX() + translation[0], 
                scale * outputPoint.getInhomY() + translation[1], 
                scale * outputPoint.getInhomZ() + translation[2]);
    }

    /**
     * Inverses this transformation.
     */
    @Override
    public void inverse() {
        inverse(this);
    }
    
    /**
     * Computes the inverse of this transformation and returns the result as a
     * new transformation instance.
     * @return inverse transformation.
     */
    @Override
    public Transformation3D inverseAndReturnNew() {
        MetricTransformation3D result = new MetricTransformation3D();
        inverse(result);
        return result;
    }    
    
    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     * @param result instance where inverse transformation will be stored.
     */
    protected void inverse(MetricTransformation3D result) {
        //Transformation is as follows: x' = s*R* x + t
        //Then inverse transformation is: (1/s)*R* x' = (1/s) * R' * s * R * x +
        //(1/s) * R'*t = x + (1/s) * R'*t
        //--> x = (1/s) * R'*x' - (1/s) * R'*t
        super.inverse(result);
        double[] translation = result.getTranslation();
        double invScale = 1.0 / scale;
        ArrayUtils.multiplyByScalar(translation, invScale, translation);
        result.scale = invScale;
    }
    
    /**
     * Converts this transformation into a metric transformation.
     * Because this method is inherited, and this instance is already metric,
     * this method just returns a copy of this transformation.
     * @return this transformation converted into a metric transformation.
     */
    @Override
    public MetricTransformation3D toMetric() {
        return new MetricTransformation3D(getRotation(), getTranslation(), 
                scale);
    }
    
    /**
     * Converts this transformation into an affine transformation.
     * @return this transformation converted into an affine transformation.
     */
    public AffineTransformation3D toAffine() {
        return new AffineTransformation3D(scale, getRotation(), 
                getTranslation());
    }    
    
    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided transformation.
     * @param transformation transformation to be combined with.
     */
    @Override
    public void combine(EuclideanTransformation3D transformation) {
        combine(transformation.toMetric(), this);
    }
    
    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     * @param transformation transformation to be combined with.
     * @return A new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    @Override
    public MetricTransformation3D combineAndReturnNew(
            EuclideanTransformation3D transformation) {
        
        MetricTransformation3D result = new MetricTransformation3D();
        combine(transformation.toMetric(), result);
        return result;
    }
    
    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided transformation.
     * @param transformation transformation to be combined with.
     */
    public void combine(MetricTransformation3D transformation) {
        combine(transformation, this);
    }
    
    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     * @param transformation transformation to be combined with.
     * @return a new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    public MetricTransformation3D combineAndReturnNew(
            MetricTransformation3D transformation) {
        
        MetricTransformation3D result = new MetricTransformation3D();
        combine(transformation, result);
        return result;
    }
    
    /**
     * Estimates this transformation internal parameters by using 4 
     * corresponding original and transformed points.
     * @param inputPoint1 1st input point.
     * @param inputPoint2 2nd input point.
     * @param inputPoint3 3rd input point.
     * @param inputPoint4 4th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input 
     * point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input 
     * point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     * point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     * point.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate
     * points or numerical instabilities).
     */
    @Override
    public void setTransformationFromPoints(Point3D inputPoint1, 
            Point3D inputPoint2, Point3D inputPoint3, Point3D inputPoint4,
            Point3D outputPoint1, Point3D outputPoint2, Point3D outputPoint3,
            Point3D outputPoint4) throws CoincidentPointsException {
        internalSetTransformationFromPoints(inputPoint1, inputPoint2, 
                inputPoint3, inputPoint4, outputPoint1, outputPoint2, 
                outputPoint3, outputPoint4);
    }
    
    /**
     * Combines this transformation with provided input transformation and 
     * stores the result into provided output transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided input transformation.
     * @param inputTransformation transformation to be combined with.
     * @param outputTransformation transformation where result will be stored.
     */    
    private void combine(MetricTransformation3D inputTransformation, 
            MetricTransformation3D outputTransformation) {
        //combination in matrix representation is:
        //[s1*R1 t1] * [s2*R2 t2] = [s1*s2*R1*R2 + t1*0T  s1*R1*t2 + t1*1] = [s1*s2*R1*R2  s1*R1*t2 + t1]
        //[0T   1 ]    [0T    1 ]   [0T*s2*R2 + 1*0T      0T*t2 + 1*1    ]   [0T           1            ]
        
        try {
            //we do translation first, because this.rotation might change later
            Matrix R1 = getRotation().asInhomogeneousMatrix();
            Matrix t2 = Matrix.newFromArray(inputTransformation.getTranslation(), 
                    true);
            R1.multiply(t2); //this is R1 * t2
            R1.multiplyByScalar(this.scale);
                  
            ArrayUtils.sum(R1.toArray(), this.getTranslation(),  
                    outputTransformation.getTranslation());
            
            outputTransformation.setRotation(
                    this.getRotation().combineAndReturnNew(
                    inputTransformation.getRotation()));
            
            outputTransformation.scale = this.scale * inputTransformation.scale;
        
        } catch (WrongSizeException ignore) { }
    }
    
    /**
     * Estimates this transformation internal parameters by using 4 
     * corresponding original and transformed points.
     * @param inputPoint1 1st input point.
     * @param inputPoint2 2nd input point.
     * @param inputPoint3 3rd input point.
     * @param inputPoint4 4th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input 
     * point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input 
     * point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     * point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     * point.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate
     * points or numerical instabilities).
     */
    private void internalSetTransformationFromPoints(Point3D inputPoint1, 
            Point3D inputPoint2, Point3D inputPoint3, Point3D inputPoint4,
            Point3D outputPoint1, Point3D outputPoint2, Point3D outputPoint3,
            Point3D outputPoint4) throws CoincidentPointsException {
        List<Point3D> inputPoints = new ArrayList<>();
        inputPoints.add(inputPoint1);
        inputPoints.add(inputPoint2);
        inputPoints.add(inputPoint3);
        inputPoints.add(inputPoint4);
        
        List<Point3D> outputPoints = new ArrayList<>();
        outputPoints.add(outputPoint1);
        outputPoints.add(outputPoint2);
        outputPoints.add(outputPoint3);
        outputPoints.add(outputPoint4);
        
        MetricTransformation3DEstimator estimator =
                new MetricTransformation3DEstimator(inputPoints,
                outputPoints);
        
        try {
            estimator.estimate(this);
        } catch (LockedException | NotReadyException ignore) {
            //never thrown
        }        
    }    
}
