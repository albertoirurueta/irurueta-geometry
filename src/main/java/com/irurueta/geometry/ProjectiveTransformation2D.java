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

import com.irurueta.algebra.*;
import com.irurueta.algebra.Utils;

import java.io.Serializable;
import java.util.Arrays;

/**
 * This class performs projective transformations on 2D space.
 * Projective transformations include any possible transformation that can be
 * applied to 2D points.
 */
public class ProjectiveTransformation2D extends Transformation2D 
        implements Serializable {

    /**
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 2;

    /**
     * Constant indicating the number of projective parameters that can be set
     * in projective parameters array.
     */
    @SuppressWarnings("all")
    public static final int NUM_PROJECTIVE_PARAMS = 3;

    /**
     * Constant defining number of inhomogeneous coordinates in 2D space.
     */
    public static final int INHOM_COORDS = 2;

    /**
     * Constant defining number of homogeneous coordinates in 2D space.
     */
    public static final int HOM_COORDS = 3;

    /**
     * Machine precision.
     */
    public static final double EPS = 1e-12;

    /**
     * Constant defining a large threshold to consider a matrix valid as
     * rotation.
     */
    private static final double LARGE_ROTATION_MATRIX_THRESHOLD = 1.0;

    /**
     * Internal 3x3 matrix containing transformation.
     */
    private Matrix t;
    
    /**
     * Indicates whether internal matrix is normalized.
     */
    private boolean normalized;

    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public ProjectiveTransformation2D() {
        super();
        try {
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation with provided internal matrix.
     * Notice that provided matrix should usually be invertible, otherwise the
     * transformation will be degenerate and its inverse will not be available.
     * @param t Internal 3x3 matrix.
     */
    public ProjectiveTransformation2D(Matrix t) {
        setT(t);
        normalize();
    }
    
    /**
     * Creates transformation with provided scale value.
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     */
    public ProjectiveTransformation2D(double scale) {
        double[] diag = new double[HOM_COORDS];
        Arrays.fill(diag, scale);
        diag[HOM_COORDS - 1] = 1.0; //set las element to 1.0
        t = Matrix.diagonal(diag);
        normalize();
    }
    
    /**
     * Creates transformation with provided rotation.
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public ProjectiveTransformation2D(Rotation2D rotation) {
        t = rotation.asHomogeneousMatrix();
        normalize();
    }
    
    /**
     * Creates transformation with provided scale and rotation.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public ProjectiveTransformation2D(double scale, Rotation2D rotation) {
        try {
            double[] diag = new double[INHOM_COORDS];
            Arrays.fill(diag, scale);
            Matrix a = Matrix.diagonal(diag);
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, a);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation with provided affine parameters and rotation.
     * @param params affine parameters including horizontal scaling, vertical
     * scaling and skewness.
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided parameters are null or
     * if provided rotation is null.
     */
    public ProjectiveTransformation2D(AffineParameters2D params, 
            Rotation2D rotation) {
        try {
            Matrix a = params.asMatrix();
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, a);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation with provided 2D translation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     * to NUM_TRANSLATION_COORDS.
     */
    public ProjectiveTransformation2D(double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 2, 1, 2, translation);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation with provided affine linear mapping and 
     * translation.
     * @param a affine linear mapping.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null or if
     * affine linear mapping is null.
     * @throws IllegalArgumentException raised if length of array is not equal 
     * to NUM_TRANSLATION_COORDS.
     */
    public ProjectiveTransformation2D(Matrix a, double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, a);
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation with provided scale and translation.
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided translation is null
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2.
     */
    public ProjectiveTransformation2D(double scale, double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        double[] diag = new double[HOM_COORDS];
        Arrays.fill(diag, scale);
        diag[HOM_COORDS - 1] = 1.0; //set last element to 1.0
        t = Matrix.diagonal(diag);
        
        //set translation
        t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                HOM_COORDS - 1, translation);
        normalize();
    }
    
    /**
     * Creates transformation with provided rotation and translation.
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided rotation or translation 
     * is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2.
     */
    public ProjectiveTransformation2D(Rotation2D rotation, double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        t = rotation.asHomogeneousMatrix();
        
        //set translation
        t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                HOM_COORDS - 1, translation);    
        normalize();
    }
    
    /**
     * Creates transformation with provided scale, rotation and translation.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided rotation or translation
     * is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2.
     */
    public ProjectiveTransformation2D(double scale, Rotation2D rotation,
            double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            double[] diag = new double[INHOM_COORDS];
            Arrays.fill(diag, scale);
            Matrix a = Matrix.diagonal(diag);
            a.multiply(rotation.asInhomogeneousMatrix());
        
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, a);
            //set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided scale, rotation and translation.
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @param projectiveParameters array of length 3 containing projective
     * parameters.
     * @throws NullPointerException raised if provided rotation or translation
     * is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2 or if projective parameters array doesn't have length 3.
     */
    public ProjectiveTransformation2D(double scale, Rotation2D rotation,
            double[] translation, double[] projectiveParameters) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        if (projectiveParameters.length != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            double value = projectiveParameters[HOM_COORDS - 1];
            double[] diag = new double[INHOM_COORDS];
            Arrays.fill(diag, scale);
            Matrix a = Matrix.diagonal(diag);
            a.multiply(rotation.asInhomogeneousMatrix());
        
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, a);
            //set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
            t.multiplyByScalar(value);
            
            t.setSubmatrix(HOM_COORDS - 1, 0, HOM_COORDS - 1,
                    HOM_COORDS - 1, projectiveParameters);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation with provided parameters, rotation and 
     * translation.
     * @param params Affine parameters including horizontal scaling, vertical
     * scaling and skewness.
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided parameters, rotation or
     * translation is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2.
     */
    public ProjectiveTransformation2D(AffineParameters2D params,
            Rotation2D rotation, double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            Matrix a = params.asMatrix();
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                    INHOM_COORDS - 1, a);
            //set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided parameters, rotation and 
     * translation.
     * @param params affine parameters including horizontal scaling, vertical
     * scaling and skewness.
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @param projectiveParameters array of length 3 containing projective
     * parameters.
     * @throws NullPointerException raised if provided parameters, rotation or
     * translation is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2 or if projective parameters array doesn't have length 3.
     */
    public ProjectiveTransformation2D(AffineParameters2D params,
            Rotation2D rotation, double[] translation, 
            double[] projectiveParameters) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        if (projectiveParameters.length != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            Matrix a = params.asMatrix();
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
            //set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
            double value = projectiveParameters[HOM_COORDS - 1];
            t.multiplyByScalar(value);
            
            t.setSubmatrix(HOM_COORDS - 1, 0, HOM_COORDS - 1, HOM_COORDS - 1,
                projectiveParameters);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        normalize();
    }
    
    /**
     * Creates transformation by estimating its internal matrix by providing 4 
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
    public ProjectiveTransformation2D(Point2D inputPoint1, Point2D inputPoint2,
            Point2D inputPoint3, Point2D inputPoint4, Point2D outputPoint1,
            Point2D outputPoint2, Point2D outputPoint3, Point2D outputPoint4)
            throws CoincidentPointsException {
        try {
            t = new Matrix(HOM_COORDS, HOM_COORDS);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, 
                inputPoint4, outputPoint1, outputPoint2, outputPoint3, 
                outputPoint4);
    }
    
    /**
     * Creates transformation by estimating its internal matrix by providing 4
     * corresponding original and transformed lines.
     * @param inputLine1 1st input line.
     * @param inputLine2 2nd input line.
     * @param inputLine3 3rd input line.
     * @param inputLine4 4th input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @param outputLine3 3rd transformed line corresponding to 3rd input line.
     * @param outputLine4 4th transformed line corresponding to 4th input line.
     * @throws CoincidentLinesException raised if transformation cannot be
     * estimated for some reason (line configuration degeneracy, duplicate lines
     * or numerical instabilities).
     */
    public ProjectiveTransformation2D(Line2D inputLine1, Line2D inputLine2,
            Line2D inputLine3, Line2D inputLine4, Line2D outputLine1, 
            Line2D outputLine2, Line2D outputLine3, Line2D outputLine4)
            throws CoincidentLinesException {
        setTransformationFromLines(inputLine1, inputLine2, inputLine3, 
                inputLine4, outputLine1, outputLine2, outputLine3, outputLine4);
    }
    
    /**
     * Returns internal matrix containing this transformation data.
     * Point transformation is computed as t * x, where x is a 2D point
     * expressed using homogeneous coordinates.
     * Usually the internal transformation matrix will be invertible.
     * When this is not the case, the transformation is considered degenerate
     * and its inverse will not be available.
     * @return internal transformation matrix.
     */
    public Matrix getT() {
        return t;
    }
    
    /**
     * Sets internal matrix containing this transformation data.
     * Point transformation is computed as t * x, where x is a 2D point
     * expressed using homogeneous coordinates.
     * Usually provided matrix will be invertible, when this is not the case
     * this transformation will become degenerate and its inverse will not be
     * available.
     * This method does not check whether provided matrix is invertible or not.
     * @param t transformation matrix.
     * @throws NullPointerException raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix is not 3x3
     */
    public final void setT(Matrix t) {
        if (t.getRows() != HOM_COORDS || t.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        this.t = t;
        normalized = false;
    }
    
    /**
     * Returns boolean indicating whether provided matrix will produce a 
     * degenerate projective transformation or not.
     * @param t a 3x3 matrix to be used as the internal matrix of a projective
     * transformation.
     * @return true if matrix will produce a degenerate transformation, false
     * otherwise.
     * @throws IllegalArgumentException raised if provided matrix is not 3x3.
     */
    public static boolean isDegenerate(Matrix t) {
        if (t.getRows() != HOM_COORDS || t.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            LUDecomposer decomposer = new LUDecomposer(t);
            decomposer.decompose();
            return decomposer.isSingular();
        } catch (AlgebraException e) {
            //if decomposition fails, assume that matrix is degenerate because
            //of numerical instabilities
            return true;
        }
    }   
    
    /**
     * Indicates whether this transformation is degenerate.
     * When a transformation is degenerate, its inverse cannot be computed.
     * @return true if transformation is degenerate, false otherwise.
     */
    public boolean isDegenerate() {
        return isDegenerate(t);
    }

    /**
     * Returns affine linear mapping matrix.
     * @see AffineTransformation2D
     * @return linear mapping matrix.
     */
    public Matrix getA() {
        Matrix a = t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1);
        a.multiplyByScalar(1.0 / t.getElementAt(HOM_COORDS - 1,
                HOM_COORDS - 1));
        return a;
    }
    
    /**
     * Sets affine linear mapping matrix.
     * @see AffineTransformation2D
     * @param a linear mapping matrix.
     * @throws NullPointerException raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix does not have
     * size 2x2.
     */
    public final void setA(Matrix a) {
        if (a == null) {
            throw new NullPointerException();
        }
        if (a.getRows() != INHOM_COORDS || a.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1,
                a.multiplyByScalarAndReturnNew(1.0 * t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1)));
    }
    
    /**
     * Normalizes current matrix instance.
     */
    public final void normalize() {
        if (!normalized) {
            double norm = Utils.normF(t);
            if (norm > EPS) {
                t.multiplyByScalar(1.0 / norm);
            }
            normalized = true;
        }
    }
    
    /**
     * Returns the 2D rotation component associated to this transformation.
     * Note: if this rotation instance is modified, its changes won't be 
     * reflected on this transformation until rotation is set again.
     * @return 2D rotation.
     * @throws AlgebraException if for some reason rotation cannot be estimated
     * (usually because of numerical instability).
     */
    public Rotation2D getRotation() throws AlgebraException {
        //Use QR decomposition to retrieve rotation component of this 
        //transformation
        normalize();
        RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        try {
            decomposer.decompose();
            return new Rotation2D(decomposer.getQ(),                     
                    LARGE_ROTATION_MATRIX_THRESHOLD); //a large threshold is 
                    //used because Q matrix is always assumed to be orthonormal
        } catch (InvalidRotationMatrixException ignore) {
            return null;
        }
    }
    
    /**
     * Sets 2D rotation for this transformation.
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     * @throws AlgebraException raised if for numerical reasons rotation cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation).
     */
    public void setRotation(Rotation2D rotation) throws AlgebraException {
        Matrix rotMatrix = rotation.asInhomogeneousMatrix();
        
        //Use QR decomposition to retrieve parameters matrix
        RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        Matrix localA = decomposer.getR(); //retrieves params matrix
        localA.multiply(rotMatrix);
        t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, localA);
        normalized = false;
    }
    
    /**
     * Adds provided rotation to current rotation assigned to this 
     * transformation.
     * @param rotation 2D rotation to be added.
     * @throws AlgebraException raised if for numerical reasons rotation cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation).
     */
    public void addRotation(Rotation2D rotation) throws AlgebraException {
       Rotation2D localRotation = getRotation();
       localRotation.combine(rotation);
       setRotation(localRotation);
    }
    
    /**
     * Sets scale of this transformation.
     * @param scale scale value to be set. A value between 0.0 and 1.0 indicates
     * that objects will be reduced, a value greater than 1.0 indicates that
     * objects will be enlarged, and a negative value indicates that objects 
     * will be reversed.
     * @throws AlgebraException raised if for numerical reasons scale cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation).
     */
    public void setScale(double scale) throws AlgebraException {
        normalize();
        double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        Matrix localA = decomposer.getR(); //params
        localA.setElementAt(0, 0, scale * value);
        localA.setElementAt(1, 1, scale * value);
        localA.multiply(decomposer.getQ());
        t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1, localA);
        normalized = false;
    }
    
    /**
     * Gets affine parameters of associated to this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     * @return affine parameters.
     * @throws AlgebraException raised if for numerical reasons affine
     * parameters cannot be retrieved (usually because of numerical instability
     * of the internal matrix of this instance).
     */
    public AffineParameters2D getAffineParameters() throws AlgebraException {
        AffineParameters2D parameters = new AffineParameters2D();
        getAffineParameters(parameters);
        return parameters;
    }
    
    /**
     * Computes affine parameters associated to this instance and stores the 
     * result in provided instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     * @param result instance where affine parameters will be stored.
     * @throws AlgebraException raised if for numerical reasons affine 
     * parameters cannot be retrieved (usually because of numerical instability
     * of the internal matrix of this instance).
     */
    public void getAffineParameters(AffineParameters2D result)
            throws AlgebraException {
        normalize();
        double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        Matrix r = decomposer.getR();
        r.multiplyByScalar(1.0 / value);
        result.fromMatrix(r);
    }
    
    /**
     * Sets affine parameters associated to this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     * @param parameters affine parameters to be set.
     * @throws AlgebraException raised if for numerical reasons affine
     * parameters cannot be set (usually because of numerical instability of
     * the internal matrix of this instance).
     */
    public void setAffineParameters(AffineParameters2D parameters)
            throws AlgebraException {
        normalize();
        double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        Matrix params = parameters.asMatrix();
        Matrix rotation = decomposer.getQ();
        
        params.multiply(rotation);  //params is equivalent to A because it
                                    //has been multiplied by rotation
        params.multiplyByScalar(value); //normalize
        t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, params);
        normalized = false;
    }
    
    /**
     * Returns the projective parameters associated to this instance.
     * These parameters are the located in the last row of the internal 
     * transformation matrix.
     * For affine, metric or euclidean transformations this last row is always
     * [0, 0, 1] (taking into account that transformation matrix is defined
     * up to scale).
     * @return Projective parameters returned as the array containing the values
     * of the last row of the internal transformation matrix.
     */
    public double[] getProjectiveParameters() {
        //return last row of matrix t
        return t.getSubmatrixAsArray(HOM_COORDS - 1, 0, HOM_COORDS - 1,
                HOM_COORDS - 1, true);
    }
    
    /**
     * Sets the projective parameters associated to this instance.
     * These parameters will be set in the last row of the internal 
     * transformation matrix.
     * For affine, matrix or euclidean transformations parameters are always
     * [0, 0, 1] (taking into account that transformation matrix is defined up
     * to scale).
     * @param params projective parameters to be set. It must be an array of
     * length 3.
     * @throws IllegalArgumentException raised if provided array does not have
     * length 3.
     */
    public final void setProjectiveParameters(double[] params) {
        if (params.length != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        t.setSubmatrix(HOM_COORDS - 1, 0, HOM_COORDS - 1,
                HOM_COORDS - 1, params);
        normalized = false;
    }
    
    /**
     * Returns 2D translation assigned to this transformation as an array
     * expressed in inhomogeneous coordinates.
     * Note: Updating the values of the returned array will not update the
     * translation of this instance. To do so, translation needs to be set 
     * again.
     * @return 2D translation array.
     */
    public double[] getTranslation() {
        normalize();
        double[] translation = t.getSubmatrixAsArray(0, HOM_COORDS - 1,
                INHOM_COORDS - 1, HOM_COORDS - 1);
        double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        ArrayUtils.multiplyByScalar(translation, 1.0 / value, translation);
        return translation;
    }
    
    /**
     * Obtains 2D translation assigned to this transformation and stores result
     * into provided array.
     * Note: updating the values of the returned array will not update the 
     * translation of this instance. To do so, translation needs to be set.
     * @param out array where translation values will be stored.
     * @throws WrongSizeException if provided array does not have length 2.
     */
    public void getTranslation(double[] out) throws WrongSizeException {
        t.getSubmatrixAsArray(0, HOM_COORDS - 1,
                INHOM_COORDS - 1, HOM_COORDS - 1, out);
        double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        ArrayUtils.multiplyByScalar(out, 1.0 / value, out);
    }
    
    /**
     * Sets 2D translation assigned to this transformation as an array expressed
     * in inhomogeneous coordinates.
     * @param translation 2D translation array.
     * @throws IllegalArgumentException raised if provided array does not have
     * length equal to NUM_TRANSLATION_COORDS.
     */
    public void setTranslation(double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        double[] translation2 = ArrayUtils.multiplyByScalarAndReturnNew(
                translation, value);
        t.setSubmatrix(0, HOM_COORDS - 1, translation2.length - 1,
                HOM_COORDS - 1, translation2);
        normalized = false;
    }
    
    /**
     * Adds provided translation to current translation on this transformation.
     * Provided translation must be expressed as an array of inhomogeneous
     * coordinates.
     * @param translation 2D translation array.
     * @throws IllegalArgumentException raised if provided array does not have
     * length equal to NUM_TRANSLATION_COORDS.
     */
    public void addTranslation(double[] translation) {
        double[] currentTranslation = getTranslation();
        ArrayUtils.sum(currentTranslation, translation, currentTranslation);
        setTranslation(currentTranslation);
    }
    
    /**
     * Returns current x coordinate translation assigned to this transformation.
     * @return X coordinate translation.
     */
    public double getTranslationX() {
        normalize();
        return t.getElementAt(0, HOM_COORDS - 1) / t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1);
    }
    
    /**
     * Sets x coordinate translation to be made by this transformation.
     * @param translationX X coordinate translation to be set.
     */
    public void setTranslationX(double translationX) {
        t.setElementAt(0, HOM_COORDS - 1, translationX * t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1));
        normalized = false;
    }

    /**
     * Returns current y coordinate translation assigned to this transformation.
     * @return Y coordinate translation.
     */
    public double getTranslationY() {
        normalize();
        return t.getElementAt(1, HOM_COORDS - 1) / t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1);
    }
    
    /**
     * Sets y coordinate translation to be made by this transformation.
     * @param translationY Y coordinate translation to be set.
     */
    public void setTranslationY(double translationY) {
        t.setElementAt(1, HOM_COORDS - 1, translationY * t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1));
        normalized = false;
    }
    
    /**
     * Sets x, y coordinates of translation to be made by this transformation.
     * @param translationX translation x coordinate to be set.
     * @param translationY translation y coordinate to be set.
     */
    public void setTranslation(double translationX, double translationY) {
        setTranslationX(translationX);
        setTranslationY(translationY);
    }
    
    /**
     * Sets x, y coordinates of translation to be made by this transformation.
     * @param translation translation to be set.
     */
    public void setTranslation(Point2D translation) {
        setTranslation(translation.getInhomX(), translation.getInhomY());
    }
    
    /**
     * Gets x, y coordinates of translation to be made by this transformation
     * as a new point.
     * @return a new point containing translation coordinates.
     */
    public Point2D getTranslationPoint() {
        Point2D out = Point2D.create();
        getTranslationPoint(out);
        return out;
    }
    
    /**
     * Gets x, y coordinates of translation to be made by this transformation
     * and stores them into provided point.
     * @param out point where translation coordinates will be stored.
     */
    public void getTranslationPoint(Point2D out) {
        out.setInhomogeneousCoordinates(getTranslationX(), getTranslationY());
    }
    
    /**
     * Adds provided x coordinate to current translation assigned to this 
     * transformation.
     * @param translationX X coordinate to be added to current translation.
     */
    public void addTranslationX(double translationX) {
        setTranslationX(getTranslationX() + translationX);
    }
    
    /**
     * Adds provided y coordinate to current translation assigned to this
     * transformation.
     * @param translationY Y coordinate to be added to current translation.
     */
    public void addTranslationY(double translationY) {
        setTranslationY(getTranslationY() + translationY);
    }

    /**
     * Adds provided coordinates to current translation assigned ot this 
     * transformation.
     * @param translationX x coordinate to be added to current translation.
     * @param translationY y coordinate to be added to current translation.
     */
    public void addTranslation(double translationX, double translationY) {
        addTranslationX(translationX);
        addTranslationY(translationY);
    }
    
    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     * @param translation x, y coordinates to be added to current translation.
     */
    public void addTranslation(Point2D translation) {
        addTranslation(translation.getInhomX(), translation.getInhomY());
    }
    
    /**
     * Represents this transformation as a 3x3 matrix.
     * A point can be transformed as t * p, where t is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     * @return This transformation in matrix form.
     */
    @Override
    public Matrix asMatrix() {
        return t.clone();
    }
    
    /**
     * Represents this transformation as a 3x3 matrix and stores the result in
     * provided instance.
     * @param m Instance where transformation matrix will be stored.
     * @throws IllegalArgumentException Raised if provided instance is not a 3x3 
     * matrix.
     */
    @Override
    public void asMatrix(Matrix m) {
        if (m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        m.copyFrom(t);
    }
    
    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     * @param inputPoint point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    @Override
    public void transform(Point2D inputPoint, Point2D outputPoint) {
        
        inputPoint.normalize();
        normalize();
        try {
            Matrix point = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            point.setElementAtIndex(0, inputPoint.getHomX());
            point.setElementAtIndex(1, inputPoint.getHomY());
            point.setElementAtIndex(2, inputPoint.getHomW());
        
            Matrix transformedPoint = t.multiplyAndReturnNew(point);
        
            outputPoint.setHomogeneousCoordinates(
                    transformedPoint.getElementAtIndex(0), 
                    transformedPoint.getElementAtIndex(1), 
                    transformedPoint.getElementAtIndex(2));
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
    
    /**
     * Transforms a conic using this transformation and stores the result into
     * provided output conic.
     * @param inputConic conic to be transformed.
     * @param outputConic instance where data of transformed conic will be 
     * stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output conic matrix is not considered to be symmetric.
     * @throws AlgebraException raised if transform cannot be computed becauseof
     * numerical instabilities.
     */            
    @Override
    public void transform(Conic inputConic, Conic outputConic)
            throws NonSymmetricMatrixException, AlgebraException {
        //point' * conic * point = 0
        //point' * t' * transformedConic * t * point = 0
        //where:
        // - transformedPoint = t * point

        //Hence:
        // transformedConic = t^-' * conic * t^-1

        inputConic.normalize();
        
        Matrix c = inputConic.asMatrix();
        normalize();

        Matrix invT = inverseAndReturnNew().asMatrix();
        //normalize transformation matrix invT to increase accuracy
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        
        Matrix m = invT.transposeAndReturnNew();
        try {
            m.multiply(c);
            m.multiply(invT);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        
        //normalize resulting m matrix to increase accuracy so that it can be
        //considered symmetric
        norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        outputConic.setParameters(m);        
    }
    
    /**
     * Transforms a dual conic using this transformation and stores the result
     * into provided output dual conic.
     * @param inputDualConic Dual conic to be transformed.
     * @param outputDualConic Instance where data of transformed dual conic will
     * be stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output dual conic matrix is not considered to be symmetric.
     * @throws AlgebraException Raised if transform cannot be computed because
     * of numerical instabilities.
     */        
    @Override
    public void transform(DualConic inputDualConic, DualConic outputDualConic) 
            throws NonSymmetricMatrixException, AlgebraException {
        //line' * dualConic * line = 0
        //line' * t^-1 * t * dualConic * t' * t^-1' * line

        //Hence:
        //transformed line : t^-1'*line
        //transformed dual conic: t * dualConic * t'

        inputDualConic.normalize();
        normalize();
        
        Matrix dualC = inputDualConic.asMatrix();
        Matrix transT = t.transposeAndReturnNew();

        Matrix m = t.multiplyAndReturnNew(dualC);
        m.multiply(transT);

        //normalize resulting m matrix to increase accuracy so that it can be
        //considered symmetric
        double norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        outputDualConic.setParameters(m);
    }
    
    /**
     * Transforms provided input line using this transformation and stores the
     * result into provided output line instance.
     * @param inputLine line to be transformed.
     * @param outputLine instance where data of transformed line will be stored
     * @throws AlgebraException Raised if transform cannot be computed because
     * of numerical instabilities.
     */        
    @Override
    public void transform(Line2D inputLine, Line2D outputLine) 
            throws AlgebraException {
        //line' * point = 0 --> line' * t^-1 * t * point
        //(line' * t^-1)*(t*point) = (t^-1'*line)'*(t*point)
        //where:
        //- transformedLine = t^-1'*line
        //- transformedPoint = t*point

        
        inputLine.normalize();
        normalize();
        
        Matrix invT = inverseAndReturnNew().asMatrix();
        Matrix l = Matrix.newFromArray(inputLine.asArray());

        invT.transpose();
        invT.multiply(l);

        outputLine.setParameters(invT.toArray());
    }
    
    /**
     * Inverses this transformation.
     * @throws AlgebraException if inverse transform cannot be computed because
     * of numerical instabilities.
     */
    public void inverse() throws AlgebraException {
        inverse(this);
    }
    
    /**
     * Computes the inverse of this transformation and returns the result as a
     * new transformation instance.
     * @return inverse transformation.
     * @throws AlgebraException if inverse transform cannot be computed because
     * of numerical instabilities.
     */
    public Transformation2D inverseAndReturnNew() throws AlgebraException {
        ProjectiveTransformation2D result = new ProjectiveTransformation2D();
        inverse(result);
        return result;
    }
    
    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     * @param result Instance where inverse transformation will be stored.
     * @throws AlgebraException if inverse transform cannot be computed because
     * of numerical instabilities.
     */
    protected void inverse(ProjectiveTransformation2D result) 
            throws AlgebraException {

        result.t = Utils.inverse(t);
    }    
    
    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided transformation.
     * @param transformation transformation to be combined with.
     */
    public void combine(ProjectiveTransformation2D transformation) {
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
    public ProjectiveTransformation2D combineAndReturnNew(
            ProjectiveTransformation2D transformation) {
        
        ProjectiveTransformation2D result = new ProjectiveTransformation2D();
        combine(transformation, result);
        return result;
    }

    /**
     * Combines this transformation with provided input transformation and 
     * stores the result into provided output transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided input transformation.
     * @param inputTransformation transformation to be combined with.
     * @param outputTransformation transformation where result will be stored.
     */
    private void combine(ProjectiveTransformation2D inputTransformation, 
            ProjectiveTransformation2D outputTransformation) {
        //combination in matrix representation is: T1 * T2
        
        normalize();
        inputTransformation.normalize();
        
        
        try {
            outputTransformation.t = this.t.multiplyAndReturnNew(
                    inputTransformation.t);
            
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }    
    
    /**
     * Estimates this transformation internal matrix by providing 4 
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
    public final void setTransformationFromPoints(Point2D inputPoint1, 
            Point2D inputPoint2, Point2D inputPoint3, Point2D inputPoint4, 
            Point2D outputPoint1, Point2D outputPoint2, Point2D outputPoint3, 
            Point2D outputPoint4) throws CoincidentPointsException {
        
        //normalize points to increase accuracy
        inputPoint1.normalize();
        inputPoint2.normalize();
        inputPoint3.normalize();
        inputPoint4.normalize();
        
        outputPoint1.normalize();
        outputPoint2.normalize();
        outputPoint3.normalize();
        outputPoint4.normalize();
        
        //matrix of homogeneous linear system of equations.
        //There are 9 unknowns and 8 equations (2 for each pair of corresponding 
        //points)
        Matrix m = null;
        try {
            m = new Matrix(8, 9); //build matrix initialized to zero
                
            //1st pair of points
            double iX = inputPoint1.getHomX();
            double iY = inputPoint1.getHomY();
            double iW = inputPoint1.getHomW();

            double oX = outputPoint1.getHomX();
            double oY = outputPoint1.getHomY();
            double oW = outputPoint1.getHomW();

            double oWiX = oW * iX;
            double oWiY = oW * iY;
            double oWiW = oW * iW;

            double oXiX = oX * iX;
            double oXiY = oX * iY;
            double oXiW = oX * iW;

            double oYiX = oY * iX;
            double oYiY = oY * iY;
            double oYiW = oY * iW;

            double tmp = oWiX * oWiX + oWiY * oWiY + oWiW * oWiW;
            double norm = Math.sqrt(tmp +
                    oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiW / norm);
            m.setElementAt(0, 6, -oXiX / norm);
            m.setElementAt(0, 7, -oXiY / norm);
            m.setElementAt(0, 8, -oXiW / norm);

            norm = Math.sqrt(tmp +
                    oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

            m.setElementAt(1, 3, oWiX / norm);
            m.setElementAt(1, 4, oWiY / norm);
            m.setElementAt(1, 5, oWiW / norm);
            m.setElementAt(1, 6, -oYiX / norm);
            m.setElementAt(1, 7, -oYiY / norm);
            m.setElementAt(1, 8, -oYiW / norm);

            //2nd pair of points
            iX = inputPoint2.getHomX();
            iY = inputPoint2.getHomY();
            iW = inputPoint2.getHomW();

            oX = outputPoint2.getHomX();
            oY = outputPoint2.getHomY();
            oW = outputPoint2.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiW = oW * iW;

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiW = oY * iW;

            tmp = oWiX * oWiX + oWiY * oWiY + oWiW * oWiW;
            norm = Math.sqrt(tmp +
                    oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

            m.setElementAt(2, 0, oWiX / norm);
            m.setElementAt(2, 1, oWiY / norm);
            m.setElementAt(2, 2, oWiW / norm);
            m.setElementAt(2, 6, -oXiX / norm);
            m.setElementAt(2, 7, -oXiY / norm);
            m.setElementAt(2, 8, -oXiW / norm);

            norm = Math.sqrt(tmp +
                    oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

            m.setElementAt(3, 3, oWiX / norm);
            m.setElementAt(3, 4, oWiY / norm);
            m.setElementAt(3, 5, oWiW / norm);
            m.setElementAt(3, 6, -oYiX / norm);
            m.setElementAt(3, 7, -oYiY / norm);
            m.setElementAt(3, 8, -oYiW / norm);

            //3rd pair of points
            iX = inputPoint3.getHomX();
            iY = inputPoint3.getHomY();
            iW = inputPoint3.getHomW();

            oX = outputPoint3.getHomX();
            oY = outputPoint3.getHomY();
            oW = outputPoint3.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiW = oW * iW;

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiW = oY * iW;

            tmp = oWiX * oWiX + oWiY * oWiY + oWiW * oWiW;
            norm = Math.sqrt(tmp +
                    oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

            m.setElementAt(4, 0, oWiX / norm);
            m.setElementAt(4, 1, oWiY / norm);
            m.setElementAt(4, 2, oWiW / norm);
            m.setElementAt(4, 6, -oXiX / norm);
            m.setElementAt(4, 7, -oXiY / norm);
            m.setElementAt(4, 8, -oXiW / norm);

            norm = Math.sqrt(tmp +
                    oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

            m.setElementAt(5, 3, oWiX / norm);
            m.setElementAt(5, 4, oWiY / norm);
            m.setElementAt(5, 5, oWiW / norm);
            m.setElementAt(5, 6, -oYiX / norm);
            m.setElementAt(5, 7, -oYiY / norm);
            m.setElementAt(5, 8, -oYiW / norm);

            //4th pair of points
            iX = inputPoint4.getHomX();
            iY = inputPoint4.getHomY();
            iW = inputPoint4.getHomW();

            oX = outputPoint4.getHomX();
            oY = outputPoint4.getHomY();
            oW = outputPoint4.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiW = oW * iW;

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiW = oY * iW;

            tmp = oWiX * oWiX + oWiY * oWiY + oWiW * oWiW;
            norm = Math.sqrt(tmp +
                    oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiW / norm);
            m.setElementAt(6, 6, -oXiX / norm);
            m.setElementAt(6, 7, -oXiY / norm);
            m.setElementAt(6, 8, -oXiW / norm);

            norm = Math.sqrt(tmp +
                    oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

            m.setElementAt(7, 3, oWiX / norm);
            m.setElementAt(7, 4, oWiY / norm);
            m.setElementAt(7, 5, oWiW / norm);
            m.setElementAt(7, 6, -oYiX / norm);
            m.setElementAt(7, 7, -oYiY / norm);
            m.setElementAt(7, 8, -oYiW / norm);
        } catch (WrongSizeException ignore) {
            //never happens
        }
                        
        //use SVD to decompose matrix m
        Matrix v;
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            //ensure that matrix m has enough rank and there is a unique 
            //solution (up to scale)
            if (decomposer.getRank() < 8) {
                throw new CoincidentPointsException();
            }
            v = decomposer.getV(); //V is 9x9
            
            //last column of V will contain parameters of transformation
            t.setSubmatrix(0, 0, HOM_COORDS - 1, HOM_COORDS - 1,
                    v.getSubmatrix(0, 8, 8, 8).toArray(), false);
            normalized = true; //because columns of V are normalized after SVD
            
        } catch (AlgebraException e) {
            throw new CoincidentPointsException(e);
        }
    }
    
    /**
     * Estimates this transformation internal matrix by providing 4 
     * corresponding original and transformed lines.
     * @param inputLine1 1st input line.
     * @param inputLine2 2nd input line.
     * @param inputLine3 3rd input line.
     * @param inputLine4 4th input line.
     * @param outputLine1 1st transformed line corresponding to 1st input 
     * line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input
     * line.
     * @param outputLine3 3rd transformed line corresponding to 3rd input
     * line.
     * @param outputLine4 4th transformed line corresponding to 4th input
     * line.
     * @throws CoincidentLinesException raised if transformation cannot be
     * estimated for some reason (line configuration degeneracy, duplicate 
     * line or numerical instabilities).
     */
    public final void setTransformationFromLines(Line2D inputLine1, 
            Line2D inputLine2, Line2D inputLine3, Line2D inputLine4, 
            Line2D outputLine1, Line2D outputLine2, Line2D outputLine3, 
            Line2D outputLine4) throws CoincidentLinesException {
        
        //normalize lines to increase accuracy
        inputLine1.normalize();
        inputLine2.normalize();
        inputLine3.normalize();
        inputLine4.normalize();
        
        outputLine1.normalize();
        outputLine2.normalize();
        outputLine3.normalize();
        outputLine4.normalize();
        
        //matrix of homogeneous linear system of equations.
        //There are 9 unknowns and 8 equations (2 for each pair of corresponding 
        //points)
        Matrix m = null;
        try {
            m = new Matrix(8, 9); //build matrix initialized to zero
                
            //1st pair of lines
            double iA = inputLine1.getA();
            double iB = inputLine1.getB();
            double iC = inputLine1.getC();

            double oA = outputLine1.getA();
            double oB = outputLine1.getB();
            double oC = outputLine1.getC();

            double oCiA = oC * iA;
            double oCiB = oC * iB;
            double oCiC = oC * iC;

            double oAiA = oA * iA;
            double oAiB = oA * iB;
            double oAiC = oA * iC;

            double oBiA = oB * iA;
            double oBiB = oB * iB;
            double oBiC = oB * iC;

            double tmp = oCiA * oCiA + oCiB * oCiB + oCiC * oCiC;
            double norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(0, 0, oCiA / norm);
            m.setElementAt(0, 1, oCiB / norm);
            m.setElementAt(0, 2, oCiC / norm);
            m.setElementAt(0, 6, -oAiA / norm);
            m.setElementAt(0, 7, -oAiB / norm);
            m.setElementAt(0, 8, -oAiC / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(1, 3, oCiA / norm);
            m.setElementAt(1, 4, oCiB / norm);
            m.setElementAt(1, 5, oCiC / norm);
            m.setElementAt(1, 6, -oBiA / norm);
            m.setElementAt(1, 7, -oBiB / norm);
            m.setElementAt(1, 8, -oBiC / norm);

            //2nd pair of lines
            iA = inputLine2.getA();
            iB = inputLine2.getB();
            iC = inputLine2.getC();

            oA = outputLine2.getA();
            oB = outputLine2.getB();
            oC = outputLine2.getC();

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;

            tmp = oCiA * oCiA + oCiB * oCiB + oCiC * oCiC;
            norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(2, 0, oCiA / norm);
            m.setElementAt(2, 1, oCiB / norm);
            m.setElementAt(2, 2, oCiC / norm);
            m.setElementAt(2, 6, -oAiA / norm);
            m.setElementAt(2, 7, -oAiB / norm);
            m.setElementAt(2, 8, -oAiC / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(3, 3, oCiA / norm);
            m.setElementAt(3, 4, oCiB / norm);
            m.setElementAt(3, 5, oCiC / norm);
            m.setElementAt(3, 6, -oBiA / norm);
            m.setElementAt(3, 7, -oBiB / norm);
            m.setElementAt(3, 8, -oBiC / norm);

            //3rd pair of points
            iA = inputLine3.getA();
            iB = inputLine3.getB();
            iC = inputLine3.getC();

            oA = outputLine3.getA();
            oB = outputLine3.getB();
            oC = outputLine3.getC();

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;

            tmp = oCiA * oCiA + oCiB * oCiB + oCiC * oCiC;
            norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(4, 0, oCiA / norm);
            m.setElementAt(4, 1, oCiB / norm);
            m.setElementAt(4, 2, oCiC / norm);
            m.setElementAt(4, 6, -oAiA / norm);
            m.setElementAt(4, 7, -oAiB / norm);
            m.setElementAt(4, 8, -oAiC / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(5, 3, oCiA / norm);
            m.setElementAt(5, 4, oCiB / norm);
            m.setElementAt(5, 5, oCiC / norm);
            m.setElementAt(5, 6, -oBiA / norm);
            m.setElementAt(5, 7, -oBiB / norm);
            m.setElementAt(5, 8, -oBiC / norm);

            //4th pair of points
            iA = inputLine4.getA();
            iB = inputLine4.getB();
            iC = inputLine4.getC();

            oA = outputLine4.getA();
            oB = outputLine4.getB();
            oC = outputLine4.getC();

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;

            tmp = oCiA * oCiA + oCiB * oCiB + oCiC * oCiC;
            norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(6, 0, oCiA / norm);
            m.setElementAt(6, 1, oCiB / norm);
            m.setElementAt(6, 2, oCiC / norm);
            m.setElementAt(6, 6, -oAiA / norm);
            m.setElementAt(6, 7, -oAiB / norm);
            m.setElementAt(6, 8, -oAiC / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(7, 3, oCiA / norm);
            m.setElementAt(7, 4, oCiB / norm);
            m.setElementAt(7, 5, oCiC / norm);
            m.setElementAt(7, 6, -oBiA / norm);
            m.setElementAt(7, 7, -oBiB / norm);
            m.setElementAt(7, 8, -oBiC / norm);
        } catch (WrongSizeException ignore) {
            //never happens
        }
                        
        //use SVD to decompose matrix m
        Matrix v;
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            //ensure that matrix m has enough rank and there is a unique 
            //solution (up to scale)
            if (decomposer.getRank() < 8) {
                throw new CoincidentLinesException();
            }
            v = decomposer.getV(); //V is 9x9
            
            //last column of V will contain parameters of transformation
            Matrix transInvT = new Matrix(HOM_COORDS, HOM_COORDS);
            transInvT.setSubmatrix(0, 0, HOM_COORDS - 1, HOM_COORDS - 1,
                    v.getSubmatrix(0, 8, 8, 8).toArray(),
                    false);
            transInvT.transpose(); //this is now invT
            t = Utils.inverse(transInvT);
            normalized = false; //invT is normalized, but not t
            
        } catch(AlgebraException e) {
            throw new CoincidentLinesException(e);
        }
    }    
}
