/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.AffineTransformation3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 28, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RQDecomposer;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import java.io.Serializable;
import java.util.Arrays;

/**
 * This class performs affine transformations on 3D space.
 * Affine transformations include transformations related to rotations,
 * translations, independently scaling horizontal or vertical coordinates
 * or skewing the coordinates axis.
 * This class is not intended to be used on points located at infinity or
 * at very large coordinates, since numerical instabilities may occur. For
 * those cases use a ProjectiveTransformation3D instead.
 */
public class AffineTransformation3D extends Transformation3D 
        implements Serializable {

    /**
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 3;
    
    
    /**
     * Constant defining number of inhomogeneous coordinates in 3D space.
     */
    public static final int INHOM_COORDS = 3;
    
    /**
     * Constant defining number of homogeneous coordinates in 3D space.
     */
    public static final int HOM_COORDS = 4;
    
    /**
     * Linear mapping.
     */
    private Matrix A;
    
    /**
     * 2D translation to be performed on geometric objects.
     * Translation is specified using inhomogeneous coordinates.
     */
    private double[] translation;    
        
    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public AffineTransformation3D() {
        super();
        try {
            A = Matrix.identity(INHOM_COORDS, INHOM_COORDS);
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided rotation.
     * @param A linear mapping.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation3D(Matrix A)
            throws NullPointerException, IllegalArgumentException {
        setA(A);
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided scale value.
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     */
    public AffineTransformation3D(double scale) {
        double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        A = Matrix.diagonal(diag);
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided rotation.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation3D(Rotation3D rotation) 
            throws NullPointerException {
        A = rotation.asInhomogeneousMatrix();
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided scale and rotation.
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation3D(double scale, Rotation3D rotation)
            throws NullPointerException {
        double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        A = Matrix.diagonal(diag);
        try {
            A.multiply(rotation.asInhomogeneousMatrix());
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];        
    }
    
    /**
     * Creates transformation with provided affine parameters and rotation.
     * @param params Affine parameters including horizontal scaling, vertical 
     * scaling and skewness.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided parameters are null or
     * if provided rotation is null.
     */
    public AffineTransformation3D(AffineParameters3D params, 
            Rotation3D rotation) throws NullPointerException {
        A = params.asMatrix();
        try {
            A.multiply(rotation.asInhomogeneousMatrix());
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];                
    }
        
    /**
     * Creates transformation with provided 3D translation.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     * to NUM_TRANSLATION_COORDS.
     */
    public AffineTransformation3D(double[] translation) 
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        try {
            A = Matrix.identity(INHOM_COORDS, INHOM_COORDS);
        } catch (WrongSizeException ignore) { }
        this.translation = translation;
    }
    
    /**
     * Creates transformation with provided rotation, translation and scale 
     * value.
     * @param A linear mapping.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null or if
     * linear mapping is null.
     * @throws IllegalArgumentException Raised if length of array is not equal 
     * to NUM_TRANSLATION_COORDS.
     */
    public AffineTransformation3D(Matrix A, double[] translation) 
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        this.translation = translation;
        
        setA(A);
    }    
    
    /**
     * Creates transformation with provided scale and translation.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param translation Array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided translation is null.
     * @throws IllegalArgumentException Raised if provided translation does not
     * have length 3.
     */
    public AffineTransformation3D(double scale, double[] translation)
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        A = Matrix.diagonal(diag);
        
        this.translation = translation;        
    }
    
    /**
     * Creates transformation with provided rotation and translation.
     * @param rotation a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided rotation or translation
     * is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 3.
     */
    public AffineTransformation3D(Rotation3D rotation, double[] translation)
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        A = rotation.asInhomogeneousMatrix();        
        this.translation = translation;                
    }
    
    /**
     * Creates transformation with provided scale, rotation and translation.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param rotation a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided rotation or translation
     * is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 3.
     */
    public AffineTransformation3D(double scale, Rotation3D rotation, 
            double[] translation) throws NullPointerException, 
            IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        A = Matrix.diagonal(diag);
        try {
            A.multiply(rotation.asInhomogeneousMatrix());
        } catch (WrongSizeException ignore) { }

        this.translation = translation;        
    }
    
    /**
     * Creates transformation with provided parameters, rotation and 
     * translation.
     * @param params affine parameters including horizontal scaling, vertical 
     * scaling and skewness.
     * @param rotation a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided parameters, rotation or
     * translation is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 3.
     */
    public AffineTransformation3D(AffineParameters3D params, 
            Rotation3D rotation, double[] translation) 
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        A = params.asMatrix();
        try {
            A.multiply(rotation.asInhomogeneousMatrix());
        } catch (WrongSizeException ignore) { }
        this.translation = translation;                
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
    public AffineTransformation3D(Point3D inputPoint1, Point3D inputPoint2,
            Point3D inputPoint3, Point3D inputPoint4, Point3D outputPoint1, 
            Point3D outputPoint2, Point3D outputPoint3, Point3D outputPoint4) 
            throws CoincidentPointsException {
        try {
            A = new Matrix(INHOM_COORDS, INHOM_COORDS);
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];
        setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3,
                inputPoint4, outputPoint1, outputPoint2, outputPoint3,
                outputPoint4);
    }    

   /**
     * Creates transformation by estimating its internal values using provided 4 
     * corresponding original and transformed planes.
     * @param inputPlane1 1st input plane.
     * @param inputPlane2 2nd input plane.
     * @param inputPlane3 3rd input plane.
     * @param inputPlane4 4th input plane.
     * @param outputPlane1 1st transformed plane corresponding to 1st input 
     * plane.
     * @param outputPlane2 2nd transformed plane corresponding to 2nd input
     * plane.
     * @param outputPlane3 3rd transformed plane corresponding to 3rd input
     * plane.
     * @param outputPlane4 4th transformed plane corresponding to 4th input
     * plane.
     * @throws CoincidentPlanesException raised if transformation cannot be
     * estimated for some reason (plane configuration degeneracy, duplicate 
     * planes or numerical instabilities).
     */    
    public AffineTransformation3D(Plane inputPlane1, Plane inputPlane2,
            Plane inputPlane3, Plane inputPlane4, Plane outputPlane1, 
            Plane outputPlane2, Plane outputPlane3, Plane outputPlane4) 
            throws CoincidentPlanesException {
        setTransformationFromPlanes(inputPlane1, inputPlane2, inputPlane3,
                inputPlane4, outputPlane1, outputPlane2, outputPlane3,
                outputPlane4);
    }    
    
    /**
     * Creates transformation by estimating internal parameters using provided 2 
     * corresponding original and transformed lines.
     * @param inputLine1 1st input line.
     * @param inputLine2 2nd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @throws CoincidentLinesException raised if transformation cannot be
     * estimated for some reason (line configuration degeneracy, duplicate lines
     * or numerical instabilities).
     */    
    public AffineTransformation3D(Line3D inputLine1, Line3D inputLine2,
            Line3D outputLine1, Line3D outputLine2) 
            throws CoincidentLinesException {
        setTransformationFromLines(inputLine1, inputLine2, outputLine1, 
                outputLine2);
    }
    
    /**
     * Returns linear mapping matrix to perform affine transformation.
     * Point transformation is computed as A * x + t, where x is a point and t
     * is the amount of translation.
     * @return linear mapping matrix.
     */
    public Matrix getA() {
        return A;
    }
    
    /**
     * Sets linear mapping matrix to perform affine transformation.
     * @param A linear mapping matrix.
     * @throws NullPointerException raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix does not have
     * size 3x3.
     */
    public final void setA(Matrix A) throws NullPointerException, 
            IllegalArgumentException {
        if(A == null) {
            throw new NullPointerException();
        }
        if(A.getRows() != INHOM_COORDS || A.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        this.A = A;
    }
    
    /**
     * Returns 3D rotation assigned to this transformation.
     * Note: if this rotation instance is modified, its changes won't be 
     * reflected on this instance until rotation is set again.
     * @return 3D rotation.
     * @throws AlgebraException if for some reason rotation cannot
     * be estimated (usually because of numerical instability).
     */
    public Rotation3D getRotation() throws AlgebraException {
        //Use QR decomposition to retrieve rotation
        RQDecomposer decomposer = new RQDecomposer(A);
        try {
            decomposer.decompose();
            return new MatrixRotation3D(decomposer.getQ());
        } catch (InvalidRotationMatrixException ignore) {
            return null;
        }
    }
    
    /**
     * Sets 3D rotation for this transformation.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     * @throws AlgebraException raised if for numerical reasons rotation cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation).
     */
    public void setRotation(Rotation3D rotation) throws NullPointerException,
            AlgebraException {
        Matrix rotMatrix = rotation.asInhomogeneousMatrix();
        
        //Use QR decomposition to retrieve parameters matrix
        RQDecomposer decomposer = new RQDecomposer(A);
        decomposer.decompose();
        Matrix localA = decomposer.getR(); //retrieves params matrix
        localA.multiply(rotMatrix);
        this.A = localA;
    }
    
    /**
     * Adds provided rotation to current rotation assigned to this 
     * transformation.
     * @param rotation 3D rotation to be added.
     * @throws AlgebraException raised if for numerical reasons rotation cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation).
     */
    public void addRotation(Rotation3D rotation) throws AlgebraException{
        Rotation3D localRotation = getRotation();
        localRotation.combine(rotation);
        setRotation(localRotation);
    }
    
    /**
     * Sets scale of this transformation.
     * @param scale acale value to be set. A value between 0.0 and 1.0 indicates
     * that objects will be reduced, a value greater than 1.0 indicates that 
     * objects will be enlarged, and a negative value indicates that objects
     * will be reversed.
     * @throws AlgebraException raised if for numerical reasons scale cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation)
     */
    public void setScale(double scale) throws AlgebraException {
        
        RQDecomposer decomposer = new RQDecomposer(A);
        decomposer.decompose();
        Matrix localA = decomposer.getR(); //params
        localA.setElementAt(0, 0, scale);
        localA.setElementAt(1, 1, scale);
        localA.multiply(decomposer.getQ()); //multiply by rotation
        this.A = localA;
    }
    
    /**
     * Gets affine parameters of this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     * @return affine parameters.
     * @throws AlgebraException raised if for numerical reasons affine 
     * parameters cannot be retrieved (usually because of numerical instability
     * in matrix A).
     */
    public AffineParameters3D getParameters() throws AlgebraException {
        AffineParameters3D parameters = new AffineParameters3D();
        getParameters(parameters);
        return parameters;
    }
    
    /**
     * Computes affine parameters of this instance and stores the result in
     * provided instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     * @param result instance where affine parameters will be stored.
     * @throws AlgebraException raised if for numerical reasons affine 
     * parameters cannot be retrieved (usually because of numerical instability
     * in matrix A).
     */
    public void getParameters(AffineParameters3D result) 
            throws AlgebraException {
        RQDecomposer decomposer = new RQDecomposer(A);
        decomposer.decompose();
        Matrix params = decomposer.getR();
        result.fromMatrix(params);
    }
    
    /**
     * Sets affine parameters of this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     * @param parameters affine parameters to be set.
     * @throws AlgebraException raised if for numerical reasons affine 
     * parameters cannot be set (usually because of numerical instability in
     * current matrix A).
     */
    public void setParameters(AffineParameters3D parameters) 
            throws AlgebraException {
        RQDecomposer decomposer = new RQDecomposer(A);
        decomposer.decompose();
        Matrix params = parameters.asMatrix();
        Matrix rotation = decomposer.getQ();
        
        params.multiply(rotation);
        A = params;
    }
        
    /**
     * Returns 3D translation assigned to this transformation as an array 
     * expressed in inhomogeneous coordinates.
     * @return 3D translation array.
     */
    public double[] getTranslation() {
        return translation;
    }
    
    /**
     * Sets 3D translation assigned to this transformation as an array expressed
     * in inhomogeneous coordinates.
     * @param translation 3D translation array.
     * @throws IllegalArgumentException Raised if provided array does not have
     * length equal to NUM_TRANSLATION_COORDS.
     */
    public void setTranslation(double[] translation) 
            throws IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        this.translation = translation;
    }
    
    /**
     * Adds provided translation to current translation on this transformation.
     * Provided translation must be expressed as an array of inhomogeneous 
     * coordinates.
     * @param translation 3D translation array.
     * @throws IllegalArgumentException Raised if provided array does not have
     * length equal to NUM_TRANSLATION_COORDS.
     */
    public void addTranslation(double[] translation)
            throws IllegalArgumentException {
        ArrayUtils.sum(this.translation, translation, this.translation);
    }
    
    /**
     * Returns current x coordinate translation assigned to this transformation.
     * @return X coordinate translation.
     */
    public double getTranslationX() {
        return translation[0];
    }
    
    /**
     * Sets x coordinate translation to be made by this transformation.
     * @param translationX X coordinate translation to be set.
     */
    public void setTranslationX(double translationX) {
        translation[0] = translationX;
    }
    
    /**
     * Returns current y coordinate translation assigned to this transformation.
     * @return Y coordinate translation.
     */
    public double getTranslationY() {
        return translation[1];
    }
    
    /**
     * Sets y coordinate translation to be made by this transformation.
     * @param translationY Y coordinate translation to be set.
     */
    public void setTranslationY(double translationY) {
        translation[1] = translationY;
    }

    /**
     * Returns current z coordinate translation assigned to this transformation.
     * @return Z coordinate translation.
     */
    public double getTranslationZ() {
        return translation[2];
    }
    
    /**
     * Sets z coordinate translation to be made by this transformation.
     * @param translationZ z coordinate translation to be set.
     */
    public void setTranslationZ(double translationZ) {
        translation[2] = translationZ;
    }
    
    /**
     * Sets x, y, z coordinates of translation to be made by this 
     * transformation.
     * @param translationX translation x coordinate to be set.
     * @param translationY translation y coordinate to be set.
     * @param translationZ translation z coordinate to be set.
     */
    public void setTranslation(double translationX, double translationY,
            double translationZ) {
        translation[0] = translationX;
        translation[1] = translationY;
        translation[2] = translationZ;
    }
    
    /**
     * Sets x, y, z coordinates of translation to be made by this 
     * transformation.
     * @param translation translation to be set.
     */
    public void setTranslation(Point3D translation) {
        setTranslation(translation.getInhomX(), translation.getInhomY(),
                translation.getInhomZ());
    }
    
    /**
     * Gets x, y, z coordinates of translation to be made by this transformation
     * as a new point.
     * @return a new point containing translation coordinates.
     */
    public Point3D getTranslationPoint() {
        Point3D out = Point3D.create();
        getTranslationPoint(out);
        return out;
    }
    
    /**
     * Gets x, y, z coordinates of translation to be made by this transformation
     * and stores them into provided point.
     * @param out point where translation coordinates will be stored.
     */
    public void getTranslationPoint(Point3D out) {
        out.setInhomogeneousCoordinates(translation[0], translation[1], 
                translation[2]);
    }
    
    /**
     * Adds provided x coordinate to current translation assigned to this 
     * transformation.
     * @param translationX X coordinate to be added to current translation.
     */
    public void addTranslationX(double translationX) {
        translation[0] += translationX;
    }
    
    /**
     * Adds provided y coordinate to current translation assigned to this
     * transformation.
     * @param translationY Y coordinate to be added to current translation.
     */
    public void addTranslationY(double translationY) {
        translation[1] += translationY;
    }
 
   /**
     * Adds provided z coordinate to current translation assigned to this
     * transformation.
     * @param translationZ Z coordinate to be added to current translation.
     */
    public void addTranslationZ(double translationZ) {
        translation[2] += translationZ;
    }   
    
    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     * @param translationX x coordinate to be added to current translation.
     * @param translationY y coordinate to be added to current translation.
     * @param translationZ z coordinate to be added to current translation.
     */
    public void addTranslation(double translationX, double translationY,
            double translationZ) {
        translation[0] += translationX;
        translation[1] += translationY;
        translation[2] += translationZ;
    }
    
    /**
     * Adds provided coordinates to current translation assigned to this 
     * transformation.
     * @param translation x, y, z coordinates to be added to current 
     * translation.
     */
    public void addTranslation(Point3D translation) {
        addTranslation(translation.getInhomX(), translation.getInhomY(),
                translation.getInhomZ());
    }
    
    /**
     * Represents this transformation as a 4x4 matrix.
     * A point can be transformed as T * p, where T is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     * @return This transformation in matrix form.
     */
    @Override
    public Matrix asMatrix() {
        Matrix m = null;
        try {
            m = new Matrix(HOM_COORDS, HOM_COORDS);
            asMatrix(m);
        } catch (WrongSizeException ignore) { }
        return m;
    }
    
    /**
     * Represents this transformation as a 4x4 matrix and stores the result in
     * provided instance.
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException raised if provided instance is not a 4x4 
     * matrix.
     */
    @Override
    public void asMatrix(Matrix m) throws IllegalArgumentException {
        if(m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }
        
        //set rotation        
        m.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, A);
        
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
        try {
            double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            coords[0] = inputPoint.getInhomX();
            coords[1] = inputPoint.getInhomY();
            coords[2] = inputPoint.getInhomZ();
            
            Matrix p = A.multiplyAndReturnNew(Matrix.newFromArray(coords, 
                    true));
        
            outputPoint.setInhomogeneousCoordinates(
                    p.getElementAtIndex(0) + translation[0], 
                    p.getElementAtIndex(1) + translation[1],
                    p.getElementAtIndex(2) + translation[2]);
        } catch (WrongSizeException ignore) { } //this exception will never be 
                                            //raised
    }

    /**
     * Transforms a quadric using this transformation and stores the result into
     * provided output quadric.
     * @param inputQuadric quadric to be transformed.
     * @param outputQuadric instance where data of transformed quadric will be 
     * stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output conic matrix is not considered to be symmetric.
     */        
    @Override
    public void transform(Quadric inputQuadric, Quadric outputQuadric) 
            throws NonSymmetricMatrixException {
        //p' * C * p = 0
        //p'*T' * C * T * p = 0
        
        inputQuadric.normalize();
        
        Matrix C = inputQuadric.asMatrix();
        Matrix T = asMatrix();
        //normalize transformation matrix T to increase accuracy
        double norm = Utils.normF(T);
        T.multiplyByScalar(1.0 / norm);
        
        Matrix m = T.transposeAndReturnNew();
        try {
            m.multiply(C);
            m.multiply(T);
        } catch (WrongSizeException ignore) { }
        
        //normalize resulting m matrix to increase accuracy so that it can be
        //considered symmetric
        norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        outputQuadric.setParameters(m);
    }

    /**
     * Transforms a dual quadric using this transformation and stores the result
     * into provided output dual quadric.
     * @param inputDualQuadric dual quadric to be transformed.
     * @param outputDualQuadric instance where data of transformed dual quadric 
     * will be stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output dual conic matrix is not considered to be symmetric.
     * @throws AlgebraException raised if transformcannot be computed becauseof 
     * numerical instabilities.
     */        
    @Override
    public void transform(DualQuadric inputDualQuadric, 
            DualQuadric outputDualQuadric) throws NonSymmetricMatrixException, 
            AlgebraException {
        //l' * C¨* l = 0
        //l'*(T^-1)' * C ¨* (T^-1) * p = 0
        
        inputDualQuadric.normalize();
        
        Matrix dualC = inputDualQuadric.asMatrix();
        Matrix invT = inverseAndReturnNew().asMatrix();
        //normalize transformation matrix T to increase accuracy
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        
        Matrix m = invT.transposeAndReturnNew();
        m.multiply(dualC);
        m.multiply(invT);
        
        //normalize resulting m matrix to increase accuracy so that it can be
        //considered symmetric
        norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        outputDualQuadric.setParameters(m);
    }

    /**
     * Transforms provided input plane using this transformation and stores the
     * result into provided output plane instance.
     * @param inputPlane plane to be transformed.
     * @param outputPlane instance where data of transformed plane will be 
     * stored.
     * @throws AlgebraException raised if transformAndReturnNew cannot be computed because
 of numerical instabilities.
     */        
    @Override
    public void transform(Plane inputPlane, Plane outputPlane) 
            throws AlgebraException {
        //p' * l = 0 --> (T*p)' * (T^-1) * l = p'*T'*(T^-1)*l = 0
        
        inputPlane.normalize();
        
        Matrix invT = inverseAndReturnNew().asMatrix();        
        Matrix l = Matrix.newFromArray(inputPlane.asArray());
        
        Matrix m = invT;
        m.multiply(l);
        
        outputPlane.setParameters(m.toArray());
    }
    
    /**
     * Transforms a camera using this transformation and stores the result into
     * provided output camera.
     * @param inputCamera camera to be transformed.
     * @param outputCamera instance where data of transforeed camera will be 
     * stored.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */    
    @Override
    public void transform(PinholeCamera inputCamera,
            PinholeCamera outputCamera) throws AlgebraException {
        
        inputCamera.normalize();
        
        Matrix invT = inverseAndReturnNew().asMatrix();
        Matrix c = inputCamera.getInternalMatrix();
        c.multiply(invT);
        outputCamera.setInternalMatrix(c);
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
    public Transformation3D inverseAndReturnNew() throws AlgebraException {
        AffineTransformation3D result = new AffineTransformation3D();
        inverse(result);
        return result;
    }
    
    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     * @param result instance where inverse transformation will be stored.
     * @throws AlgebraException if inverse transform cannot be computed because
     * of numerical instabilities.
     */
    protected void inverse(AffineTransformation3D result) 
            throws AlgebraException {
        
        //x' = A * x + t -->
        //A^-1 * x' = A^-1 * A * x + A^-1 * t = x + A^-1 * t -->
        //x = A^-1 * x' - A^-1 * t
        
        try {
            //reverse rotation
            Matrix invA = Utils.inverse(A);
            result.A = invA;
        
            //reverse translation
            Matrix t = Matrix.newFromArray(translation, true);
            t.multiplyByScalar(-1.0);
            
            Matrix resultT = invA.multiplyAndReturnNew(t);
            result.translation = resultT.toArray();
        } catch (WrongSizeException ignore) { }
    }    
    
    /**
     * Converts this transformation into a metric transformation.
     * @return This transformation converted into a projective transformation.
     */
    public ProjectiveTransformation3D toProjective() {
        return new ProjectiveTransformation3D(A, translation);
    }    
    
    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided transformation.
     * @param transformation Transformation to be combined with.
     */
    public void combine(AffineTransformation3D transformation) {
        combine(transformation, this);
    }
    
    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     * @param transformation Transformation to be combined with.
     * @return A new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    public AffineTransformation3D combineAndReturnNew(
            AffineTransformation3D transformation) {
        
        AffineTransformation3D result = new AffineTransformation3D();
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
    private void combine(AffineTransformation3D inputTransformation, 
            AffineTransformation3D outputTransformation) {
        //combination in matrix representation is:
        //[A1 t1] * [A2 t2] = [A1*A2 + t1*0T  A1*t2 + t1*1] = [A1*A2 A1*t2 + t1]
        //[0T 1 ]   [0T 1 ]   [0T*A2 + 1*0T   0T*t2 + 1*1 ]   [0T    1         ]
        
        try {
            //we do translation first, because this.rotation might change later
            Matrix A1 = this.A.clone();
            Matrix t2 = Matrix.newFromArray(inputTransformation.translation, true);
            A1.multiply(t2); //this is R1 * t2
                  
            ArrayUtils.sum(A1.toArray(), this.translation,  
                    outputTransformation.translation);

            outputTransformation.A = this.A.multiplyAndReturnNew(
                    inputTransformation.A);
        
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
    public final void setTransformationFromPoints(Point3D inputPoint1, 
            Point3D inputPoint2, Point3D inputPoint3, Point3D inputPoint4,
            Point3D outputPoint1, Point3D outputPoint2, Point3D outputPoint3,
            Point3D outputPoint4) throws CoincidentPointsException {
        
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
        //There are 13 unknowns and 12 equations (3 for each pair of 
        //corresponding points)
        Matrix m = null;
        try {
            m = new Matrix(12, 13); //build matrix initialized to zero
        } catch (WrongSizeException ignore) { }
        
        //1st pair of points
        double iX = inputPoint1.getHomX();
        double iY = inputPoint1.getHomY();
        double iZ = inputPoint1.getHomZ();
        double iW = inputPoint1.getHomW();
        
        double oX = outputPoint1.getHomX();
        double oY = outputPoint1.getHomY();
        double oZ = outputPoint1.getHomZ();
        double oW = outputPoint1.getHomW();
        
        double oWiX = oW * iX;
        double oWiY = oW * iY;
        double oWiZ = oW * iZ;
        double oWiW = oW * iW;
        
        double oXiW = oX * iW;
        double oYiW = oY * iW;
        double oZiW = oZ * iW;
                
        double norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oXiW * oXiW);
        
        m.setElementAt(0, 0, oWiX / norm);
        m.setElementAt(0, 1, oWiY / norm);
        m.setElementAt(0, 2, oWiZ / norm);
        m.setElementAt(0, 9, oWiW / norm);        
        m.setElementAt(0, 12, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oYiW * oYiW);
        
        m.setElementAt(1, 3, oWiX / norm);
        m.setElementAt(1, 4, oWiY / norm);
        m.setElementAt(1, 5, oWiZ / norm);
        m.setElementAt(1, 10, oWiW / norm);        
        m.setElementAt(1, 12, -oYiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oZiW * oZiW);
        
        m.setElementAt(2, 6, oWiX / norm);
        m.setElementAt(2, 7, oWiY / norm);
        m.setElementAt(2, 8, oWiZ / norm);
        m.setElementAt(2, 11, oWiW / norm);        
        m.setElementAt(2, 12, -oZiW / norm);
        
        
        //2nd pair of points
        iX = inputPoint2.getHomX();
        iY = inputPoint2.getHomY();
        iZ = inputPoint2.getHomZ();
        iW = inputPoint2.getHomW();
        
        oX = outputPoint2.getHomX();
        oY = outputPoint2.getHomY();
        oZ = outputPoint2.getHomZ();
        oW = outputPoint2.getHomW();
        
        oWiX = oW * iX;
        oWiY = oW * iY;
        oWiZ = oW * iZ;
        oWiW = oW * iW;
        
        oXiW = oX * iW;
        oYiW = oY * iW;
        oZiW = oZ * iW;
                
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oXiW * oXiW);
        
        m.setElementAt(3, 0, oWiX / norm);
        m.setElementAt(3, 1, oWiY / norm);
        m.setElementAt(3, 2, oWiZ / norm);
        m.setElementAt(3, 9, oWiW / norm);        
        m.setElementAt(3, 12, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oYiW * oYiW);
        
        m.setElementAt(4, 3, oWiX / norm);
        m.setElementAt(4, 4, oWiY / norm);
        m.setElementAt(4, 5, oWiZ / norm);
        m.setElementAt(4, 10, oWiW / norm);        
        m.setElementAt(4, 12, -oYiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oZiW * oZiW);
        
        m.setElementAt(5, 6, oWiX / norm);
        m.setElementAt(5, 7, oWiY / norm);
        m.setElementAt(5, 8, oWiZ / norm);
        m.setElementAt(5, 11, oWiW / norm);        
        m.setElementAt(5, 12, -oZiW / norm);
        
        
        //3rd pair of points
        iX = inputPoint3.getHomX();
        iY = inputPoint3.getHomY();
        iZ = inputPoint3.getHomZ();
        iW = inputPoint3.getHomW();
        
        oX = outputPoint3.getHomX();
        oY = outputPoint3.getHomY();
        oZ = outputPoint3.getHomZ();
        oW = outputPoint3.getHomW();
        
        oWiX = oW * iX;
        oWiY = oW * iY;
        oWiZ = oW * iZ;
        oWiW = oW * iW;
        
        oXiW = oX * iW;
        oYiW = oY * iW;
        oZiW = oZ * iW;
                
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oXiW * oXiW);
        
        m.setElementAt(6, 0, oWiX / norm);
        m.setElementAt(6, 1, oWiY / norm);
        m.setElementAt(6, 2, oWiZ / norm);
        m.setElementAt(6, 9, oWiW / norm);        
        m.setElementAt(6, 12, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oYiW * oYiW);
        
        m.setElementAt(7, 3, oWiX / norm);
        m.setElementAt(7, 4, oWiY / norm);
        m.setElementAt(7, 5, oWiZ / norm);
        m.setElementAt(7, 10, oWiW / norm);        
        m.setElementAt(7, 12, -oYiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oZiW * oZiW);
        
        m.setElementAt(8, 6, oWiX / norm);
        m.setElementAt(8, 7, oWiY / norm);
        m.setElementAt(8, 8, oWiZ / norm);
        m.setElementAt(8, 11, oWiW / norm);        
        m.setElementAt(8, 12, -oZiW / norm);
        

        //4th pair of points
        iX = inputPoint4.getHomX();
        iY = inputPoint4.getHomY();
        iZ = inputPoint4.getHomZ();
        iW = inputPoint4.getHomW();
        
        oX = outputPoint4.getHomX();
        oY = outputPoint4.getHomY();
        oZ = outputPoint4.getHomZ();
        oW = outputPoint4.getHomW();
        
        oWiX = oW * iX;
        oWiY = oW * iY;
        oWiZ = oW * iZ;
        oWiW = oW * iW;
        
        oXiW = oX * iW;
        oYiW = oY * iW;
        oZiW = oZ * iW;
                
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oXiW * oXiW);
        
        m.setElementAt(9, 0, oWiX / norm);
        m.setElementAt(9, 1, oWiY / norm);
        m.setElementAt(9, 2, oWiZ / norm);
        m.setElementAt(9, 9, oWiW / norm);        
        m.setElementAt(9, 12, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oYiW * oYiW);
        
        m.setElementAt(10, 3, oWiX / norm);
        m.setElementAt(10, 4, oWiY / norm);
        m.setElementAt(10, 5, oWiZ / norm);
        m.setElementAt(10, 10, oWiW / norm);        
        m.setElementAt(10, 12, -oYiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                oWiW * oWiW + oZiW * oZiW);
        
        m.setElementAt(11, 6, oWiX / norm);
        m.setElementAt(11, 7, oWiY / norm);
        m.setElementAt(11, 8, oWiZ / norm);
        m.setElementAt(11, 11, oWiW / norm);        
        m.setElementAt(11, 12, -oZiW / norm);
                                
        //use SVD to decompose matrix m
        Matrix V;
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            //ensure that matrix m has enough rank and there is a unique 
            //solution (up to scale)
            if(decomposer.getRank() < 12) throw new CoincidentPointsException();
            V = decomposer.getV(); //V is 13x13
            
            //last column of V will contain parameters of transformation
            double value = V.getElementAt(12, 12);
            A.setElementAt(0, 0, V.getElementAt(0, 12) / value);
            A.setElementAt(0, 1, V.getElementAt(1, 12) / value);
            A.setElementAt(0, 2, V.getElementAt(2, 12) / value);
            A.setElementAt(1, 0, V.getElementAt(3, 12) / value);
            A.setElementAt(1, 1, V.getElementAt(4, 12) / value);
            A.setElementAt(1, 2, V.getElementAt(5, 12) / value);
            A.setElementAt(2, 0, V.getElementAt(6, 12) / value);
            A.setElementAt(2, 1, V.getElementAt(7, 12) / value);
            A.setElementAt(2, 2, V.getElementAt(8, 12) / value);
            
            translation[0] = V.getElementAt(9, 12) / value;
            translation[1] = V.getElementAt(10, 12) / value;
            translation[2] = V.getElementAt(11, 12) / value;
                        
        } catch (AlgebraException e) {
            throw new CoincidentPointsException(e);
        }        
    }        
    
   /**
     * Estimates this transformation internal parameters by using 4 
     * corresponding original and transformed planes.
     * @param inputPlane1 1st input plane.
     * @param inputPlane2 2nd input plane.
     * @param inputPlane3 3rd input plane.
     * @param inputPlane4 4th input plane.
     * @param outputPlane1 1st transformed plane corresponding to 1st input 
     * plane.
     * @param outputPlane2 2nd transformed plane corresponding to 2nd input
     * plane.
     * @param outputPlane3 3rd transformed plane corresponding to 3rd input
     * plane.
     * @param outputPlane4 4th transformed plane corresponding to 4th input
     * plane.
     * @throws CoincidentPlanesException raised if transformation cannot be
     * estimated for some reason (plane configuration degeneracy, duplicate 
     * points or numerical instabilities).
     */    
    public final void setTransformationFromPlanes(Plane inputPlane1, 
            Plane inputPlane2, Plane inputPlane3, Plane inputPlane4,
            Plane outputPlane1, Plane outputPlane2, Plane outputPlane3,
            Plane outputPlane4) throws CoincidentPlanesException {
        
        //normalize points to increase accuracy
        inputPlane1.normalize();
        inputPlane2.normalize();
        inputPlane3.normalize();
        inputPlane4.normalize();
        
        outputPlane1.normalize();
        outputPlane2.normalize();
        outputPlane3.normalize();
        outputPlane4.normalize();
        
        //matrix of homogeneous linear system of equations.
        //There are 13 unknowns and 12 equations (3 for each pair of 
        //corresponding points)
        Matrix m = null;
        try {
            m = new Matrix(12, 13); //build matrix initialized to zero
        } catch (WrongSizeException ignore) { }
        
        //1st pair of planes
        double iA = inputPlane1.getA();
        double iB = inputPlane1.getB();
        double iC = inputPlane1.getC();
        double iD = inputPlane1.getD();
        
        double oA = outputPlane1.getA();
        double oB = outputPlane1.getB();
        double oC = outputPlane1.getC();
        double oD = outputPlane1.getD();
        
        double oDiA = oD * iA;
        double oDiB = oD * iB;
        double oDiC = oD * iC;
        double oDiD = oD * iD;
        
        double oAiD = oA * iD;
        double oBiD = oB * iD;
        double oCiD = oC * iD;
                
        double norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oAiD * oAiD);
        
        m.setElementAt(0, 0, oDiA / norm);
        m.setElementAt(0, 1, oDiB / norm);
        m.setElementAt(0, 2, oDiC / norm);
        m.setElementAt(0, 9, oDiD / norm);        
        m.setElementAt(0, 12, -oAiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oBiD * oBiD);
        
        m.setElementAt(1, 3, oDiA / norm);
        m.setElementAt(1, 4, oDiB / norm);
        m.setElementAt(1, 5, oDiC / norm);
        m.setElementAt(1, 10, oDiD / norm);        
        m.setElementAt(1, 12, -oBiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oCiD * oCiD);
        
        m.setElementAt(2, 6, oDiA / norm);
        m.setElementAt(2, 7, oDiB / norm);
        m.setElementAt(2, 8, oDiC / norm);
        m.setElementAt(2, 11, oDiD / norm);        
        m.setElementAt(2, 12, -oCiD / norm);
        
        
        //2nd pair of planes
        iA = inputPlane2.getA();
        iB = inputPlane2.getB();
        iC = inputPlane2.getC();
        iD = inputPlane2.getD();
        
        oA = outputPlane2.getA();
        oB = outputPlane2.getB();
        oC = outputPlane2.getC();
        oD = outputPlane2.getD();
        
        oDiA = oD * iA;
        oDiB = oD * iB;
        oDiC = oD * iC;
        oDiD = oD * iD;
        
        oAiD = oA * iD;
        oBiD = oB * iD;
        oCiD = oC * iD;
                
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oAiD * oAiD);
        
        m.setElementAt(3, 0, oDiA / norm);
        m.setElementAt(3, 1, oDiB / norm);
        m.setElementAt(3, 2, oDiC / norm);
        m.setElementAt(3, 9, oDiD / norm);        
        m.setElementAt(3, 12, -oAiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oBiD * oBiD);
        
        m.setElementAt(4, 3, oDiA / norm);
        m.setElementAt(4, 4, oDiB / norm);
        m.setElementAt(4, 5, oDiC / norm);
        m.setElementAt(4, 10, oDiD / norm);        
        m.setElementAt(4, 12, -oBiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oCiD * oCiD);
        
        m.setElementAt(5, 6, oDiA / norm);
        m.setElementAt(5, 7, oDiB / norm);
        m.setElementAt(5, 8, oDiC / norm);
        m.setElementAt(5, 11, oDiD / norm);        
        m.setElementAt(5, 12, -oCiD / norm);
        
        
        //3rd pair of planes
        iA = inputPlane3.getA();
        iB = inputPlane3.getB();
        iC = inputPlane3.getC();
        iD = inputPlane3.getD();
        
        oA = outputPlane3.getA();
        oB = outputPlane3.getB();
        oC = outputPlane3.getC();
        oD = outputPlane3.getD();
        
        oDiA = oD * iA;
        oDiB = oD * iB;
        oDiC = oD * iC;
        oDiD = oD * iD;
        
        oAiD = oA * iD;
        oBiD = oB * iD;
        oCiD = oC * iD;
                
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oAiD * oAiD);
        
        m.setElementAt(6, 0, oDiA / norm);
        m.setElementAt(6, 1, oDiB / norm);
        m.setElementAt(6, 2, oDiC / norm);
        m.setElementAt(6, 9, oDiD / norm);        
        m.setElementAt(6, 12, -oAiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oBiD * oBiD);
        
        m.setElementAt(7, 3, oDiA / norm);
        m.setElementAt(7, 4, oDiB / norm);
        m.setElementAt(7, 5, oDiC / norm);
        m.setElementAt(7, 10, oDiD / norm);        
        m.setElementAt(7, 12, -oBiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oCiD * oCiD);
        
        m.setElementAt(8, 6, oDiA / norm);
        m.setElementAt(8, 7, oDiB / norm);
        m.setElementAt(8, 8, oDiC / norm);
        m.setElementAt(8, 11, oDiD / norm);        
        m.setElementAt(8, 12, -oCiD / norm);
        

        //4th pair of planes
        iA = inputPlane4.getA();
        iB = inputPlane4.getB();
        iC = inputPlane4.getC();
        iD = inputPlane4.getD();
        
        oA = outputPlane4.getA();
        oB = outputPlane4.getB();
        oC = outputPlane4.getC();
        oD = outputPlane4.getD();
        
        oDiA = oD * iA;
        oDiB = oD * iB;
        oDiC = oD * iC;
        oDiD = oD * iD;
        
        oAiD = oA * iD;
        oBiD = oB * iD;
        oCiD = oC * iD;
                
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oAiD * oAiD);
        
        m.setElementAt(9, 0, oDiA / norm);
        m.setElementAt(9, 1, oDiB / norm);
        m.setElementAt(9, 2, oDiC / norm);
        m.setElementAt(9, 9, oDiD / norm);        
        m.setElementAt(9, 12, -oAiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oBiD * oBiD);
        
        m.setElementAt(10, 3, oDiA / norm);
        m.setElementAt(10, 4, oDiB / norm);
        m.setElementAt(10, 5, oDiC / norm);
        m.setElementAt(10, 10, oDiD / norm);        
        m.setElementAt(10, 12, -oBiD / norm);
        
        norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                oDiD * oDiD + oCiD * oCiD);
        
        m.setElementAt(11, 6, oDiA / norm);
        m.setElementAt(11, 7, oDiB / norm);
        m.setElementAt(11, 8, oDiC / norm);
        m.setElementAt(11, 11, oDiD / norm);        
        m.setElementAt(11, 12, -oCiD / norm);
                                
        //use SVD to decompose matrix m
        Matrix V;
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            //ensure that matrix m has enough rank and there is a unique 
            //solution (up to scale)
            if(decomposer.getRank() < 12) throw new CoincidentPlanesException();
            V = decomposer.getV(); //V is 13x13
            
            //last column of V will contain parameters of transformation
            double value = V.getElementAt(12, 12);
            AffineTransformation3D transformation = new AffineTransformation3D();
            transformation.A.setElementAt(0, 0, V.getElementAt(0, 12) / value);
            transformation.A.setElementAt(0, 1, V.getElementAt(1, 12) / value);
            transformation.A.setElementAt(0, 2, V.getElementAt(2, 12) / value);
            transformation.A.setElementAt(1, 0, V.getElementAt(3, 12) / value);
            transformation.A.setElementAt(1, 1, V.getElementAt(4, 12) / value);
            transformation.A.setElementAt(1, 2, V.getElementAt(5, 12) / value);
            transformation.A.setElementAt(2, 0, V.getElementAt(6, 12) / value);
            transformation.A.setElementAt(2, 1, V.getElementAt(7, 12) / value);
            transformation.A.setElementAt(2, 2, V.getElementAt(8, 12) / value);
            
            transformation.translation[0] = V.getElementAt(9, 12) / value;
            transformation.translation[1] = V.getElementAt(10, 12) / value;
            transformation.translation[2] = V.getElementAt(11, 12) / value;
                        
            //planes compute inverse transformation, so we need to inverse it
            //to get the right solution
            transformation.inverse(); //(we do it on another instance in case 
            //that transformation inversion fails, so that original A and
            //translation get preserved
            
            this.A = transformation.A;
            this.translation = transformation.translation;                        
        } catch (AlgebraException e) {
            throw new CoincidentPlanesException(e);
        }             
    }        
    
    /**
     * Estimates this transformation internal parameters by using provided 2 
     * corresponding original and transformed lines.
     * @param inputLine1 1st input line.
     * @param inputLine2 2nd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @throws CoincidentLinesException Raised if transformation cannot be
     * estimated for some reason (line configuration degeneracy, duplicate lines
     * or numerical instabilities).
     */
    public final void setTransformationFromLines(Line3D inputLine1, 
            Line3D inputLine2, Line3D outputLine1, Line3D outputLine2) 
            throws CoincidentLinesException {
        try {
            setTransformationFromPlanes(inputLine1.getPlane1(), 
                    inputLine1.getPlane2(), inputLine2.getPlane1(), 
                    inputLine2.getPlane2(), outputLine1.getPlane1(), 
                    outputLine1.getPlane2(), outputLine2.getPlane1(), 
                    outputLine2.getPlane2());
        } catch (CoincidentPlanesException e) {
            throw new CoincidentLinesException(e);
        }
    }    
}
