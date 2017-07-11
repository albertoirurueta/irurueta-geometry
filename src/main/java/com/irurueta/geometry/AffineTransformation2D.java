/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.AffineTransformation2D
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
 * This class performs affine transformations on 2D space.
 * Affine transformations include transformations related to rotations,
 * translations, independently scaling horizontal or vertical coordinates
 * or skewing the coordinates axis.
 * This class is not intended to be used on points located at infinity or
 * at very large coordinates, since numerical instabilities may occur. For
 * those cases use a ProjectiveTransformation2D instead.
 */
public class AffineTransformation2D extends Transformation2D 
        implements Serializable {

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
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 2;
    
    
    /**
     * Constant defining number of inhomogeneous coordinates in 2D space.
     */
    public static final int INHOM_COORDS = 2;
    
    /**
     * Constant defining number of homogeneous coordinates in 2D space.
     */
    public static final int HOM_COORDS = 3;
    
    
    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public AffineTransformation2D() {
        super();
        try {
            A = Matrix.identity(INHOM_COORDS, INHOM_COORDS);
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided linear mapping matrix.
     * @param A linear mapping.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation2D(Matrix A)
            throws NullPointerException, IllegalArgumentException {
        setA(A);
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided scale value.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     */
    public AffineTransformation2D(double scale) {
        double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        A = Matrix.diagonal(diag);
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided rotation.
     * @param rotation a 2D rotation.
     * @throws NullPointerException Raised if provided rotation is null.
     */
    public AffineTransformation2D(Rotation2D rotation) 
            throws NullPointerException {
        A = rotation.asInhomogeneousMatrix();
        translation = new double[NUM_TRANSLATION_COORDS];
    }
    
    /**
     * Creates transformation with provided scale and rotation.
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     * values greater than 1.0 enlarge objects and negative values reverse
     * objects.
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation2D(double scale, Rotation2D rotation)
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
     * @param params affine parameters including horizontal scaling, vertical 
     * scaling and skewness.
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided parameters are null or
     * if provided rotation is null.
     */
    public AffineTransformation2D(AffineParameters2D params, 
            Rotation2D rotation) throws NullPointerException{
        A = params.asMatrix();
        try {
            A.multiply(rotation.asInhomogeneousMatrix());
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];                
    }
        
    /**
     * Creates transformation with provided 2D translation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     * to NUM_TRANSLATION_COORDS.
     */
    public AffineTransformation2D(double[] translation) 
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
     * Creates transformation with provided linear mapping and translation.
     * @param A linear mapping.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided array is null or if
     * linear mapping is null.
     * @throws IllegalArgumentException raised if length of array is not equal 
     * to NUM_TRANSLATION_COORDS.
     */
    public AffineTransformation2D(Matrix A, double[] translation) 
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        this.translation = translation;
        
        setA(A);
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
    public AffineTransformation2D(double scale, double[] translation)
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
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided rotation or translation
     * is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2.
     */
    public AffineTransformation2D(Rotation2D rotation, double[] translation)
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        
        A = rotation.asInhomogeneousMatrix();        
        this.translation = translation;                
    }
    
    /**
     * Creates transformation with provided scale, rotation and translation
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
    public AffineTransformation2D(double scale, Rotation2D rotation, 
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
     * @param rotation a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     * coordinates.
     * @throws NullPointerException raised if provided parameters, rotation or
     * translation is null.
     * @throws IllegalArgumentException raised if provided translation does not
     * have length 2.
     */
    public AffineTransformation2D(AffineParameters2D params, 
            Rotation2D rotation, double[] translation) 
            throws NullPointerException, IllegalArgumentException {
        if(translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        A = params.asMatrix();
        try {
            A.multiply(rotation.asInhomogeneousMatrix());
        } catch(WrongSizeException ignore) { }
        this.translation = translation;                
    }
    
   /**
     * Creates transformation by estimating its internal values using provided 3 
     * corresponding original and transformed points.
     * @param inputPoint1 1st input point.
     * @param inputPoint2 2nd input point.
     * @param inputPoint3 3rd input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input 
     * point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     * point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     * point.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate 
     * points or numerical instabilities).
     */    
    public AffineTransformation2D(Point2D inputPoint1, Point2D inputPoint2,
            Point2D inputPoint3, Point2D outputPoint1, Point2D outputPoint2, 
            Point2D outputPoint3) throws CoincidentPointsException {
        try {
            A = new Matrix(INHOM_COORDS, INHOM_COORDS);
        } catch (WrongSizeException ignore) { }
        translation = new double[NUM_TRANSLATION_COORDS];
        setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, 
                outputPoint1, outputPoint2, outputPoint3);
    }    
    
    /**
     * Creates transformation by estimating its internal values using provided 3
     * corresponding original and transformed lines.
     * @param inputLine1 1st input line.
     * @param inputLine2 2nd input line.
     * @param inputLine3 3rd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @param outputLine3 3rd transformed line corresponding to 3rd input line.
     * @throws CoincidentLinesException raised if transformation cannot be
     * estimated for some reason (line configuration degeneracy, duplicate lines
     * or numerical instabilities).
     */
    public AffineTransformation2D(Line2D inputLine1, Line2D inputLine2,
            Line2D inputLine3, Line2D outputLine1, Line2D outputLine2,
            Line2D outputLine3) throws CoincidentLinesException {
        setTransformationFromLines(inputLine1, inputLine2, inputLine3, 
                outputLine1, outputLine2, outputLine3);
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
     * size 2x2.
     */
    public final void setA(Matrix A) throws NullPointerException, 
            IllegalArgumentException {
        if (A == null) {
            throw new NullPointerException();
        }
        if(A.getRows() != INHOM_COORDS || A.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        this.A = A;
    }
    
    /**
     * Returns 2D rotation assigned to this transformation.
     * Note: if this rotation instance is modified, its changes won't be 
     * reflected on this instance until rotation is set again.
     * @return 2D rotation.
     * @throws AlgebraException if for some reason rotation cannot
     * be estimated (usually because of numerical instability).
     */
    public Rotation2D getRotation() throws AlgebraException {
        //Use QR decomposition to retrieve rotation
        RQDecomposer decomposer = new RQDecomposer(A);
        try {
            decomposer.decompose();
            return new Rotation2D(decomposer.getQ());
        } catch (InvalidRotationMatrixException ignore) {
            return null;
        }
    }
    
    /**
     * Sets 2D rotation for this transformation.
     * @param rotation A 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     * @throws AlgebraException raised if for numerical reasons rotation cannot
     * be set (usually because of numerical instability in parameters of this
     * transformation).
     */
    public void setRotation(Rotation2D rotation) throws NullPointerException,
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
    public AffineParameters2D getParameters() throws AlgebraException {
        AffineParameters2D parameters = new AffineParameters2D();
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
    public void getParameters(AffineParameters2D result) 
            throws AlgebraException{
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
    public void setParameters(AffineParameters2D parameters) 
            throws AlgebraException {
        RQDecomposer decomposer = new RQDecomposer(A);
        decomposer.decompose();
        Matrix params = parameters.asMatrix();
        Matrix rotation = decomposer.getQ();
        
        params.multiply(rotation);
        A = params;
    }
        
    /**
     * Returns 2D translation assigned to this transformation as an array 
     * expressed in inhomogeneous coordinates.
     * @return 2D translation array.
     */
    public double[] getTranslation() {
        return translation;
    }
    
    /**
     * Sets 2D translation assigned to this transformation as an array expressed
     * in inhomogeneous coordinates
     * @param translation 2D translation array.
     * @throws IllegalArgumentException raised if provided array does not have
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
     * @param translation 2D translation array.
     * @throws IllegalArgumentException raised if provided array does not have
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
     * Sets x, y coordinates of translation to be made by this transformation.
     * @param translationX translation x coordinate to be set.
     * @param translationY translation y coordinate to be set.
     */
    public void setTranslation(double translationX, double translationY) {
        translation[0] = translationX;
        translation[1] = translationY;
    }
    
    /**
     * Sets x, y, coordinates of translation to be made by this transformation.
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
        out.setInhomogeneousCoordinates(translation[0], translation[1]);
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
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     * @param translationX x coordinate to be added to current translation.
     * @param translationY y coordinate to be added to current translation.
     */
    public void addTranslation(double translationX, double translationY) {
        translation[0] += translationX;
        translation[1] += translationY;
    }
    
    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     * @param translation x, y, coordinates to be added to current translation.
     */
    public void addTranslation(Point2D translation) {
        addTranslation(translation.getInhomX(), translation.getInhomY());
    }
    
    /**
     * Represents this transformation as a 3x3 matrix.
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
     * Represents this transformation as a 3x3 matrix and stores the result in
     * provided instance.
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException raised if provided instance is not a 3x3 
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
        for(int i = 0; i < INHOM_COORDS; i++) {
            m.setElementAt(INHOM_COORDS, i, 0.0);
        }
        m.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);
    }       

    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     * @param inputPoint point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */        
    @Override
    public void transform(Point2D inputPoint, Point2D outputPoint) {        
        try {
            double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            coords[0] = inputPoint.getInhomX();
            coords[1] = inputPoint.getInhomY();
            
            Matrix p = A.multiplyAndReturnNew(Matrix.newFromArray(coords, true));
        
            outputPoint.setInhomogeneousCoordinates(
                    p.getElementAtIndex(0) + translation[0], 
                    p.getElementAtIndex(1) + translation[1]);
        } catch (WrongSizeException ignore) { } //this exception will never be 
                                            //raised
    }

    /**
     * Transforms a conic using this transformation and stores the result into
     * provided output conic.
     * @param inputConic conic to be transformed.
     * @param outputConic instance where data of transformed conic will be 
     * stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output conic matrix is not considered to be symmetric.
     */        
    @Override
    public void transform(Conic inputConic, Conic outputConic) 
            throws NonSymmetricMatrixException {
        //p' * C * p = 0
        //p'*T' * C * T * p = 0
        
        inputConic.normalize();
        
        Matrix C = inputConic.asMatrix();
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
        
        outputConic.setParameters(m);
    }

    /**
     * Transforms a dual conic using this transformation and stores the result
     * into provided output dual conic.
     * @param inputDualConic dual conic to be transformed.
     * @param outputDualConic instance where data of transformed dual conic will
     * be stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output dual conic matrix is not considered to be symmetric.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */        
    @Override
    public void transform(DualConic inputDualConic, DualConic outputDualConic) 
            throws NonSymmetricMatrixException, AlgebraException {
        //l' * C¨* l = 0
        //l'*(T^-1)' * C ¨* (T^-1) * p = 0
        
        inputDualConic.normalize();
        
        Matrix dualC = inputDualConic.asMatrix();
        Matrix invT = inverseAndReturnNew().asMatrix();
        //normalize transformation matrix T to increase accuracy
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        
        Matrix m = invT.transposeAndReturnNew();
        try{
            m.multiply(dualC);
            m.multiply(invT);
        }catch(WrongSizeException ignore){}
        
        //normalize resulting m matrix to increase accuracy so that it can be
        //considered symmetric
        norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        outputDualConic.setParameters(m);
    }

    /**
     * Transforms provided input line using this transformation and stores the
     * result into provided output line instance.
     * @param inputLine line to be transformed.
     * @param outputLine instance where data of transformed line will be stored.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */        
    @Override
    public void transform(Line2D inputLine, Line2D outputLine) 
            throws AlgebraException {
        //p' * l = 0 --> (T*p)' * (T^-1) * l = p'*T'*(T^-1)*l = 0
        
        inputLine.normalize();
        
        Matrix invT = inverseAndReturnNew().asMatrix();
        //normalize inverse transform matrix to increase accuracy
        double norm = Utils.normF(invT);
        
        Matrix l = Matrix.newFromArray(inputLine.asArray());
        
        Matrix m = invT;
        try {
            m.multiply(l);        
        } catch (WrongSizeException ignore) { }
        
        outputLine.setParameters(m.toArray());
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
     * @return Inverse transformation.
     * @throws AlgebraException if inverse transform cannot be computed because
     * of numerical instabilities.
     */
    public Transformation2D inverseAndReturnNew() throws AlgebraException {
        AffineTransformation2D result = new AffineTransformation2D();
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
    protected void inverse(AffineTransformation2D result) 
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
    public ProjectiveTransformation2D toProjective() {
        return new ProjectiveTransformation2D(A, translation);
    }
    
    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this 
     * transformation with the matrix of provided transformation.
     * @param transformation Transformation to be combined with.
     */
    public void combine(AffineTransformation2D transformation) {
        combine(transformation, this);
    }
    
    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     * @param transformation Transformation to be combined with
     * @return A new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    public AffineTransformation2D combineAndReturnNew(
            AffineTransformation2D transformation) {
        
        AffineTransformation2D result = new AffineTransformation2D();
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
    private void combine(AffineTransformation2D inputTransformation, 
            AffineTransformation2D outputTransformation) {
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
     * Estimates this transformation internal parameters by using 3 
     * corresponding original and transformed points.
     * @param inputPoint1 1st input point.
     * @param inputPoint2 2nd input point.
     * @param inputPoint3 3rd input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input 
     * point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     * point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     * point.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate 
     * points or numerical instabilities).
     */    
    public final void setTransformationFromPoints(Point2D inputPoint1, 
            Point2D inputPoint2, Point2D inputPoint3, Point2D outputPoint1, 
            Point2D outputPoint2, Point2D outputPoint3) 
            throws CoincidentPointsException {
        
        //normalize points to increase accuracy
        inputPoint1.normalize();
        inputPoint2.normalize();
        inputPoint3.normalize();
        
        outputPoint1.normalize();
        outputPoint2.normalize();
        outputPoint3.normalize();
        
        //matrix of homogeneous linear system of equations.
        //There are 7 unknowns and 6 equations (2 for each pair of corresponding 
        //points)
        Matrix m = null;
        try {
            m = new Matrix(6, 7); //build matrix initialized to zero
        } catch (WrongSizeException ignore) { }
        
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
        
        double oXiW = oX * iW;
        double oYiW = oY * iW;
                
        double norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                oXiW * oXiW);
        
        m.setElementAt(0, 0, oWiX / norm);
        m.setElementAt(0, 1, oWiY / norm);        
        m.setElementAt(0, 4, oWiW / norm);        
        m.setElementAt(0, 6, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                oYiW * oYiW);
        
        m.setElementAt(1, 2, oWiX / norm);
        m.setElementAt(1, 3, oWiY / norm);        
        m.setElementAt(1, 5, oWiW / norm);        
        m.setElementAt(1, 6, -oYiW / norm);
        
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
        
        oXiW = oX * iW;
        oYiW = oY * iW;
                
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                oXiW * oXiW);
        
        m.setElementAt(2, 0, oWiX / norm);
        m.setElementAt(2, 1, oWiY / norm);        
        m.setElementAt(2, 4, oWiW / norm);        
        m.setElementAt(2, 6, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                oYiW * oYiW);
        
        m.setElementAt(3, 2, oWiX / norm);
        m.setElementAt(3, 3, oWiY / norm);        
        m.setElementAt(3, 5, oWiW / norm);        
        m.setElementAt(3, 6, -oYiW / norm);
               
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
        
        oXiW = oX * iW;
        oYiW = oY * iW;
                
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                oXiW * oXiW);
        
        m.setElementAt(4, 0, oWiX / norm);
        m.setElementAt(4, 1, oWiY / norm);        
        m.setElementAt(4, 4, oWiW / norm);        
        m.setElementAt(4, 6, -oXiW / norm);
        
        norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW +
                oYiW * oYiW);
        
        m.setElementAt(5, 2, oWiX / norm);
        m.setElementAt(5, 3, oWiY / norm);        
        m.setElementAt(5, 5, oWiW / norm);        
        m.setElementAt(5, 6, -oYiW / norm);
                        
        //use SVD to decompose matrix m
        Matrix V;
        try{
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            //ensure that matrix m has enough rank and there is a unique 
            //solution (up to scale)
            if(decomposer.getRank() < 6) throw new CoincidentPointsException();
            V = decomposer.getV(); //V is 7x7
            
            //last column of V will contain parameters of transformation
            double value = V.getElementAt(6, 6);
            A.setElementAt(0, 0, V.getElementAt(0, 6) / value);
            A.setElementAt(0, 1, V.getElementAt(1, 6) / value);
            A.setElementAt(1, 0, V.getElementAt(2, 6) / value);
            A.setElementAt(1, 1, V.getElementAt(3, 6) / value);
            
            translation[0] = V.getElementAt(4, 6) / value;
            translation[1] = V.getElementAt(5, 6) / value;
                        
        } catch(AlgebraException e) {
            throw new CoincidentPointsException(e);
        }        
    }    
    
   /**
     * Estimates this transformation internal parameters by using 3 
     * corresponding original and transformed lines.
     * @param inputLine1 1st input line.
     * @param inputLine2 2nd input line.
     * @param inputLine3 3rd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input 
     * line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input
     * line.
     * @param outputLine3 3rd transformed line corresponding to 3rd input
     * line.
     * @throws CoincidentLinesException raised if transformation cannot be
     * estimated for some reason (line configuration degeneracy, duplicate 
     * lines or numerical instabilities).
     */    
    public final void setTransformationFromLines(Line2D inputLine1, 
            Line2D inputLine2, Line2D inputLine3, Line2D outputLine1, 
            Line2D outputLine2, Line2D outputLine3) 
            throws CoincidentLinesException {
        
        //normalize points to increase accuracy
        inputLine1.normalize();
        inputLine2.normalize();
        inputLine3.normalize();
        
        outputLine1.normalize();
        outputLine2.normalize();
        outputLine3.normalize();
        
        //matrix of homogeneous linear system of equations.
        //There are 7 unknowns and 6 equations (2 for each pair of corresponding 
        //points)
        Matrix m = null;
        try {
            m = new Matrix(6, 7); //build matrix initialized to zero
        } catch (WrongSizeException ignore) { }
        
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
        
        double oAiC = oA * iC;
        double oBiC = oB * iC;
                
        double norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                oAiC * oAiC);
        
        m.setElementAt(0, 0, oCiA / norm);
        m.setElementAt(0, 1, oCiB / norm);        
        m.setElementAt(0, 4, oCiC / norm);        
        m.setElementAt(0, 6, -oAiC / norm);
        
        norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                oBiC * oBiC);
        
        m.setElementAt(1, 2, oCiA / norm);
        m.setElementAt(1, 3, oCiB / norm);        
        m.setElementAt(1, 5, oCiC / norm);        
        m.setElementAt(1, 6, -oBiC / norm);
        
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
        
        oAiC = oA * iC;
        oBiC = oB * iC;
                
        norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                oAiC * oAiC);
        
        m.setElementAt(2, 0, oCiA / norm);
        m.setElementAt(2, 1, oCiB / norm);        
        m.setElementAt(2, 4, oCiC / norm);        
        m.setElementAt(2, 6, -oAiC / norm);
        
        norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                oBiC * oBiC);
        
        m.setElementAt(3, 2, oCiA / norm);
        m.setElementAt(3, 3, oCiB / norm);        
        m.setElementAt(3, 5, oCiC / norm);        
        m.setElementAt(3, 6, -oBiC / norm);
               
        //3rd pair of lines
        iA = inputLine3.getA();
        iB = inputLine3.getB();
        iC = inputLine3.getC();
        
        oA = outputLine3.getA();
        oB = outputLine3.getB();
        oC = outputLine3.getC();
        
        oCiA = oC * iA;
        oCiB = oC * iB;
        oCiC = oC * iC;
        
        oAiC = oA * iC;
        oBiC = oB * iC;
                
        norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                oAiC * oAiC);
        
        m.setElementAt(4, 0, oCiA / norm);
        m.setElementAt(4, 1, oCiB / norm);        
        m.setElementAt(4, 4, oCiC / norm);        
        m.setElementAt(4, 6, -oAiC / norm);
        
        norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                oBiC * oBiC);
        
        m.setElementAt(5, 2, oCiA / norm);
        m.setElementAt(5, 3, oCiB / norm);        
        m.setElementAt(5, 5, oCiC / norm);        
        m.setElementAt(5, 6, -oBiC / norm);
                        
        //use SVD to decompose matrix m
        Matrix V;
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            //ensure that matrix m has enough rank and there is a unique 
            //solution (up to scale)
            if(decomposer.getRank() < 6) throw new CoincidentLinesException();
            V = decomposer.getV(); //V is 7x7
            
            //last column of V will contain parameters of transformation
            double value = V.getElementAt(6, 6);                       
            AffineTransformation2D transformation = new AffineTransformation2D();
            transformation.A.setElementAt(0, 0, V.getElementAt(0, 6) / value);
            transformation.A.setElementAt(0, 1, V.getElementAt(1, 6) / value);
            transformation.A.setElementAt(1, 0, V.getElementAt(2, 6) / value);
            transformation.A.setElementAt(1, 1, V.getElementAt(3, 6) / value);            
            
            transformation.translation[0] = V.getElementAt(4, 6) / value;
            transformation.translation[1] = V.getElementAt(5, 6) / value;
                        
            //lines compute inverse transformation, so we need to inverse it
            //to get the right solution
            transformation.inverse(); //(we do it on another instance in case 
            //that transformation inversion fails, so that original A and
            //translation get preserved
            
            this.A = transformation.A;
            this.translation = transformation.translation;            
        } catch (AlgebraException e) {
            throw new CoincidentLinesException(e);
        }        
    }        
}