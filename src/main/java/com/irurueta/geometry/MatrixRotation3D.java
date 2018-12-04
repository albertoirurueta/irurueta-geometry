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

/**
 * This class defines the amount of rotation for 3D points or planes.
 * Rotation is defined internally as a matrix.
 */
@SuppressWarnings("WeakerAccess")
public class MatrixRotation3D extends Rotation3D implements Serializable {
    
    /**
     * Constant defining the number of rows on a 3D rotation matrix expressed
     * in inhomogeneous coordinates.
     */
    public static final int ROTATION3D_INHOM_MATRIX_ROWS = 3;
    
    /**
     * Constant defining the number of columns on a 3D rotation matrix expressed
     * in inhomogeneous coordinates.
     */
    public static final int ROTATION3D_INHOM_MATRIX_COLS = 3;
    
    /**
     * Constant defining the number of rows on a 3D rotation matrix expressed
     * in homogeneous coordinates.
     */
    public static final int ROTATION3D_HOM_MATRIX_ROWS = 4;
    
    /**
     * Constant defining the number of columns on a 3D rotation matrix expressed
     * in homogeneous coordinates.
     */
    public static final int ROTATION3D_HOM_MATRIX_COLS = 4;
    
    /**
     * Threshold to determine that a gimbal locked might have been achieved 
     * when trying to find roll, pitch and yaw angles.
     */
    public static final double GIMBAL_THRESHOLD = 1e-6;
        
    /**
     * Internal matrix containing rotation using inhomogeneous coordinates.
     * This matrix will be square, 3x3, orthogonal and will have determinant
     * equal to one.
     */
    protected Matrix internalMatrix;
    
    
    /**
     * Empty Constructor.
     * Initializes rotation so that no rotation exists (i.e. internal matrix is
     * the identity.
     */
    public MatrixRotation3D() {
        try {
            internalMatrix = Matrix.identity(ROTATION3D_INHOM_MATRIX_ROWS, 
                    ROTATION3D_INHOM_MATRIX_ROWS);
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
    
    /**
     * Copy constructor.
     * Copies provided rotation into this instance.
     * @param rotation Instance to be copied.
     */
    public MatrixRotation3D(MatrixRotation3D rotation) {
        internalMatrix = rotation.internalMatrix.clone();
    }
    
    /**
     * Copy constructor.
     * Copies and converts provided rotation into this instance.
     * @param rotation Instance to be copied.
     */
    public MatrixRotation3D(Rotation3D rotation) {
        internalMatrix = rotation.asInhomogeneousMatrix();
    }
    
    /**
     * Constructor.
     * Creates a 3D rotation using provided matrix.
     * Provided matrix can be expressed in either homogeneous or inhomogeneous
     * coordinates, and it must also be orthogonal and having determinant equal
     * to 1.
     * The threshold to determine whether provided matrix is orthonormal will
     * be DEFAULT_VALID_THRESHOLD.
     * @param m Matrix to create rotation from.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     * valid (its size is wrong or it is not orthonormal).
     * {@link #isValidRotationMatrix(Matrix)}.
     */
    public MatrixRotation3D(Matrix m) throws InvalidRotationMatrixException {
        fromMatrix(m);
    }
    
    /**
     * Constructor.
     * Creates a 3D rotation using provided matrix.
     * Provided matrix can be expressed in either homogeneous or inhomogeneous
     * coordinates, and it must also be orthogonal up to provided threshold, and
     * must have determinant equal to 1.
     * @param m Matrix to create rotation from.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     * valid (its size is wrong or it is not orthonormal).
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     * {@link #isValidRotationMatrix(Matrix)}.
     */
    public MatrixRotation3D(Matrix m, double threshold) 
            throws InvalidRotationMatrixException {
        fromMatrix(m, threshold);
    }
    
    /**
     * Constructor.
     * Creates a 3D rotation using provided Euler angles expressed in radians.
     * @param alphaEuler Alpha Euler angle expressed in radians.
     * @param betaEuler Beta Euler angle expressed in radians.
     * @param gammaEuler Gamma Euler angle expressed in radians.
     */
    public MatrixRotation3D(double alphaEuler, double betaEuler, 
            double gammaEuler) {
        setEulerAngles(alphaEuler, betaEuler, gammaEuler);
    }
    
    /**
     * Constructor.
     * Creates a 3D reconstruction using provided rotation axis and rotation
     * angle expressed in radians.
     * @param axis Axis of rotation. Axis must be a length-3 array containing
     * the axis vector. For better accuracy axis coordinates should be 
     * normalized (norm equal to 1).
     * @param theta Angle of rotation respect the axis expressed in radians.
     * @throws IllegalArgumentException Raised if provided axis does not have
     * length 3.
     */
    public MatrixRotation3D(double[] axis, double theta) {
        setAxisAndRotation(axis, theta);
    }
    
    /**
     * Constructor.
     * Creates a 3D reconstruction using provided rotation axis coordinates and 
     * rotation angle expressed in radians.
     * Note: for better accuracy axis coordinates should be normalized (norm 
     * equal to 1).
     * @param axisX X coordinate of axis.
     * @param axisY Y coordinate of axis.
     * @param axisZ Z coordinate of axis.
     * @param theta Angle of rotation respect the axis expressed in radians.
     */
    public MatrixRotation3D(double axisX, double axisY, double axisZ, 
            double theta) {
        setAxisAndRotation(axisX, axisY, axisZ, theta);
    }
    
    /**
     * Returns type of this rotation.
     * @return Type of this rotation.
     */
    @Override
    public Rotation3DType getType() {
        return Rotation3DType.MATRIX_ROTATION3D;
    }    
    
    /**
     * Returns a copy of the internal matrix so that the internal matrix cannot
     * be modified accidentally.
     * Returned matrix will be 3x3, orthogonal and will have determinant equal
     * to one.
     * @return Internal matrix containing rotation of this instance.
     */
    public Matrix getInternalMatrix() {
        return internalMatrix.clone();
    }
    
    /**
     * Sets the internal matrix of this rotation. Provided matrix must be 3x3
     * and orthonormal (orthogonal with determinant equal to 1).
     * @param internalMatrix Internal matrix to be set.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     * 3x3 or it is not orthonormal.
     */
    public final void setInternalMatrix(Matrix internalMatrix) 
            throws InvalidRotationMatrixException {
        setInternalMatrix(internalMatrix, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Sets the internal matrix of this rotation. Provided matrix must be 3x3
     * and orthonormal (orthogonal with determinant equal to 1) up to an error
     * equal to provided threshold.
     * @param m Internal matrix to be set.
     * @param threshold Threshold to determine whether matrix is orthonormal or
     * not.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     * 3x3 or it is not orthonormal.
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     */
    public final void setInternalMatrix(Matrix m, double threshold)
            throws InvalidRotationMatrixException {
        if (m.getRows() != ROTATION3D_INHOM_MATRIX_ROWS ||
                m.getColumns() != ROTATION3D_INHOM_MATRIX_COLS) {
            throw new InvalidRotationMatrixException();
        }
        if (!isValidRotationMatrix(m, threshold)) {
            throw new InvalidRotationMatrixException();
        }
        
        internalMatrix = m;
    }
    
    /**
     * Returns alpha euler angle within the range -pi and pi.
     * @return Alpha euler angle expressed in radians.
     */
    public double getAlphaEulerAngle() {
        return Math.atan2(-internalMatrix.getElementAt(2, 0), 
                internalMatrix.getElementAt(2, 2));
    }
    
    /**
     * Returns beta euler angle within the range -pi/2 and pi/2.
     * @return Beta euler angle expressed in radians.
     */
    public double getBetaEulerAngle() {
        return Math.asin(internalMatrix.getElementAt(2, 1));
    }
    
    /**
     * Returns gamma euler angle within the range -pi and pi.
     * @return Gamma euler angle expressed in radians.
     */
    public double getGammaEulerAngle() {
        return Math.atan2(-internalMatrix.getElementAt(0, 1), 
                internalMatrix.getElementAt(1, 1));
    }
    
    /**
     * Sets euler angles of this rotation, expressed in radians.
     * @param alphaEuler Alpha euler angle in radians.
     * @param betaEuler Beta euler angle in radians.
     * @param gammaEuler Gamma euler angle in radians.
     */
    public final void setEulerAngles(double alphaEuler, double betaEuler, 
            double gammaEuler) {
        
        double sinAlpha = Math.sin(alphaEuler);
        double cosAlpha = Math.cos(alphaEuler);
        
        double sinBeta = Math.sin(betaEuler);
        double cosBeta = Math.cos(betaEuler);
        
        double sinGamma = Math.sin(gammaEuler);
        double cosGamma = Math.cos(gammaEuler);
        
        //reuse internal matrix if possible
        try {
            if (internalMatrix == null) {
                internalMatrix = new Matrix(ROTATION3D_INHOM_MATRIX_ROWS,
                        ROTATION3D_INHOM_MATRIX_COLS);
            }

            internalMatrix.setElementAt(0, 0, cosAlpha * cosGamma -
                    sinAlpha * sinBeta * sinGamma);
            internalMatrix.setElementAt(1, 0, cosAlpha * sinGamma +
                    sinAlpha * sinBeta * cosGamma);
            internalMatrix.setElementAt(2, 0, -sinAlpha * cosBeta);

            internalMatrix.setElementAt(0, 1, -cosBeta * sinGamma);
            internalMatrix.setElementAt(1, 1, cosBeta * cosGamma);
            internalMatrix.setElementAt(2, 1, sinBeta);

            internalMatrix.setElementAt(0, 2, sinAlpha * cosGamma +
                    cosAlpha * sinBeta * sinGamma);
            internalMatrix.setElementAt(1, 2, sinAlpha * sinGamma -
                    cosAlpha * sinBeta * cosGamma);
            internalMatrix.setElementAt(2, 2, cosAlpha * cosBeta);
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
    
    /**
     * Returns roll angle around x axis expressed in radians for the 1st 
     * possible set of solutions.
     * When obtaining roll, pitch and yaw angles from a rotation matrix, there 
     * might be two possible sets of solutions (#getRollAngle(), 
     * #getPitchAngle(), #getYawAngle()) or (#getRollAngle2(), 
     * #getPitchAngle2(), #getYawAngle2()).
     * @return roll angle around x axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public double getRollAngle() {
        return getRollAngle(getPitchAngle());
    }
    
    /**
     * Returns roll angle around x axis expressed in radians for the 2nd 
     * possible set of solutions.
     * When obtaining roll, pitch and yaw angles from a rotation matrix, there 
     * might be two possible sets of solutions (#getRollAngle(), 
     * #getPitchAngle(), #getYawAngle()) or (#getRollAngle2(), 
     * #getPitchAngle2(), #getYawAngle2()).
     * @return roll angle around x axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public double getRollAngle2() {
        return getRollAngle(getPitchAngle2());
    }
    
    /**
     * Returns roll angle around x axis expressed in radians corresponding to
     * provided pitch value. 
     * This method is used internally.
     * @param pitch pitch angle expressed in radians.
     * @return roll angle around x axis.
     */
    private double getRollAngle(double pitch) {
        if (!hasGimbalLock()) {
            double cosPitch = Math.cos(pitch);
            return Math.atan2(internalMatrix.getElementAt(2, 1) / cosPitch, 
                internalMatrix.getElementAt(2, 2) / cosPitch);        
        } else {
            //gimbal lock (pitch is close to +-90 degrees)
            if (internalMatrix.getElementAt(2, 0) < 0.0) {
                //pitch is +90 degrees
                return Math.atan2(internalMatrix.getElementAt(0, 1), 
                        internalMatrix.getElementAt(0, 2));
            } else {
                //pitch is -90 degrees
                return Math.atan2(-internalMatrix.getElementAt(0, 1),
                        -internalMatrix.getElementAt(0, 2));
            }
        }                
    }
        
    /**
     * Returns pitch angle around y axis expressed in radians for the 1st 
     * possible set of solutions.
     * When obtaining roll, pitch and yaw angles from a rotation matrix, there 
     * might be two possible sets of solutions (#getRollAngle(), 
     * #getPitchAngle(), #getYawAngle()) or (#getRollAngle2(), 
     * #getPitchAngle2(), #getYawAngle2()).
     * @return pitch angle around y axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public double getPitchAngle() {
        return -Math.asin(internalMatrix.getElementAt(2, 0));
    }
    
    /**
     * Returns pitch angle around y axis expressed in radians for the 2nd
     * possible set of solutions.
     * When obtaining roll, pitch and yaw angles from a rotation matrix, there 
     * might be two possible sets of solutions (#getRollAngle(), 
     * #getPitchAngle(), #getYawAngle()) or (#getRollAngle2(), 
     * #getPitchAngle2(), #getYawAngle2()).
     * When a gimbal lock occurs this both pitch angles are equal because only
     * yaw is undefined, buth pitch and roll are unique.
     * @return pitch angle around y axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public double getPitchAngle2() {
        if (!hasGimbalLock()) {
            return Math.PI - getPitchAngle();
        } else {
            return getPitchAngle();
        }
    }
    
    /**
     * Returns yaw angle around z axis expressed in radians for the 1st possible
     * set of solutions.
     * When a gimbal lock occurs (pitch angle is close to +- 90 degrees), then
     * yaw angle is undefined, and can be any value, although this method will
     * return 0.0.
     * When obtaining roll, pitch and yaw angles from a rotation matrix, there 
     * might be two possible sets of solutions (#getRollAngle(), 
     * #getPitchAngle(), #getYawAngle()) or (#getRollAngle2(), 
     * #getPitchAngle2(), #getYawAngle2()).
     * @return yaw angle around z axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public double getYawAngle() {
        return getYawAngle(getPitchAngle());
    }
    
    /**
     * Returns yaw angle arounx z axis expressed in radians for the 2nd possible
     * set of solutions.
     * When a gimbal lock occurs (pitch angle is close to +- 90 degrees), then
     * yaw angle is undefined, and can be any value, although this method will
     * return 0.0.
     * When obtaining roll, pitch and yaw angles from a rotation matrix, there 
     * might be two possible sets of solutions (#getRollAngle(), 
     * #getPitchAngle(), #getYawAngle()) or (#getRollAngle2(), 
     * #getPitchAngle2(), #getYawAngle2()).
     * @return yaw angle around z axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public double getYawAngle2() {
        return getYawAngle(getPitchAngle2());
    }
    
    /**
     * Returns yaw angle around x axis expressed in radians corresponding to
     * provided pitch value. 
     * This method is used internally.
     * @param pitch pitch angle expressed in radians.
     * @return yaw angle around x axis.
     */
    private double getYawAngle(double pitch) {
        if (!hasGimbalLock()) {
            double cosPitch = Math.cos(pitch);
            return Math.atan2(internalMatrix.getElementAt(1, 0) / cosPitch, 
                    internalMatrix.getElementAt(0, 0) / cosPitch);
        } else {
            //gimbal lock (pitch is close to +-90 degrees)
            return 0.0; //can be anything.
        }        
    }
    
    /**
     * Indicates whether current rotation contains ambiguities (a.k.a gimbal
     * lock). This situation happens when pitch angle is close to +-90 degrees).
     * @return true if current rotation contains a gimbal lock, false otherwise.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">R2e.m at https://github.com/joansola/slamtb</a>
     */
    public boolean hasGimbalLock() {
        return Math.abs(Math.abs(internalMatrix.getElementAt(2, 0)) - 1.0) < 
                GIMBAL_THRESHOLD;
    }        
    
    /**
     * Sets rotation angles, expressed in radians.
     * @param roll roll angle in radians around x axis.
     * @param pitch pitch angle in radians around y axis.
     * @param yaw yaw angle in radians around z axis.
     * @see <a href="http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf">http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf</a>
     * @see <a href="https://github.com/joansola/slamtb">e2R.m at https://github.com/joansola/slamtb</a>
     */
    public void setRollPitchYaw(double roll, double pitch, double yaw) {
        double sr = Math.sin(roll);
        double cr = Math.cos(roll);
        
        double sp = Math.sin(pitch);
        double cp = Math.cos(pitch);
        
        double sy = Math.sin(yaw);
        double cy = Math.cos(yaw);

        try {
            //reuse internal matrix if possible
            if (internalMatrix == null) {
                internalMatrix = new Matrix(ROTATION3D_INHOM_MATRIX_ROWS,
                        ROTATION3D_INHOM_MATRIX_COLS);
            }

            internalMatrix.setElementAt(0, 0, cp * cy);
            internalMatrix.setElementAt(1, 0, cp * sy);
            internalMatrix.setElementAt(2, 0, -sp);

            internalMatrix.setElementAt(0, 1, -cr * sy + sr * sp * cy);
            internalMatrix.setElementAt(1, 1, cr * cy + sr * sp * sy);
            internalMatrix.setElementAt(2, 1, sr * cp);

            internalMatrix.setElementAt(0, 2, sr * sy + cr * sp * cy);
            internalMatrix.setElementAt(1, 2, -sr * cy + cr * sp * sy);
            internalMatrix.setElementAt(2, 2, cr * cp);
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
        
    /**
     * Sets the axis and rotation of this instance.
     * Once set, points will rotate around provided axis an amount equal to
     * provided rotation angle in radians.
     * Note: to avoid numerical instabilities and improve accuracy, axis
     * coordinates should be normalized (e.g. norm equal to 1).
     * @param axisX X coordinate of rotation axis.
     * @param axisY Y coordinate of rotation axis.
     * @param axisZ Z coordinate of rotation axis.
     * @param theta Amount of rotation in radians.
     */
    @Override
    public final void setAxisAndRotation(double axisX, double axisY, 
            double axisZ, double theta) {
        double axisX2 = axisX * axisX;
        double axisY2 = axisY * axisY;
        double axisZ2 = axisZ * axisZ;
        
        double axisXY = axisX * axisY;
        double axisXZ = axisX * axisZ;
        double axisYZ = axisY * axisZ;
        
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);

        try {
            if (internalMatrix == null) {
                internalMatrix = new Matrix(ROTATION3D_INHOM_MATRIX_ROWS,
                        ROTATION3D_INHOM_MATRIX_COLS);
            }

            internalMatrix.setElementAt(0, 0, axisX2 + (1.0 - axisX2) * cosTheta);
            internalMatrix.setElementAt(1, 0, axisXY * (1.0 - cosTheta) +
                    axisZ * sinTheta);
            internalMatrix.setElementAt(2, 0, axisXZ * (1.0 - cosTheta) -
                    axisY * sinTheta);

            internalMatrix.setElementAt(0, 1, axisXY * (1.0 - cosTheta) -
                    axisZ * sinTheta);
            internalMatrix.setElementAt(1, 1, axisY2 + (1.0 - axisY2) * cosTheta);
            internalMatrix.setElementAt(2, 1, axisYZ * (1.0 - cosTheta) +
                    axisX * sinTheta);

            internalMatrix.setElementAt(0, 2, axisXZ * (1.0 - cosTheta) +
                    axisY * sinTheta);
            internalMatrix.setElementAt(1, 2, axisYZ * (1.0 - cosTheta) -
                    axisX * sinTheta);
            internalMatrix.setElementAt(2, 2, axisZ2 + (1.0 - axisZ2) * cosTheta);
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
        
    /**
     * Returns rotation axis corresponding to this instance.
     * Result is stored in provided axis array, which must have length 3.
     * @param axis Array where axis coordinates will be stored.
     * @throws IllegalArgumentException Raised if provided array does not have
     * length 3.
     * @throws RotationException Raised if numerical instabilities happen.
     * Because internal matrix will always be well defined (orthogonal and
     * determinant equal to 1), this exception will rarely happen.
     */
    @Override
    public void rotationAxis(double[] axis) throws RotationException {
        if (axis.length != ROTATION3D_INHOM_MATRIX_ROWS) {
            throw new IllegalArgumentException();
        }

        try {
            //Rotation axis follows: R*v = v, which means that rotation axis is
            //left unchanged after rotation. Hence:
            //R*v = I*v --> (R - I)*v = 0, consequently rotation axis is the
            //null-space of R-I
            Matrix identity = Matrix.identity(ROTATION3D_INHOM_MATRIX_ROWS, 
                    ROTATION3D_INHOM_MATRIX_COLS);
            //line below: identity = internalMatrix - identity
            internalMatrix.subtract(identity, identity);
            //internalMatrix - identity
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    identity);
        
            decomposer.decompose();
        
            Matrix v = decomposer.getV();
        
            //last column of V contains axis values
            axis[0] = v.getElementAt(0, 2);
            axis[1] = v.getElementAt(1, 2);
            axis[2] = v.getElementAt(2, 2);
        } catch (AlgebraException e) {
            throw new RotationException(e);
        }        
    }
    
    /**
     * Returns rotation amount or angle in radians around the rotation axis
     * associated to this instance.
     * @return Rotation angle in radians.
     * @throws RotationException Raised if numerical instabilities happen.
     * Because internal matrix will always be well defined (orthogonal and
     * determinant equal to 1), this exception will rarely happen.
     */
    @Override
    public double getRotationAngle() throws RotationException {
        
        //obtain rotation axis
        double[] axis = getRotationAxis();
        
        try {
            Matrix axisMatrix = new Matrix(1, ROTATION3D_INHOM_MATRIX_COLS);
            axisMatrix.setSubmatrix(0, 0, 0,
                    ROTATION3D_INHOM_MATRIX_COLS - 1, axis);
        
            SingularValueDecomposer decomposer = 
                    new SingularValueDecomposer(axisMatrix);
            decomposer.decompose();
        
            Matrix v = decomposer.getV();
        
            //because axisMatrix has rank 1, its null-space will contain a
            //two-dimensional space (two vectors) perpendicular to axisMatrix
            Matrix perpendicular = v.getSubmatrix(0, 2, 2, 2);
            double normPerpendicular = Utils.normF(perpendicular);
        
            //use internal matrix to rotate perpendicular vector
            Matrix rotated = internalMatrix.multiplyAndReturnNew(perpendicular);
            double normRotated = Utils.normF(rotated);
        
            //normalize vectors
            perpendicular.multiplyByScalar(1.0 / normPerpendicular);
            rotated.multiplyByScalar(1.0 / normRotated);
        
            //their dot product is the cosine of their angle
            //(we transpose rotated matrix to compute dot product)
            rotated.transpose();
            rotated.multiply(perpendicular);
            double dotProduct = rotated.getElementAtIndex(0);
        
            double theta = Math.acos(dotProduct);
        
            //we need to determine sign of theta, for that reason we instantiate 
            //two camera rotations with theta and -theta using the same rotation 
            //axis and check for the rotation matrix that produces less error
            MatrixRotation3D rotation1 = new MatrixRotation3D(axis, theta);
            MatrixRotation3D rotation2 = new MatrixRotation3D(axis, -theta);
        
            Matrix rotationMatrix1 = rotation1.internalMatrix;
            Matrix rotationMatrix2 = rotation2.internalMatrix;
        
            //normalize rotation matrices for their comparison
            double norm1 = Utils.normF(rotationMatrix1);
            double norm2 = Utils.normF(rotationMatrix2);
            rotationMatrix1.multiplyByScalar(1.0 / norm1);
            rotationMatrix2.multiplyByScalar(1.0 / norm2);
        
            //normalize current internal matrix for its comparison
            double internalNorm = Utils.normF(internalMatrix);
            Matrix internal = internalMatrix.multiplyByScalarAndReturnNew(
                    1.0 / internalNorm);
        
            //compare matrices
            rotationMatrix1.subtract(internal);
            rotationMatrix2.subtract(internal);
        
            //now rotation matrices 1 and 2 contain the difference

            double normDiff1 = Utils.normF(rotationMatrix1);
            double normDiff2 = Utils.normF(rotationMatrix2);
        
            //pich the rotation matrix that produces less error
            if (normDiff1 < normDiff2) {
                return theta; //positive theta
            } else {
                return -theta; //negative theta
            }
        } catch (AlgebraException e) {
            throw new RotationException(e);
        }
    }        
        
    /**
     * Returns a 3D rotation which is inverse to this instance.
     * In other words, the combination of this rotation with its inverse 
     * produces no change.
     * @return Inverse 3D rotation.
     */
    @Override
    public MatrixRotation3D inverseRotationAndReturnNew() {
        MatrixRotation3D result = new MatrixRotation3D();
        inverseRotation(result);
        return result;
    }
    
    /**
     * Sets into provided MatrixRotation3D instance a rotation inverse to this 
     * instance.
     * The combination of this rotation with its inverse produces no change.
     * @param result Instance where inverse rotation will be set.
     */
    public void inverseRotation(MatrixRotation3D result) {
        try {
            result.internalMatrix = Utils.inverse(internalMatrix);
        } catch (AlgebraException ignore) {
            //matrix should always be invertible
        }
    }
    
    /**
     * Sets into provided MatrixRotation3D instance a rotation inverse to this 
     * instance.
     * The combination of this rotation with its inverse produces no change.
     * @param result Instance where inverse rotation will be set.
     */    
    @Override
    public void inverseRotation(Rotation3D result) {
        if (result instanceof MatrixRotation3D) {
            inverseRotation((MatrixRotation3D)result);
        } else if (result instanceof AxisRotation3D) {
            MatrixRotation3D rot = new MatrixRotation3D();
            inverseRotation(rot);
            try {
                result.fromMatrix(rot.asInhomogeneousMatrix());
            } catch (InvalidRotationMatrixException ignore) {
                //never happens
            }
        }
    }    
    
    /**
     * Reverses the rotation of this instance.
     */
    @Override
    public void inverseRotation() {
        inverseRotation(this);
    }
    
    
    /**
     * Returns this 3D rotation instance expressed as a 3x3 inhomogeneous 
     * matrix.
     * This is equivalent to call getInternalMatrix().
     * @return Rotation matrix expressed in inhomogeneous coordinates.
     */
    @Override
    public Matrix asInhomogeneousMatrix() {
        return getInternalMatrix();
    }       
    
    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 3x3 inhomogeneous matrix.
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     * have size 3x3.
     */
    @Override
    public void asInhomogeneousMatrix(Matrix result) {
        if (result.getRows() != ROTATION3D_INHOM_MATRIX_ROWS ||
                result.getColumns() != ROTATION3D_INHOM_MATRIX_COLS) {
            throw new IllegalArgumentException();
        }
        
        result.copyFrom(internalMatrix);
    }
    
    /**
     * Returns this 3D rotation instance expressed as a 4x4 homogeneous matrix.
     * @return Rotation matrix expressed in homogeneous coordinates.
     */    
    @Override
    public Matrix asHomogeneousMatrix() {
        Matrix result = null;
        try {
            result = Matrix.identity(ROTATION3D_HOM_MATRIX_ROWS, 
                    ROTATION3D_HOM_MATRIX_COLS);
            //sets into 3x3 top-left submatrix the internal matrix of this
            //instance, the remaining part will continue to be the identity
            result.setSubmatrix(0, 0, ROTATION3D_INHOM_MATRIX_ROWS - 1, 
                    ROTATION3D_INHOM_MATRIX_COLS - 1, internalMatrix);
        } catch (WrongSizeException ignore) {
            //never happens
        }
        return result;
    }

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 4x4 homogeneous matrix.
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     * have size 4x4.
     */
    @Override
    public void asHomogeneousMatrix(Matrix result) {
        if (result.getRows() != ROTATION3D_HOM_MATRIX_ROWS ||
                result.getColumns() != ROTATION3D_HOM_MATRIX_COLS) {
            throw new IllegalArgumentException();
        }
        
        result.initialize(0.0);
        //sets into 3x3 top-left submatrix the internal matrix of this instance
        result.setSubmatrix(0, 0, ROTATION3D_INHOM_MATRIX_ROWS - 1, 
                ROTATION3D_INHOM_MATRIX_COLS - 1, internalMatrix);
        //set las element to 1.0 (to be like the identity
        result.setElementAt(ROTATION3D_HOM_MATRIX_ROWS - 1, 
                ROTATION3D_HOM_MATRIX_COLS - 1, 1.0);
    }
        
    /**
     * Sets amount of rotation from provided inhomogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be it's inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 3x3.
     * @param m Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     * valid (has wrong size or it is not orthonormal).
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     * {@link #isValidRotationMatrix(Matrix)}.
     */    
    @Override
    public void fromInhomogeneousMatrix(Matrix m, double threshold)
            throws InvalidRotationMatrixException {
        setInternalMatrix(m, threshold);
    }
        
    /**
     * Sets amount of rotation from provided homogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be it's inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 4x4, and its last row and column must
     * be zero, except for element in last row and column which must be 1.
     * @param m Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws InvalidRotationMatrixException Raised if provided matrix is not
     * valid (has wrong size or it is not orthonormal).
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     * {@link #isValidRotationMatrix(Matrix)}.
     */      
    @Override
    public void fromHomogeneousMatrix(Matrix m, double threshold)
            throws InvalidRotationMatrixException {
        if (m.getRows() != ROTATION3D_HOM_MATRIX_ROWS ||
                m.getColumns() != ROTATION3D_HOM_MATRIX_COLS) {
            throw new InvalidRotationMatrixException();
        }
        if (!isValidRotationMatrix(m, threshold)) {
            throw new InvalidRotationMatrixException();
        }
        if (Math.abs(m.getElementAt(3, 0)) > threshold ||
                Math.abs(m.getElementAt(3, 1)) > threshold ||
                Math.abs(m.getElementAt(3, 2)) > threshold ||
                Math.abs(m.getElementAt(0, 3)) > threshold ||
                Math.abs(m.getElementAt(1, 3)) > threshold ||
                Math.abs(m.getElementAt(2, 3)) > threshold ||
                Math.abs(m.getElementAt(3, 3) - 1.0) > threshold) {
            throw new InvalidRotationMatrixException();
        }
        
        internalMatrix = m.getSubmatrix(0, 0, ROTATION3D_INHOM_MATRIX_ROWS - 1, 
                ROTATION3D_INHOM_MATRIX_COLS - 1);
    }
        
    /**
     * Rotates a 3D point using the origin of coordinates as the axis of 
     * rotation.
     * Point will be rotated by the amount of rotation contained in this 
     * instance.
     * @param inputPoint Input point to be rotated.
     * @param resultPoint Rotated point.
     */
    @Override
    public void rotate(Point3D inputPoint, Point3D resultPoint) {
        try {
            Matrix r = asHomogeneousMatrix();
            Matrix p = new Matrix(
                Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            
            inputPoint.normalize(); //to increase accuracy
            p.setElementAt(0, 0, inputPoint.getHomX());
            p.setElementAt(1, 0, inputPoint.getHomY());
            p.setElementAt(2, 0, inputPoint.getHomZ());
            p.setElementAt(3, 0, inputPoint.getHomW());
            
            //Rotated point below is R * p
            r.multiply(p);
            
            resultPoint.setHomogeneousCoordinates(r.getElementAt(0, 0),
                    r.getElementAt(1, 0), r.getElementAt(2, 0),
                    r.getElementAt(3, 0));
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
                            
    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new MatrixRotation3D instance.
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the multiplication of the
     * internal matrix of provided rotation with the internal matrix of this 
     * instance.
     */    
    public MatrixRotation3D combineAndReturnNew(MatrixRotation3D rotation) {
        MatrixRotation3D result = new MatrixRotation3D();
        combine(this, rotation, result);
        return result;
    }
    
    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new MatrixRotation3D instance.
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the multiplication of the
     * internal matrix of provided rotation with the internal matrix of this 
     * instance.
     */        
    @Override
    public Rotation3D combineAndReturnNew(Rotation3D rotation) {
        if (rotation instanceof MatrixRotation3D) {
            return combineAndReturnNew((MatrixRotation3D)rotation);
        } else {
            return combineAndReturnNew(new MatrixRotation3D(rotation));
        }
    }    
    
    /**
     * Combines provided rotation into this rotation resulting in the 
     * multiplication of the internal matrices of both rotations.
     * @param rotation Input rotation to be combined.
     */    
    public void combine(MatrixRotation3D rotation) {
        combine(this, rotation, this);
    }
    
    /**
     * Combines provided rotation into this rotation resulting in the 
     * multiplication of the internal matrices of both rotations.
     * @param rotation Input rotation to be combined.
     */        
    @Override
    public void combine(Rotation3D rotation) {
        if (rotation instanceof MatrixRotation3D) {
            combine((MatrixRotation3D)rotation);
        } else {
            combine(new MatrixRotation3D(rotation));
        }
    }    
   
    /**
     * Combines the rotation of instances rot1 and rot1 into provided result
     * instance.
     * @param rot1 1st input rotation.
     * @param rot2 2nd input rotation.
     * @param result Combined rotation, which is equal to the multiplication of
     * the internal matrix of provided rotation with the internal matrix of this
     * instance.
     */    
    public static void combine(MatrixRotation3D rot1, MatrixRotation3D rot2,
            MatrixRotation3D result) {
        
        try {
            result.internalMatrix = rot1.internalMatrix.multiplyAndReturnNew(
                    rot2.internalMatrix);
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
    
    /**
     * Sets values of this rotation from a 3D matrix rotation.
     * @param rot 3D matrix rotation to set values from.
     */
    @Override
    public void fromRotation(MatrixRotation3D rot) {
        rot.internalMatrix.copyTo(internalMatrix);
    }
        
    /**
     * Sets values of this rotation from a quaternion.
     * @param q a quaternion to set values from.
     */
    @Override
    public void fromRotation(Quaternion q) {
        q.toMatrixRotation(internalMatrix);
    }

    /**
     * Converts this 3D rotation into a matrix rotation storing the result
     * into provided instance.
     * @param result instance where result wil be stored.
     */
    @Override
    public void toMatrixRotation(MatrixRotation3D result) {
        internalMatrix.copyTo(result.internalMatrix);
    }    
    
    /**
     * Converts this 3D rotation into an axis rotation storing the result into
     * provided instance.
     * @param result instance where result will be stored.
     */
    @Override
    public void toAxisRotation(AxisRotation3D result) {
        try {
            result.fromInhomogeneousMatrix(internalMatrix);
        } catch (InvalidRotationMatrixException ignore) { /* never thrown */ }
    }
    
    /**
     * Converts this 3D rotation into a quaterion storing the result into 
     * provided instance.
     * @param result instance where result will be stored.
     */
    @Override
    public void toQuaternion(Quaternion result) {
        Quaternion.matrixRotationToQuaternion(internalMatrix, result);
    }
}
