/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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

import java.io.Serializable;
import java.util.Arrays;

/**
 * Contains a representation of a 3D rotation in a more precise and compact way
 * than in matrix notation.
 * This implementation of a quaternion contains values in the basis 1, i, j, k
 * expressed as (a, b, c, d).
 * a value is related only to the rotation angle, while b, c, d values are related
 * both to the rotation axis and the rotation angle.
 */
@SuppressWarnings("WeakerAccess")
public class Quaternion extends Rotation3D implements Serializable {
    
    /**
     * Number of parameters contained in a quaternion.
     */
    public static final int N_PARAMS = 4;
    
    /**
     * Number of euler angles.
     */
    public static final int N_ANGLES = 3;
    
    /**
     * Threshold of axis norm to convert quaternions to axis and rotation angle.
     */
    public static final double AXIS_NORM_THRESHOLD = 1e-7;
    
    /**
     * Large threshold of axis norm to convert quaternions to axis and rotation
     * angle.
     */
    public static final double LARGE_AXIS_NORM_THRESHOLD = 1e-6;
    
    /**
     * Trace threshold to convert rotation matrices into quaternions.
     */
    public static final double TRACE_THRESHOLD = 1e-8;
        
    /**
     * Value corresponding to real numbers basis.
     */
    private double mA;
    
    /**
     * Value corresponding to basis i.
     */
    private double mB;
    
    /**
     * Value corresponding to basis j.
     */
    private double mC;
    
    /**
     * Value corresponding to basis k.
     */
    private double mD;
    
    /**
     * Indicates whether quaternion is normalized or not.
     */
    private boolean mNormalized;
    
    /**
     * Default constructor.
     * Creates a quaternion containing no rotation.
     */
    public Quaternion() {
        mA = 1.0;
    }
    
    /**
     * Constructor.
     *
     * @param a value corresponding to real numbers basis.
     * @param b value corresponding to basis i.
     * @param c value corresponding to basis j.
     * @param d value corresponding to basis k.
     */
    public Quaternion(double a, double b, double c, double d) {
        mA = a;
        mB = b;
        mC = c;
        mD = d;
    }
    
    /**
     * Constructor.
     *
     * @param quaternion quaternion to be copied from.
     */
    public Quaternion(Quaternion quaternion) {
        fromQuaternion(quaternion);
    }
    
    /**
     * Constructor.
     *
     * @param values    values to be stored in the quaternion expressed in the
     *                  basis (1, i, j, k)
     * @throws IllegalArgumentException if provided array does not have length 
     * 4.
     */
    public Quaternion(double[] values) {
        setValues(values);
    }
    
    /**
     * Constructor.
     *
     * @param axis  a rotation axis.
     * @param theta a rotation angle expressed in radians.
     * @throws IllegalArgumentException if provided axis array does not have 
     * length 3.
     */
    public Quaternion(double[] axis, double theta) {
        setFromAxisAndRotation(axis, theta);
    }
    
    /**
     * Constructor from and axis 3D rotation.
     *
     * @param axisRotation an axis 3D rotation.
     */
    public Quaternion(AxisRotation3D axisRotation) {
        setFromAxisAndRotation(axisRotation);
    }
    
    /**
     * Constructor from euler angles.
     *
     * @param roll  roll angle expressed in radians.
     * @param pitch pitch angle expressed in radians.
     * @param yaw   yaw angle expressed in radians.
     */
    public Quaternion(double roll, double pitch, double yaw) {
        setFromEulerAngles(roll, pitch, yaw);
    }
    
    /**
     * Constructor from matrix rotation.
     *
     * @param matrixRotation matrix rotation.
     */
    public Quaternion(MatrixRotation3D matrixRotation) {
        setFromMatrixRotation(matrixRotation);
    }
    
    /**
     * Gets value corresponding to real numbers basis.
     *
     * @return value corresponding to real numbers basis.
     */
    public double getA() {
        return mA;
    }
    
    /**
     * Sets value corresponding to real numbers basis.
     *
     * @param a value corresponding to real numbers basis.
     */
    public void setA(double a) {
        mA = a;
        mNormalized = false;
    }
    
    /**
     * Gets value corresponding to basis i.
     *
     * @return value corresponding to basis i.
     */
    public double getB() {
        return mB;
    }
    
    /**
     * Sets value corresponding to basis i.
     *
     * @param b value corresponding to basis i.
     */
    public void setB(double b) {
        mB = b;
        mNormalized = false;
    }
    
    /**
     * Gets value corresponding to basis j.
     *
     * @return value corresponding to basis j.
     */
    public double getC() {
        return mC;
    }
    
    /**
     * Sets value corresponding to basis j.
     *
     * @param c value corresponding to basis j.
     */
    public void setC(double c) {
        mC = c;
        mNormalized = false;
    }
    
    /**
     * Gets value corresponding to basis k.
     *
     * @return value corresponding to basis k.
     */
    public double getD() {
        return mD;
    }
    
    /**
     * Sets value corresponding to basis k.
     *
     * @param d value corresponding to basis k.
     */
    public void setD(double d) {
        mD = d;
        mNormalized = false;
    }
    
    /**
     * Gets values that parameterize this quaternion.
     *
     * @return values of this quaternion.
     */
    public double[] getValues() {
        double[] result = new double[N_PARAMS];
        values(result);
        return result;
    }
    
    /**
     * Stores values that parameterize this quaternion into provided array.
     *
     * @param result array where quaternion parameters will be stored.
     * @throws IllegalArgumentException if length of provided array is not 4.
     */
    public void values(double[] result) {
        if (result.length != N_PARAMS) {
            throw new IllegalArgumentException("result length must be 4");
        }
        
        result[0] = mA;
        result[1] = mB;
        result[2] = mC;
        result[3] = mD;
    }
    
    /**
     * Sets values that parameterize this quaternion in basis (1, i, j ,k).
     *
     * @param values    values that parameterize this quaternion in basis (1, i, j,
     *                  k).
     * @throws IllegalArgumentException if provided array length is not 4.
     */
    public final void setValues(double[] values) {
        if (values.length != N_PARAMS) {
            throw new IllegalArgumentException("values length must be 4");
        }
        
        mA = values[0];
        mB = values[1];
        mC = values[2];
        mD = values[3];  
        mNormalized = false;
    }
    
    /**
     * Copies values from provided quaternion into this instance.
     *
     * @param quaternion quaternion to copy from.
     */
    public final void fromQuaternion(Quaternion quaternion) {
        mA = quaternion.mA;
        mB = quaternion.mB;
        mC = quaternion.mC;
        mD = quaternion.mD;
        mNormalized = quaternion.mNormalized;
    }
    
    /**
     * Returns a new quaternion instance containing the same data as this 
     * instance.
     *
     * @return a copy of this quaternion instance.
     */
    @Override
    @SuppressWarnings("all")
    public Quaternion clone() {
        return new Quaternion(this);
    }
    
    /**
     * Copies this instance data into provided quaternion instance.
     *
     * @param output destination instance where data is copied to.
     */
    public void copyTo(Quaternion output) {
        output.mA = mA;
        output.mB = mB;
        output.mC = mC;
        output.mD = mD;
        output.mNormalized = mNormalized;
    }
    
    /**
     * Sets quaternion parameters from axis and rotation values.
     *
     * @param axisX x coordinate of rotation axis.
     * @param axisY y coordinate of rotation axis.
     * @param axisZ z coordinate of rotation axis.
     * @param theta rotation angle expressed in radians.
     */    
    public final void setFromAxisAndRotation(double axisX, double axisY, 
            double axisZ, double theta) {
        setFromAxisAndRotation(axisX, axisY, axisZ, theta, null, null);
    }
    
    /**
     * Sets quaternion parameters from axis and rotation values.
     *
     * @param axisX             x coordinate of rotation axis.
     * @param axisY             y coordinate of rotation axis.
     * @param axisZ             z coordinate of rotation axis.
     * @param theta             rotation angle expressed in radians.
     * @param jacobianOfTheta   if provided, matrix where jacobian of rotation
     *                          angle will be stored. Must be a 4x1 matrix.
     * @param jacobianOfAxis    if provided, matrix where jacobian of rotation axis
     *                          will be stored. Must be a 4x3 matrix.
     * @throws IllegalArgumentException if any of the provided jacobian matrices
     * does not have proper size.
     * @see <a href="https://github.com/joansola/slamtb">au2q.m at https://github.com/joansola/slamtb</a>
     */
    public void setFromAxisAndRotation(double axisX, double axisY, 
            double axisZ, double theta, Matrix jacobianOfTheta, 
            Matrix jacobianOfAxis) {
        
        //validations
        if (jacobianOfTheta != null && (jacobianOfTheta.getRows() != N_PARAMS ||
                    jacobianOfTheta.getColumns() != 1)) {
            throw new IllegalArgumentException("jacobian of theta must be 4x1");
        }
        
        if (jacobianOfAxis != null && (jacobianOfAxis.getRows() != N_PARAMS ||
                    jacobianOfAxis.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException("jacobian of axis must be 4x3");
        }        
        
        
        double halfTheta = theta / 2.0;
        double c = Math.cos(halfTheta);
        double s = Math.sin(halfTheta);
        
        mA = c;
        
        mB = axisX * s;
        mC = axisY * s;
        mD = axisZ * s;
        mNormalized = false;
        
        if (jacobianOfTheta != null) {
            double halfC = c / 2.0;
            double halfS = s / 2.0;
        
            jacobianOfTheta.getBuffer()[0] = -halfS;
            jacobianOfTheta.getBuffer()[1] = axisX * halfC;
            jacobianOfTheta.getBuffer()[2] = axisY * halfC;
            jacobianOfTheta.getBuffer()[3] = axisZ * halfC;            
        }
        
        if (jacobianOfAxis != null) {
            jacobianOfAxis.initialize(0.0);
            jacobianOfAxis.setElementAt(1, 0, s);
            jacobianOfAxis.setElementAt(2, 1, s);
            jacobianOfAxis.setElementAt(3, 2, s);            
        }
    }
    
    /**
     * Sets quaternion parameters from axis and rotation values.
     *
     * @param axis  axis values.
     * @param theta rotation angle expressed in radians.
     * @throws IllegalArgumentException if provided axis array does not have 
     * length 3.
     */
    public final void setFromAxisAndRotation(double[] axis, double theta) {
        setFromAxisAndRotation(axis, theta, null, null);
    }
    
    /**
     * Sets quaternion parameters from axis and rotation values.
     *
     * @param axis              axis values.
     * @param theta             rotation angle expressed in radians.
     * @param jacobianOfTheta   if provided, matrix where jacobian of rotation
     *                          angle will be stored. Must be a 4x1 matrix.
     * @param jacobianOfAxis    if provided, matrix where jacobian of rotation axis
     *                          will be stored. Must be a 4x4 matrix.
     * @throws IllegalArgumentException if provided axis array does not have 
     * length 3 or if if any of the provided jacobian matrices
     * does not have proper size.
     */    
    public void setFromAxisAndRotation(double[] axis, double theta, 
            Matrix jacobianOfTheta, Matrix jacobianOfAxis) {
        if (axis.length != AxisRotation3D.AXIS_PARAMS) {
            throw new IllegalArgumentException("axis length must be 3");
        }
        
        setFromAxisAndRotation(axis[0], axis[1], axis[2], theta, 
                jacobianOfTheta, jacobianOfAxis);        
    }
    
    /**
     * Sets quaternion parameters from an axis 3D rotation.
     *
     * @param axisRotation an axis 3D rotation.
     */
    public final void setFromAxisAndRotation(AxisRotation3D axisRotation) {
        setFromAxisAndRotation(axisRotation, null, null);
    }
    
    /**
     * Sets quaternion parameters from an axis 3D rotation.
     *
     * @param axisRotation      an axis 3D rotation.
     * @param jacobianOfTheta   if provided, matrix where jacobian of rotation
     *                          angle will be stored. Must be a 4x1 matrix.
     * @param jacobianOfAxis    if provided, matrix where jacobian of rotation axis
     *                          will be stored. Must be a 4x4 matrix.
     * @throws IllegalArgumentException if any of the provided jacobian matrices
     * does not have proper size.
     */
    public void setFromAxisAndRotation(AxisRotation3D axisRotation,
            Matrix jacobianOfTheta, Matrix jacobianOfAxis) {

        double theta = axisRotation.getRotationAngle();
        
        setFromAxisAndRotation(axisRotation.getAxisX(), axisRotation.getAxisY(),
                axisRotation.getAxisZ(), theta, jacobianOfTheta, 
                jacobianOfAxis);                
    }
    
    /**
     * Multiplies this quaternion with provided one and stores the result in
     * this instance.
     *
     * @param q quaternion to multiply with.
     */
    public void multiply(Quaternion q) {
        multiply(q, this);
    }
        
    /**
     * Multiplies this quaternion with provided one and returns the result as a
     * new quaternion instance.
     *
     * @param q quaternion to multiply with.
     * @return obtained result.
     */
    public Quaternion multiplyAndReturnNew(Quaternion q) {
        Quaternion result = new Quaternion(0.0, 0.0, 0.0, 0.0);
        multiply(q, result);
        return result;
    }
    
    /**
     * Multiplies this quaternion with provided one and stores the result into
     * provided instance.
     *
     * @param q         quaternion to multiply with.
     * @param result    instance where result is stored.
     */
    public void multiply(Quaternion q, Quaternion result) {
        product(this, q, result);
    }    
    
    /**
     * Multiplies quaternion q1 with quaternion q2 and stores the result into
     * provided instance.
     *
     * @param q1        1st product operator of quaternions.
     * @param q2        2nd product operator of quaternions.
     * @param result    instance where result of product is stored.
     */
    public static void product(Quaternion q1, Quaternion q2, Quaternion result) {
        product(q1, q2, result, null, null);
    }
    
    /**
     * Multiplies quaternion q1 with quaternion q2 and stores the result into
     * provided instance. This method also computes the Jacobians wrt of Q1 and
     * Q2 if provided.
     *
     * @param q1            1st productor operator of quaternions.
     * @param q2            2nd productor operator of quaternions.
     * @param result        instance where result of product is stored.
     * @param jacobianQ1    instance where jacobian of q1 is stored.
     * @param jacobianQ2    instance where jacobian of q2 is stored.
     * @throws IllegalArgumentException if any of the provided jacobian matrices
     * is not 4x4.
     * @see <a href="https://github.com/joansola/slamtb">qProd.m at https://github.com/joansola/slamtb</a>
     */
    public static void product(Quaternion q1, Quaternion q2, Quaternion result,
            Matrix jacobianQ1, Matrix jacobianQ2) {
        
        if (jacobianQ1 != null && (jacobianQ1.getRows() != Quaternion.N_PARAMS ||
                jacobianQ1.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian of q1 must be 4x4");
        }
        if (jacobianQ2 != null && (jacobianQ2.getRows() != Quaternion.N_PARAMS ||
                jacobianQ2.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian of q2 must be 4x4");
        }
        
        double q1A = q1.mA;
        double q1B = q1.mB;
        double q1C = q1.mC;
        double q1D = q1.mD;
        double q2A = q2.mA;
        double q2B = q2.mB;
        double q2C = q2.mC;
        double q2D = q2.mD;
        
        result.mA = q1A * q2A - q1B * q2B - q1C * q2C - q1D * q2D;
        result.mB = q1A * q2B + q1B * q2A + q1C * q2D - q1D * q2C;
        result.mC = q1A * q2C - q1B * q2D + q1C * q2A + q1D * q2B;
        result.mD = q1A * q2D + q1B * q2C - q1C * q2B + q1D * q2A;
        result.mNormalized = false;
        
        if (jacobianQ1 != null) {
            jacobianQ1.setElementAt(0, 0, q2A);
            jacobianQ1.setElementAt(1, 0, q2B);
            jacobianQ1.setElementAt(2, 0, q2C);
            jacobianQ1.setElementAt(3, 0, q2D);
            
            jacobianQ1.setElementAt(0, 1, -q2B);
            jacobianQ1.setElementAt(1, 1, q2A);
            jacobianQ1.setElementAt(2, 1, -q2D);
            jacobianQ1.setElementAt(3, 1, q2C);
            
            jacobianQ1.setElementAt(0, 2, -q2C);
            jacobianQ1.setElementAt(1, 2, q2D);
            jacobianQ1.setElementAt(2, 2, q2A);
            jacobianQ1.setElementAt(3, 2, -q2B);
            
            jacobianQ1.setElementAt(0, 3, -q2D);
            jacobianQ1.setElementAt(1, 3, -q2C);
            jacobianQ1.setElementAt(2, 3, q2B);
            jacobianQ1.setElementAt(3, 3, q2A);
        }
        
        if (jacobianQ2 != null) {
            jacobianQ2.setElementAt(0, 0, q1A);
            jacobianQ2.setElementAt(1, 0, q1B);
            jacobianQ2.setElementAt(2, 0, q1C);
            jacobianQ2.setElementAt(3, 0, q1D);
            
            jacobianQ2.setElementAt(0, 1, -q1B);
            jacobianQ2.setElementAt(1, 1, q1A);
            jacobianQ2.setElementAt(2, 1, q1D);
            jacobianQ2.setElementAt(3, 1, -q1C);
            
            jacobianQ2.setElementAt(0, 2, -q1C);
            jacobianQ2.setElementAt(1, 2, -q1D);
            jacobianQ2.setElementAt(2, 2, q1A);
            jacobianQ2.setElementAt(3, 2, q1B);
            
            jacobianQ2.setElementAt(0, 3, -q1D);
            jacobianQ2.setElementAt(1, 3, q1C);
            jacobianQ2.setElementAt(2, 3, -q1B);
            jacobianQ2.setElementAt(3, 3, q1A);
        }
    }
    
    /**
     * Sets quaternion from euler angles (roll, pitch and yaw).
     *
     * @param roll      roll angle expressed in radians. Rotation around x axis.
     * @param pitch     pitch angle expressed in radians. Rotation around y axis.
     * @param yaw       yaw angle expressed in radians. Rotation around z axis.
     * @param jacobian  matrix where jacobian will be stored if provided.
     * @throws IllegalArgumentException if provided jacobian matrix does not 
     * have size 4x3
     * @see <a href="https://github.com/joansola/slamtb">e2q.m at https://github.com/joansola/slamtb</a>
     */
    public void setFromEulerAngles(double roll, double pitch, double yaw, 
            Matrix jacobian) {
        
        if (jacobian != null && (jacobian.getRows() != N_PARAMS ||
                jacobian.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException("jacobian must be 4x3");
        }
        
        //roll rotation on X axis
        Quaternion qx = new Quaternion(new double[]{1.0, 0.0, 0.0}, roll);
        //pitch rotation on Y axis
        Quaternion qy = new Quaternion(new double[]{0.0, 1.0, 0.0}, pitch);
        //yaw rotation on Z axis (qProd(qProd(qz, qy), qx)
        Quaternion qz = new Quaternion(new double[]{0.0, 0.0, 1.0}, yaw);        
        
        product(qz, qy, this);
        product(this, qx, this);
        normalize();
                
        if (jacobian != null) {
            double halfRoll = roll / 2.0;
            double halfPitch = pitch / 2.0;
            double halfYaw = yaw / 2.0;
            
            double sr = Math.sin(halfRoll);
            double sp = Math.sin(halfPitch);
            double sy = Math.sin(halfYaw);
            
            double cr = Math.cos(halfRoll);
            double cp = Math.cos(halfPitch);
            double cy = Math.cos(halfYaw);
            
            jacobian.setElementAt(0, 0, 0.5 * (-cy * cp * sr + sy * sp *cr));
            jacobian.setElementAt(1, 0, 0.5 * (cy * cp * cr + sy * sp * sr));
            jacobian.setElementAt(2, 0, 0.5 * (-cy * sp * sr + sy * cp * cr));
            jacobian.setElementAt(3, 0, 0.5 * (-sy * cp * sr - cy * sp * cr));
            
            jacobian.setElementAt(0, 1, 0.5 * (-cy * sp * cr + sy * cp * sr));
            jacobian.setElementAt(1, 1, 0.5 * (-cy * sp * sr - sy * cp * cr));
            jacobian.setElementAt(2, 1, 0.5 * (cy * cp * cr - sy * sp * sr));
            jacobian.setElementAt(3, 1, 0.5 * (-cy * cp * sr - sy * sp * cr));
            
            jacobian.setElementAt(0, 2, 0.5 * (-sy * cp * cr + cy * sp * sr));
            jacobian.setElementAt(1, 2, jacobian.getElementAt(3, 0));
            jacobian.setElementAt(2, 2, 0.5 * (-sy * sp * cr + cy * cp * sr));
            jacobian.setElementAt(3, 2, jacobian.getElementAt(1, 0));
        }
    }
    
    /**
     * Sets quaternion from euler angles (roll, pitch and yaw).
     *
     * @param roll  roll angle expressed in radians. Rotation around x axis.
     * @param pitch pitch angle expressed in radians. Rotation around y axis.
     * @param yaw   yaw angle expressed in radians. Rotation around z axis.
     * @see <a href="https://github.com/joansola/slamtb">e2q.m at https://github.com/joansola/slamtb</a>
     */
    public final void setFromEulerAngles(double roll, double pitch, double yaw) {
        setFromEulerAngles(roll, pitch, yaw, null);
    }
    
    /**
     * Sets quaternion from euler angles.
     *
     * @param angles    euler angles expressed in radians in the following order:
     *                  roll, pitch and yaw.
     * @param jacobian  matrix where jacobian will be stored if provided.
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void setFromEulerAngles(double[] angles, Matrix jacobian) {
        if (angles.length != N_ANGLES) {
            throw new IllegalArgumentException("angles length must be 3");
        }
        
        setFromEulerAngles(angles[0], angles[1], angles[2], jacobian);
    }
    
    /**
     * Sets quaternion from euler angles (roll, pitch and yaw).
     *
     * @param angles    euler angles expressed in radians in the following order:
     *                  roll, pitch and yaw.
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void setFromEulerAngles(double[] angles) {
        setFromEulerAngles(angles, null);
    }   
    
    /**
     * Computes the rotation matrix body-to-world corresponding to the body
     * orientation given by the Euler angles (roll, pitch, yaw).
     *
     * @param roll      roll angle expressed in radians. Rotation around x axis.
     * @param pitch     pitch angle expressed in radians. Rotation around y axis.
     * @param yaw       yaw angle expressed in radians. Rotation around z axis.
     * @param result    instance where computed rotation will be stored.
     * @param jacobian  jacobian of computed rotation (optional).
     * @throws IllegalArgumentException if provided jacobian is not 9x3.
     * @see <a href="https://github.com/joansola/slamtb">e2R.m at https://github.com/joansola/slamtb</a>
     */
    public static void eulerToMatrixRotation(double roll, double pitch, 
            double yaw, MatrixRotation3D result, Matrix jacobian) {
        
        if (jacobian != null && (jacobian.getRows() != 3 * N_ANGLES ||
                jacobian.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException("jacobian must be 9x3");
        }        
        
        result.setRollPitchYaw(roll, pitch, yaw);
                
        if (jacobian != null) {
            double sr = Math.sin(roll);
            double cr = Math.cos(roll);
            double sp = Math.sin(pitch);
            double cp = Math.cos(pitch);
            double sy = Math.sin(yaw);
            double cy = Math.cos(yaw);
            
            jacobian.setElementAt(0, 0, 0.0);
            jacobian.setElementAt(1, 0, 0.0);
            jacobian.setElementAt(2, 0, 0.0);            
            jacobian.setElementAt(3, 0, sr * sy + cr * sp * cy);
            jacobian.setElementAt(4, 0, -sr * cy + cr * sp * sy);
            jacobian.setElementAt(5, 0, cr * cp);            
            jacobian.setElementAt(6, 0, cr * sy - sr * sp * cy);
            jacobian.setElementAt(7, 0, -cr * cy - sr * sp * sy);
            jacobian.setElementAt(8, 0, -sr * cp);
            
            jacobian.setElementAt(0, 1, -sp * cy);
            jacobian.setElementAt(1, 1, -sp * sy);
            jacobian.setElementAt(2, 1, -cp);
            jacobian.setElementAt(3, 1, sr * cp * cy);
            jacobian.setElementAt(4, 1, sr * cp * sy);
            jacobian.setElementAt(5, 1, -sr * sp);
            jacobian.setElementAt(6, 1, cr * cp * cy);
            jacobian.setElementAt(7, 1, cr * cp * sy);
            jacobian.setElementAt(8, 1, -cr * sp);
            
            jacobian.setElementAt(0, 2, -cp * sy);
            jacobian.setElementAt(1, 2, cp * cy);
            jacobian.setElementAt(2, 2, 0.0);
            jacobian.setElementAt(3, 2, -cr * cy - sr * sp * sy);
            jacobian.setElementAt(4, 2, -cr * sy + sr * sp * cy);
            jacobian.setElementAt(5, 2, 0.0);
            jacobian.setElementAt(6, 2, sr * cy - cr * sp * sy);
            jacobian.setElementAt(7, 2, sr * sy + cr * sp * cy);
            jacobian.setElementAt(8, 2, 0.0);
        }
    }
    
    /**
     * Computes the rotation matrix body-to-world corresponding to the body
     * orientation given by the Euler angles (roll, pitch, yaw).
     *
     * @param roll      roll angle expressed in radians. Rotation around x axis.
     * @param pitch     pitch angle expressed in radians. Rotation around y axis.
     * @param yaw       yaw angle expressed in radians. Rotation around z axis.
     * @param result    instance where computed rotation will be stored.
     * @see <a href="https://github.com/joansola/slamtb">e2R.m at https://github.com/joansola/slamtb</a>
     */
    public static void eulerToMatrixRotation(double roll, double pitch,
            double yaw, MatrixRotation3D result) {
        eulerToMatrixRotation(roll, pitch, yaw, result, null);
    }
    
    /**
     * Computes the rotation matrix body-to-world corresponding to the body
     * orientation given by the Euler angles (roll, pitch, yaw).
     *
     * @param angles    array containing roll, pitch and yaw angles.
     * @param result    instance where computed rotation will be stored.
     * @param jacobian  jacobian of computed rotation (optional).
     * @throws IllegalArgumentException if provided angles length is not 3, or 
     * if provided jacobian is not 9x3.
     * @see <a href="https://github.com/joansola/slamtb">e2R.m at https://github.com/joansola/slamtb</a>
     */
    public static void eulerToMatrixRotation(double[] angles, 
            MatrixRotation3D result, Matrix jacobian) {
        if (angles.length != N_ANGLES) {
            throw new IllegalArgumentException("angles must have length 3");
        }
        
        eulerToMatrixRotation(angles[0], angles[1], angles[2], result, 
                jacobian);
    }
    
    /**
     * Computes the rotation matrix body-to-world corresponding to the body
     * orientation given by the Euler angles (roll, pitch, yaw).
     *
     * @param angles array containing roll, pitch and yaw angles.
     * @param result instance where computed rotation will be stored.
     * @throws IllegalArgumentException if provided angles length is not 3.
     * @see <a href="https://github.com/joansola/slamtb">e2R.m at https://github.com/joansola/slamtb</a>
     */    
    public static void eulerToMatrixRotation(double[] angles, 
            MatrixRotation3D result) {
        eulerToMatrixRotation(angles, result, null);
    }
            
    /**
     * Computes rotation angle and axis of this instance.
     *
     * @param axis          array where normalized rotation axis will be stored.
     * @param jacobianAngle matrix where jacobian of angle will be stored, if
     *                      provided. Must be 1x4.
     * @param jacobianAxis  matrix where jacobian of axis will be stored, if
     *                      provided. Must be 3x4.
     * @return rotation angle expressed in radians.
     * @throws IllegalArgumentException if length of axis or size of provided
     * jacobians is not correct.
     * @see <a href="https://github.com/joansola/slamtb">q2au.m at https://github.com/joansola/slamtb</a>
     */
    public double toAxisAndRotationAngle(double[] axis, Matrix jacobianAngle,
            Matrix jacobianAxis) {
        if (axis.length != AxisRotation3D.AXIS_PARAMS) {
            throw new IllegalArgumentException("axis length must be 3");
        }
        if (jacobianAngle != null && (jacobianAngle.getRows() != 1 ||
                jacobianAngle.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException("jacobian of angle must be 1x4");
        }
        if (jacobianAxis != null &&
                (jacobianAxis.getRows() != AxisRotation3D.AXIS_PARAMS || 
                jacobianAxis.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException("jacobian of axis must be 3x4");
        }
        
        //non-normalized rotation axis
        double[] v = new double[]{ mB, mC, mD };
        
        //norm of rotation axis
        double n = com.irurueta.algebra.Utils.normF(v);
        
        //normalized rotation axis
        ArrayUtils.multiplyByScalar(v, 1.0 / n, axis);
        
        double s = mA; //scalar part        
        double a = 2.0 * Math.atan2(n, s);
        
        if (jacobianAngle != null) {
            if (n > AXIS_NORM_THRESHOLD) {
                double denom = n * n + s * s;
                double aN = 2.0 * s / denom;
                double aS = -2.0 * n / denom;
                double[] aV = ArrayUtils.multiplyByScalarAndReturnNew(axis, aN);
            
                jacobianAngle.setElementAtIndex(0, aS);
                jacobianAngle.setElementAtIndex(1, aV[0]);
                jacobianAngle.setElementAtIndex(2, aV[1]);
                jacobianAngle.setElementAtIndex(3, aV[2]);
            } else {
                jacobianAngle.initialize(0.0);
            }
        }
        
        if (jacobianAxis != null) {
            jacobianAxis.initialize(0.0);            
            
            try {
                if (n > AXIS_NORM_THRESHOLD) {
                    //uV = (eye(3)*n - v * axis') / n^2
                    Matrix uV = Matrix.identity(AxisRotation3D.AXIS_PARAMS, 
                            AxisRotation3D.AXIS_PARAMS);
                    uV.multiplyByScalar(n);
                    uV.subtract(Matrix.newFromArray(v, true).multiplyAndReturnNew(
                            Matrix.newFromArray(axis, false)));
                    uV.multiplyByScalar(1.0 / (n * n));
                    //uQ = [zeros(3, 1) uV]
                    jacobianAxis.setSubmatrix(0, 1, 2, 3, uV);
                } else {
                    //2*eye(3)
                    Matrix m = Matrix.identity(AxisRotation3D.AXIS_PARAMS, 
                            AxisRotation3D.AXIS_PARAMS);
                    m.multiplyByScalar(2.0);
                    //uQ = [zeros(3,1) 2*eye(3)]
                    jacobianAxis.setSubmatrix(0, 1, 2, 3, m);
                }
            } catch (WrongSizeException ignore) { /* never thrown */ }
        }
        
        return a;
    } 
    
    /**
     * Computes rotation angle and axis.
     *
     * @param axis normalized rotation axis.
     * @return rotation angle expressed in radians.
     * @throws IllegalArgumentException if length of axis is not 3.
     */
    public double toAxisAndRotationAngle(double[] axis) {
        return toAxisAndRotationAngle(axis, null, null);
    }
    
    /**
     * Converts this quaternion into an axis 3D rotation and stores the result
     * into provided rotation instance.
     *
     * @param result rotation instance where result will be stored.
     */
    @Override
    public void toAxisRotation(AxisRotation3D result) {
        double[] axis = new double[AxisRotation3D.AXIS_PARAMS];
        double theta = toAxisAndRotationAngle(axis, null, null);
        result.setAxisAndRotation(axis, theta);
    }
    
    /**
     * Converts this quaternion into an axis 3D rotation.
     *
     * @return a new axis 3D rotation equivalent to this quaternion.
     */
    @Override
    public AxisRotation3D toAxisRotation() {
        AxisRotation3D result = new AxisRotation3D();
        toAxisRotation(result);
        return result;
    }
    
    /**
     * Computes rotation vector, which is equivalent to the rotation axis but
     * having a norm equal to the rotation angle.
     *
     * @param result    array where rotation vector is stored.
     * @param jacobian  matrix where jacobian of vector will be stored, if
     *                  provided.
     * @throws IllegalArgumentException if length of result is not 3 or size of 
     * provided jacobian is not 3x4.
     * @see <a href="https://github.com/joansola/slamtb">q2v.m at https://github.com/joansola/slamtb</a>
     */
    public void toRotationVector(double[] result, Matrix jacobian) {
        if (result.length != AxisRotation3D.AXIS_PARAMS) {
            throw new IllegalArgumentException("result length must be 3");
        }
        if (jacobian != null &&
                (jacobian.getRows() != AxisRotation3D.AXIS_PARAMS || 
                jacobian.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException("jacobian must be 3x4");
        }
        
        if (jacobian == null) {
            double theta = toAxisAndRotationAngle(result, null, null);
            ArrayUtils.multiplyByScalar(result, theta, result);
        } else {
            try {
                Matrix jacobianAngle = new Matrix(1, N_PARAMS);
                Matrix jacobianAxis = new Matrix(AxisRotation3D.AXIS_PARAMS, 
                        N_PARAMS);
                double[] axis = new double[AxisRotation3D.AXIS_PARAMS];
                double theta = toAxisAndRotationAngle(axis, jacobianAngle, 
                        jacobianAxis);
                ArrayUtils.multiplyByScalar(axis, theta, result);

                Matrix vA = Matrix.newFromArray(axis, true);
                Matrix vU = Matrix.diagonal(new double[]{theta, theta, theta});

                if (theta > AXIS_NORM_THRESHOLD) {
                    //vA * jacobianAngle + vU * jacobianAxis //3x1 * 1x4 + 3x3 * 3x4
                    vA.multiply(jacobianAngle); //vA * jacobianAngle
                    vU.multiply(jacobianAxis); //vU * jacobianAxis

                    vA.add(vU); //vA * jacobianAnle + vU * jacobianAxis

                    jacobian.copyFrom(vA);
                } else {
                    //2*eye(3)
                    Matrix m = Matrix.identity(AxisRotation3D.AXIS_PARAMS, 
                            AxisRotation3D.AXIS_PARAMS);
                    m.multiplyByScalar(2.0);
                    //uQ = [zeros(3,1) 2*eye(3)]
                    jacobian.setSubmatrix(0, 1, 2, 3, m);                
                }            
            } catch (WrongSizeException ignore) { /* never thrown */ }
        }
    }     
    
    /**
     * Computes rotation vector, which is equivalent to the rotation axis but 
     * having a norm equal to the rotation angle.
     *
     * @param result array where rotation vector is stored.
     * @throws IllegalArgumentException if length of result is not 3.
     */
    public void toRotationVector(double[] result) {
        toRotationVector(result, null);
    }
    
    /**
     * Computes the euler angles (roll, pitch, yaw) equivalent to this 
     * quaternion rotation and stores the result into provided array.
     * If provided, this method also computes the jacobian matrix.
     *
     * @param angles    euler angles (roll, pitch, yaw).
     * @param jacobian  matrix where jacobian is stored, if provided.
     * @throws IllegalArgumentException if provided angles array length is not 3
     * or if provided jacobian matrix is not 3x4.
     * @see <a href="https://github.com/joansola/slamtb">q2e.m at https://github.com/joansola/slamtb</a>
     */
    public void toEulerAngles(double[] angles, Matrix jacobian) {
        if (angles.length != N_ANGLES) {
            throw new IllegalArgumentException("angles length must be 3");
        }
        if (jacobian != null && (jacobian.getRows() != N_ANGLES ||
                jacobian.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException("jacobian must be 3x4");
        }
        
        double y1 = 2.0 * mC * mD + 2.0 * mA * mB;
        double x1 = mA * mA - mB * mB - mC * mC + mD * mD;
        double z2 = -2.0 * mB * mD + 2.0 * mA * mC;
        double y3 = 2.0 * mB * mC + 2.0 * mA * mD;
        double x3 = mA * mA + mB * mB - mC * mC - mD * mD;
        
        //roll
        angles[0] = Math.atan2(y1, x1);
        
        //pitch
        angles[1] = Math.asin(z2);
        
        //yaw
        angles[2] = Math.atan2(y3, x3);
        
        if (jacobian != null) {
            
            double[] dx1dq = new double[]{ 2 * mA, -2 * mB, -2 * mC, 2 * mD };
            double[] dy1dq = new double[]{ 2 * mB, 2 * mA, 2 * mD, 2 * mC };
            double[] dz2dq = new double[]{ 2 * mC, -2 * mD, 2 * mA, -2 * mB };
            double[] dx3dq = new double[]{ 2 * mA, 2 * mB, -2 * mC, -2 * mD };
            double[] dy3dq = new double[]{ 2 * mD, 2 * mC, 2 * mB, 2 * mA };
            
            double de1dx1 = -y1 / (x1 * x1 + y1 * y1);
            double de1dy1 = x1 / (x1 * x1 + y1 * y1);
            double de2dz2 = 1 / Math.sqrt(1 - z2 * z2);
            double de3dx3 = -y3 / (x3 * x3 + y3 * y3);
            double de3dy3 = x3 / (x3 * x3 + y3 * y3);
            
            //de1dq = de1dx1 * dx1dq + de1dy * dy1dq
            ArrayUtils.multiplyByScalar(dx1dq, de1dx1, dx1dq);
            ArrayUtils.multiplyByScalar(dy1dq, de1dy1, dy1dq);            
            double[] de1dq = ArrayUtils.sumAndReturnNew(dx1dq, dy1dq);
            
            //de2dq = de2dz2 * dz2dq
            double[] de2dq = ArrayUtils.multiplyByScalarAndReturnNew(dz2dq, 
                    de2dz2);
            
            //de3dq = de3dx3 * dx3dq + de3dy3 * dy3dq
            ArrayUtils.multiplyByScalar(dx3dq, de3dx3, dx3dq);
            ArrayUtils.multiplyByScalar(dy3dq, de3dy3, dy3dq);
            double[] de3dq = ArrayUtils.sumAndReturnNew(dx3dq, dy3dq);
            
            jacobian.setSubmatrix(0, 0, 0, N_PARAMS - 1, de1dq);
            jacobian.setSubmatrix(1, 0, 1, N_PARAMS - 1, de2dq);
            jacobian.setSubmatrix(2, 0, 2, N_PARAMS - 1, de3dq);
        }
    }
    
    /**
     * Computes the euler angles (roll, pitch, yaw) equivalent to this 
     * quaternion rotation and stores the result into provided array.
     *
     * @param angles euler angles (roll, pitch, yaw).
     * @throws IllegalArgumentException if provided angles array length is not 
     * 3.
     */
    public void toEulerAngles(double[] angles) {
        toEulerAngles(angles, null);
    }
    
    /**
     * Computes the euler angles (roll, pitch, yaw) resulting in an equivalent
     * rotation to this quaternion.
     *
     * @return euler angles (roll, pitch, yaw)
     */
    public double[] toEulerAngles() {
        double[] result = new double[N_ANGLES];
        toEulerAngles(result, null);
        return result;
    }
    
    /**
     * Converts this quaternion into a quaternion matrix so that the quaternion
     * product q1 x q2 is equivalent to the matrix product: 
     * q1.toQuaternionMatrix().multiplyAndReturnNew(q2.toQuaternionMatrix())
     *
     * @param result matrix where result will be stored.
     * @throws IllegalArgumentException if provided matrix is not 4x4.
     * @see <a href="https://github.com/joansola/slamtb">q2Q.m at https://github.com/joansola/slamtb</a>
     */
    public void quaternionMatrix(Matrix result) {
        if (result.getRows() != N_PARAMS || result.getColumns() != N_PARAMS) {
            throw new IllegalArgumentException("matrix must be 4x4");
        }
        
        result.setElementAt(0, 0, mA);
        result.setElementAt(1, 0, mB);
        result.setElementAt(2, 0, mC);
        result.setElementAt(3, 0, mD);
        
        result.setElementAt(0, 1, -mB);
        result.setElementAt(1, 1, mA);
        result.setElementAt(2, 1, mD);
        result.setElementAt(3, 1, -mC);
        
        result.setElementAt(0, 2, -mC);
        result.setElementAt(1, 2, -mD);
        result.setElementAt(2, 2, mA);
        result.setElementAt(3, 2, mB);
        
        result.setElementAt(0, 3, -mD);
        result.setElementAt(1, 3, mC);
        result.setElementAt(2, 3, -mB);
        result.setElementAt(3, 3, mA);
    }
    
    /**
     * Converts this quaternion into a quaternion matrix so that quaternion
     * product q1 x q2 is equivalent to the matrix product: 
     * q1.toQuaternionMatrix().multiplyAndReturnNew(q2.toQuaternionMatrix())
     *
     * @return the quaternion matrix.
     * @see <a href="https://github.com/joansola/slamtb">q2Q.m at https://github.com/joansola/slamtb</a>
     */
    public Matrix toQuaternionMatrix() {
        Matrix result = null;
        try{
            result = new Matrix(N_PARAMS, N_PARAMS);
            quaternionMatrix(result);
        }catch(WrongSizeException ignore){ /* never thrown */ }
        return result;
    }
        
    /**
     * Computes the conjugate of this quaternion and stores the result into 
     * provided instance.
     *
     * @param result    instance where result is stored.
     * @param jacobian  matrix where jacobian is stored.
     * @throws IllegalArgumentException if provided jacobian matrix is not 4x4.
     * @see <a href="https://github.com/joansola/slamtb">q2qc.m at https://github.com/joansola/slamtb</a>
     */
    public void conjugate(Quaternion result, Matrix jacobian) {
        if (jacobian != null && (jacobian.getRows() != N_PARAMS ||
                jacobian.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException("jacobian must be 4x4");
        }
        
        result.mA = mA;
        result.mB = -mB;
        result.mC = -mC;
        result.mD = -mD;
        result.mNormalized = mNormalized;
        
        if (jacobian != null) {
            jacobian.initialize(0.0);
            jacobian.setElementAt(0, 0, 1.0);
            for (int i = 1; i < N_PARAMS; i++) {
                jacobian.setElementAt(i, i, -1.0);
            }
        }
    }
    
    /**
     * Computes the conjugate of this quaternion and stores the result into 
     * provided instance.
     *
     * @param result instance where result is stored.
     * @see <a href="https://github.com/joansola/slamtb">q2qc.m at https://github.com/joansola/slamtb</a>
     */    
    public void conjugate(Quaternion result) {
        conjugate(result, null);
    }
    
    /**
     * Computes the conjugate of this quaternion.
     *
     * @return conjugate of this quaternion.
     * @see <a href="https://github.com/joansola/slamtb">q2qc.m at https://github.com/joansola/slamtb</a>
     */
    public Quaternion conjugateAndReturnNew() {
        Quaternion q = new Quaternion();
        conjugate(q);
        return q;
    }
    
    /**
     * Converts this quaternion into a quaternion matrix so that the quaternion
     * product q1 x q2 is equivalent to the matrix product: 
     * q2.toQuaternionMatrixN().multiplyAndReturnNew(q1.toQuaternionMatrixN()).
     * Notice that matrix order in the product is the opposite as the order used
     * when multiplying matrices obtained by method #toQuaternionMatrix().
     *
     * @param result matrix where result will be stored.
     * @throws IllegalArgumentException if provided matrix is not 4x4.
     * @see <a href="https://github.com/joansola/slamtb">q2Qn.m at https://github.com/joansola/slamtb</a>
     */
    public void quaternionMatrixN(Matrix result) {
        if (result.getRows() != N_PARAMS || result.getColumns() != N_PARAMS) {
            throw new IllegalArgumentException("matrix must be 4x4");
        }
        
        result.setElementAt(0, 0, mA);
        result.setElementAt(1, 0, mB);
        result.setElementAt(2, 0, mC);
        result.setElementAt(3, 0, mD);
        
        result.setElementAt(0, 1, -mB);
        result.setElementAt(1, 1, mA);
        result.setElementAt(2, 1, -mD);
        result.setElementAt(3, 1, mC);
        
        result.setElementAt(0, 2, -mC);
        result.setElementAt(1, 2, mD);
        result.setElementAt(2, 2, mA);
        result.setElementAt(3, 2, -mB);
        
        result.setElementAt(0, 3, -mD);
        result.setElementAt(1, 3, -mC);
        result.setElementAt(2, 3, mB);
        result.setElementAt(3, 3, mA);
    }
    
    /**
     * Converts this quaternion into a quaternion matrix so that quaternion
     * product q1 x q2 is equivalent to the matrix product: 
     * q2.toQuaternionMatrixN().multiplyAndReturnNew(q1.toQuaternionMatrixN()).
     * Notice that matrix order in the product is the opposite as the order used
     * when multiplying matrices obtained by method #toQuaternionMatrix().
     *
     * @return the quaternion matrix.
     * @see <a href="https://github.com/joansola/slamtb">q2Qn.m at https://github.com/joansola/slamtb</a>
     */
    public Matrix toQuaternionMatrixN() {
        Matrix result = null;
        try {
            result = new Matrix(N_PARAMS, N_PARAMS);
            quaternionMatrixN(result);
        } catch (WrongSizeException ignore){ /* never thrown */ }
        return result;
    }
    
    /**
     * Computes the matrix representing this quaternion rotation.
     *
     * @param result    matrix where rotation data will be stored.
     * @param jacobian  jacobian wrt of this quaternion.
     * @throws IllegalArgumentException if provided result matrix is not 3x3 o
     * jacobian matrix is not 9x4.
     * @see <a href="https://github.com/joansola/slamtb">q2R.m at https://github.com/joansola/slamtb</a>
     */
    public void toMatrixRotation(Matrix result, Matrix jacobian) {
        if (result.getRows() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS ||
                result.getColumns() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS) {
            throw new IllegalArgumentException("result matrix is not 3x3");
        }
        if (jacobian != null && (jacobian.getRows() != 9 ||
                jacobian.getColumns() != 4)) {
            throw new IllegalArgumentException("jacobian matrix is not 9x4");
        }
        
        double aa = mA * mA;
        double ab = 2.0 * mA * mB;
        double ac = 2.0 * mA * mC;
        double ad = 2.0 * mA * mD;
        double bb = mB * mB;
        double bc = 2.0 * mB * mC;
        double bd = 2.0 * mB * mD;
        double cc = mC * mC;
        double cd = 2.0 * mC * mD;
        double dd = mD * mD;
        
        result.setElementAt(0, 0, aa + bb - cc - dd);
        result.setElementAt(1, 0, bc + ad);
        result.setElementAt(2, 0, bd - ac);
        
        result.setElementAt(0, 1, bc - ad);
        result.setElementAt(1, 1, aa - bb + cc - dd);
        result.setElementAt(2, 1, cd + ab);
        
        result.setElementAt(0, 2, bd + ac);
        result.setElementAt(1, 2, cd - ab);
        result.setElementAt(2, 2, aa - bb - cc + dd);
        
        if (jacobian != null) {
            double a2 = 2.0 * mA;
            double b2 = 2.0 * mB;
            double c2 = 2.0 * mC;
            double d2 = 2.0 * mD;
            
            jacobian.setElementAt(0, 0, a2);
            jacobian.setElementAt(1, 0, d2);
            jacobian.setElementAt(2, 0, -c2);
            jacobian.setElementAt(3, 0, -d2);
            jacobian.setElementAt(4, 0, a2);
            jacobian.setElementAt(5, 0, b2);
            jacobian.setElementAt(6, 0, c2);
            jacobian.setElementAt(7, 0, -b2);
            jacobian.setElementAt(8, 0, a2);
            
            jacobian.setElementAt(0, 1, b2);
            jacobian.setElementAt(1, 1, c2);
            jacobian.setElementAt(2, 1, d2);
            jacobian.setElementAt(3, 1, c2);
            jacobian.setElementAt(4, 1, -b2);
            jacobian.setElementAt(5, 1, a2);
            jacobian.setElementAt(6, 1, d2);
            jacobian.setElementAt(7, 1, -a2);
            jacobian.setElementAt(8, 1, -b2);
            
            jacobian.setElementAt(0, 2, -c2);
            jacobian.setElementAt(1, 2, b2);
            jacobian.setElementAt(2, 2, -a2);
            jacobian.setElementAt(3, 2, b2);
            jacobian.setElementAt(4, 2, c2);
            jacobian.setElementAt(5, 2, d2);
            jacobian.setElementAt(6, 2, a2);
            jacobian.setElementAt(7, 2, d2);
            jacobian.setElementAt(8, 2, -c2);
            
            jacobian.setElementAt(0, 3, -d2);
            jacobian.setElementAt(1, 3, a2);
            jacobian.setElementAt(2, 3, b2);
            jacobian.setElementAt(3, 3, -a2);
            jacobian.setElementAt(4, 3, -d2);
            jacobian.setElementAt(5, 3, c2);
            jacobian.setElementAt(6, 3, b2);
            jacobian.setElementAt(7, 3, c2);
            jacobian.setElementAt(8, 3, d2);
        }
    }
    
    /**
     * Computes the matrix representing this quaternion rotation.
     *
     * @param result matrix where rotation data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     * @see <a href="https://github.com/joansola/slamtb">q2R.m at https://github.com/joansola/slamtb</a>
     */
    public void toMatrixRotation(Matrix result) {
        toMatrixRotation(result, null);
    }
    
    /**
     * Converts this quaternion into a 3D matrix rotation.
     *
     * @param result matrix rotation instance where result will be stored.
     * @see <a href="https://github.com/joansola/slamtb">q2R.m at https://github.com/joansola/slamtb</a>
     */
    @Override
    public void toMatrixRotation(MatrixRotation3D result) {
        toMatrixRotation(result.internalMatrix);
    }
    
    /**
     * Converts this quaternion into a 3D matrix rotation.
     *
     * @return a 3D matrix rotation.
     * @see <a href="https://github.com/joansola/slamtb">q2R.m at https://github.com/joansola/slamtb</a>
     */
    @Override
    public MatrixRotation3D toMatrixRotation() {
        MatrixRotation3D rotation = new MatrixRotation3D();
        toMatrixRotation(rotation);
        return rotation;
    }
    
    /**
     * Rotates a 3D point using the origin of coordinates as the axis of 
     * rotation.
     * Point will be rotated by the amount of rotation contained in provided
     * quaternion.
     *
     * @param q                     a quaternion.
     * @param inputPoint            input point to be rotated.
     * @param resultPoint           rotated point.
     * @param jacobianPoint         jacobian wrt of point.
     * @param jacobianQuaternion    jacobian wrt of quaternion.
     * @throws IllegalArgumentException if jacobian of point is not 3x3 or
     * jacobian of quaternoin is not 3x4.
     * @see <a href="https://github.com/joansola/slamtb">qRot.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotate(Quaternion q, Point3D inputPoint, 
            Point3D resultPoint, Matrix jacobianPoint, 
            Matrix jacobianQuaternion) {
        if (jacobianPoint != null && (jacobianPoint.getRows() != N_ANGLES ||
                jacobianPoint.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException("jacobian of point must be 3x3");
        }
        if (jacobianQuaternion != null &&
                (jacobianQuaternion.getRows() != N_ANGLES ||
                jacobianQuaternion.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException(
                    "jacobian of quaternion must be 3x4");
        }
        
        Quaternion v0 = new Quaternion(0.0, inputPoint.getInhomX(), 
                inputPoint.getInhomY(), inputPoint.getInhomZ());
        
        Quaternion tmp = q.multiplyAndReturnNew(v0).multiplyAndReturnNew(
                q.conjugateAndReturnNew());

        resultPoint.setInhomogeneousCoordinates(tmp.getB(), tmp.getC(), 
                tmp.getD());      
        
        if (jacobianPoint != null) {
            q.toMatrixRotation(jacobianPoint);
        }
        
        if (jacobianQuaternion != null) {
            double a = q.mA;
            double b = q.mB;
            double c = q.mC;
            double d = q.mD;
            
            double x = inputPoint.getInhomX();
            double y = inputPoint.getInhomY();
            double z = inputPoint.getInhomZ();
            
            double axdycz = 2.0 * (a * x - d * y + c * z);
            double bxcydz = 2.0 * (b * x + c * y + d * z);
            double cxbyaz = 2.0 * (c * x - b * y - a * z);
            double dxaybz = 2.0 * (d * x + a * y - b * z);
            
            jacobianQuaternion.setElementAt(0, 0, axdycz);
            jacobianQuaternion.setElementAt(1, 0, dxaybz);
            jacobianQuaternion.setElementAt(2, 0, -cxbyaz);
            
            jacobianQuaternion.setElementAt(0, 1, bxcydz);
            jacobianQuaternion.setElementAt(1, 1, cxbyaz);
            jacobianQuaternion.setElementAt(2, 1, dxaybz);
            
            jacobianQuaternion.setElementAt(0, 2, -cxbyaz);
            jacobianQuaternion.setElementAt(1, 2, bxcydz);
            jacobianQuaternion.setElementAt(2, 2, -axdycz);
            
            jacobianQuaternion.setElementAt(0, 3, -dxaybz);
            jacobianQuaternion.setElementAt(1, 3, axdycz);
            jacobianQuaternion.setElementAt(2, 3, bxcydz);
        }
    }
    
    /**
     * Rotates a 3D point using the origin of coordinates as the axis of 
     * rotation.
     * Point will be rotated by the amount of rotation contained in this 
     * quaternion instance.
     *
     * @param inputPoint            input point to be rotated.
     * @param resultPoint           rotated point.
     * @param jacobianPoint         jacobian wrt of point.
     * @param jacobianQuaternion    jacobian wrt of quaternion.
     * @throws IllegalArgumentException if jacobian of point is not 3x3 or
     * jacobian of quaternion is no 3x4.
     * @see <a href="https://github.com/joansola/slamtb">qRot.m at https://github.com/joansola/slamtb</a>
     */
    public void rotate(Point3D inputPoint, Point3D resultPoint, 
            Matrix jacobianPoint, Matrix jacobianQuaternion) {
        
        rotate(this, inputPoint, resultPoint, jacobianPoint, 
                jacobianQuaternion);
    }
    
    
    /**
     * Rotates a 3D point using the origin of coordinates as the axis of 
     * rotation.
     * Point will be rotated by the amount of rotation contained in this 
     * quaternion instance.
     *
     * @param inputPoint    Input point to be rotated.
     * @param resultPoint   Rotated point.
     * @see <a href="https://github.com/joansola/slamtb">qRot.m at https://github.com/joansola/slamtb</a>
     */ 
    @Override
    public void rotate(Point3D inputPoint, Point3D resultPoint) {
        rotate(inputPoint, resultPoint, null, null);
    }
    
    /**
     * Returns a 3D point containing a rotated version of provided point.
     * Point will be rotated using the origin of the coordinates as the axis of
     * rotation.
     * Point will be rotated by the amount of rotation contained in this 
     * quaternion instance.
     *
     * @param point Point to be rotated.
     * @return Rotated point.
     * @see <a href="https://github.com/joansola/slamtb">qRot.m at https://github.com/joansola/slamtb</a>
     */  
    @Override
    public Point3D rotate(Point3D point) {
        Point3D result = new HomogeneousPoint3D();
        rotate(point, result);
        return result;        
    }    
    
    /**
     * Converts rotation matrix into a quaternion.
     *
     * @param r         a rotation matrix to be converted from.
     * @param result    quaternion where result is stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3
     * @see <a href="https://github.com/joansola/slamtb">R2q.m at https://github.com/joansola/slamtb</a>
     */
    public static void matrixRotationToQuaternion(Matrix r, 
            Quaternion result) {
        if (r.getRows() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS ||
                r.getColumns() != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS) {
            throw new IllegalArgumentException("rotation matrix must be 3x3");
        }
        
        double trace = com.irurueta.algebra.Utils.trace(r) + 1.0;
        double s;
        double a;
        double b;
        double c;
        double d;
        
        if (trace > TRACE_THRESHOLD) { //to avoid large distortions
            s = 2.0 * Math.sqrt(trace);
            a = 0.25 * s;
            b = (r.getElementAt(1, 2) - r.getElementAt(2, 1)) / s;
            c = (r.getElementAt(2, 0) - r.getElementAt(0, 2)) / s;
            d = (r.getElementAt(0, 1) - r.getElementAt(1, 0)) / s;
        } else {
            if (r.getElementAt(0, 0) > r.getElementAt(1, 1) &&
                    r.getElementAt(0, 0) > r.getElementAt(2, 2)) {
                //column 1:
                //tested with R2 = diag([1 -1 -1])
                
                s = 2.0 * Math.sqrt(1.0 + r.getElementAt(0, 0) - 
                        r.getElementAt(1, 1) - r.getElementAt(2, 2));
                a = (r.getElementAt(1, 2) - r.getElementAt(2, 1)) / s;
                b = 0.25 * s;
                c = (r.getElementAt(0, 1) + r.getElementAt(1, 0)) / s;
                d = (r.getElementAt(2, 0) + r.getElementAt(0, 2)) / s;
            } else if (r.getElementAt(1, 1) > r.getElementAt(2, 2)) {
                //column 2:
                //tested with R3 = [0 1 0; 1 0 0; 0 0 -1]
                
                s = 2.0 * Math.sqrt(1.0 + r.getElementAt(1, 1) -
                        r.getElementAt(0, 0) - r.getElementAt(2, 2));
                a = (r.getElementAt(2, 0) - r.getElementAt(0, 2)) / s;
                b = (r.getElementAt(0, 1) + r.getElementAt(1, 0)) / s;
                c = 0.25 * s;
                d = (r.getElementAt(1, 2) + r.getElementAt(2, 1)) / s;
            } else {
                //column 3:
                //tested with R4 = [-1 0 0; 0 0 1; 0 1 0]
                
                s = 2.0 * Math.sqrt(1.0 + r.getElementAt(2, 2) - 
                        r.getElementAt(0, 0) - r.getElementAt(1, 1));
                a = (r.getElementAt(0, 1) - r.getElementAt(1, 0)) / s;
                b = (r.getElementAt(2, 0) + r.getElementAt(0, 2)) / s;
                c = (r.getElementAt(1, 2) + r.getElementAt(2, 1)) / s;
                d = 0.25 * s;
            }
        }
        
        result.mA = a;
        result.mB = -b;
        result.mC = -c;
        result.mD = -d;
        result.mNormalized = false;
    }
    
    /**
     * Converts 3D matrix rotation into a quaternion.
     *
     * @param rotation  a 3D matrix rotation to be converted from.
     * @param result    quaternion where result is stored.
     * @see <a href="https://github.com/joansola/slamtb">R2q.m at https://github.com/joansola/slamtb</a>
     */
    public static void matrixRotationToQuaternion(MatrixRotation3D rotation,
            Quaternion result) {
        matrixRotationToQuaternion(rotation.internalMatrix, result);
    }
        
    /**
     * Sets quaternion values associated to provided rotation.
     *
     * @param matrix a rotation matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3. 
     * @see <a href="https://github.com/joansola/slamtb">R2q.m at https://github.com/joansola/slamtb</a>
     */
    public void setFromMatrixRotation(Matrix matrix) {
        matrixRotationToQuaternion(matrix, this);
    }
    
    /**
     * Sets quaternion values associated to provided rotation.
     *
     * @param rotation a rotation to be converted into a quaternion.
     * @see <a href="https://github.com/joansola/slamtb">R2q.m at https://github.com/joansola/slamtb</a>
     */
    public final void setFromMatrixRotation(MatrixRotation3D rotation) {
        matrixRotationToQuaternion(rotation, this);
    }         
    
    /**
     * Converts a rotation vector (rotation axis having a norm equal to the 
     * rotation angle) into a normalized rotation axis and its corresponding 
     * rotation angle.
     *
     * @param rotationVector            input rotation vector to be converted.
     * @param axis                      obtained normalized rotation axis.
     * @param jacobianAlpha             jacobian wrt of angle.
     * @param jacobianRotationVector    jacobian wrt of rotation vector.
     * @return rotation angle.
     * @throws IllegalArgumentException if provided rotation vector length is 
     * not 3, jacobian of angle is not 1x3 or jacobian of rotation vector is
     * not 3x3.
     * @see <a href="https://github.com/joansola/slamtb">v2au.m at https://github.com/joansola/slamtb</a>
     */
    public static double rotationVectorToRotationAxisAndAngle(
            double [] rotationVector, double[] axis, Matrix jacobianAlpha, 
            Matrix jacobianRotationVector) {
        if (rotationVector.length != AxisRotation3D.AXIS_PARAMS) {
            throw new IllegalArgumentException(
                    "rotation vector length must be 3");
        }
        
        if (jacobianAlpha != null && (jacobianAlpha.getRows() != 1 ||
                jacobianAlpha.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException("jacobian alpha must be 1x3");
        }
        
        if (jacobianRotationVector != null &&
                (jacobianRotationVector.getRows() != N_ANGLES || 
                jacobianRotationVector.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException(
                    "jacobian rotation vector must be 3x3");
        }
        
        double alpha = com.irurueta.algebra.Utils.normF(rotationVector);
        
        if (alpha > AXIS_NORM_THRESHOLD) {
            ArrayUtils.multiplyByScalar(rotationVector, 1.0 / alpha, axis);
            
            if (jacobianAlpha != null) {
                jacobianAlpha.setSubmatrix(0, 0, 0,
                        axis.length - 1, axis);
            }
        
            if (jacobianRotationVector != null) {
                jacobianRotationVector.setElementAt(0, 0,
                        1.0 / alpha - axis[0] * axis[0] / alpha);
                jacobianRotationVector.setElementAt(1, 0, 
                        -axis[0] / alpha * axis[1]);
                jacobianRotationVector.setElementAt(2, 0, 
                        -axis[0] / alpha * axis[2]);
                
                jacobianRotationVector.setElementAt(0, 1, 
                        -axis[0] / alpha * axis[1]);
                jacobianRotationVector.setElementAt(1, 1, 
                        1.0 / alpha - axis[1] * axis[1] / alpha);
                jacobianRotationVector.setElementAt(2, 1, 
                        -axis[1] / alpha * axis[2]);
                
                jacobianRotationVector.setElementAt(0, 2, 
                        -axis[0] / alpha * axis[2]);
                jacobianRotationVector.setElementAt(1, 2, 
                        -axis[1] / alpha * axis[2]);
                jacobianRotationVector.setElementAt(2, 2, 
                        1.0 / alpha - axis[2] * axis[2] / alpha);
            }
            
        } else {
            alpha = 0.0;
            Arrays.fill(axis, 0.0);
            
            if (jacobianAlpha != null) {
                jacobianAlpha.initialize(0.0);
            }
        
            if (jacobianRotationVector != null) {
                jacobianRotationVector.initialize(0.0);
            }            
        }

        return alpha;        
    }
    
    /**
     * Converts a rotation vector (rotation axis having a norm equal to the
     * rotation angle) into a normalized rotation axis and its corresponding
     * rotation angle.
     *
     * @param rotationVector    input rotation vector to be converted.
     * @param axis              obtained normalized rotation axis.
     * @return rotation angle.
     * @throws IllegalArgumentException if provided rotation vector length is 
     * not 3.
     * @see <a href="https://github.com/joansola/slamtb">v2au.m at https://github.com/joansola/slamtb</a>
     */
    public static double rotationVectorToRotationAxisAndAngle(
            double[] rotationVector, double[] axis) {
        return rotationVectorToRotationAxisAndAngle(rotationVector, axis, null, 
                null);
    }
    
    /**
     * Converts a rotation vector (rotation axis having a norm equal to the 
     * rotation angle) into a quaternion, and stores the corresponding jacobian
     * of the quaternion respect to the vector if provided.
     *
     * @param rotationVector    input rotation vector to be converted.
     * @param result            quaternion where result will be stored.
     * @param jacobian          if provided, matrix where jacobian of the quaternion
     *                          respect to the vector will be stored. Must be 4x3.
     * @throws IllegalArgumentException if provided rotation vector is not 
     * length 3 or if provided jacobian matrix is not 4x3.
     * @see <a href="https://github.com/joansola/slamtb">v2q.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationVectorToQuaternion(double[] rotationVector, 
            Quaternion result, Matrix jacobian) {
        if (rotationVector.length != AxisRotation3D.AXIS_PARAMS) {
            throw new IllegalArgumentException(
                    "rotation vector length must be 3");
        }
        if (jacobian != null && (jacobian.getRows() != N_PARAMS ||
                jacobian.getColumns() != N_ANGLES)) {
            throw new IllegalArgumentException("jacobian must be 4x3");
        }
        
        double[] axis = new double[AxisRotation3D.AXIS_PARAMS];        
        if (jacobian == null) {
            double alpha = rotationVectorToRotationAxisAndAngle(rotationVector, 
                    axis);
            result.setFromAxisAndRotation(axis, alpha);
        } else {
            double alpha = com.irurueta.algebra.Utils.normF(rotationVector);
            
            if (alpha < LARGE_AXIS_NORM_THRESHOLD) {
                //use small signal approximation
                result.mA = 1 - alpha * alpha / 8.0;
                result.mB = rotationVector[0] / 2.0;
                result.mC = rotationVector[1] / 2.0;
                result.mD = rotationVector[2] / 2.0;
                result.mNormalized = false;
                
                jacobian.setElementAt(0, 0, -0.25 * rotationVector[0]);
                jacobian.setElementAt(0, 1, -0.25 * rotationVector[1]);
                jacobian.setElementAt(0, 2, -0.25 * rotationVector[2]);
                
                jacobian.setElementAt(1, 0, 0.5);
                jacobian.setElementAt(1, 1, 0.0);
                jacobian.setElementAt(1, 2, 0.0);
                
                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.5);
                jacobian.setElementAt(2, 2, 0.0);
                
                jacobian.setElementAt(3, 0, 0.0);
                jacobian.setElementAt(3, 1, 0.0);
                jacobian.setElementAt(3, 2, 0.5);
            } else {
                try {
                    //Av
                    Matrix jacobianAlpha = new Matrix(1, N_ANGLES);
                    //Uv
                    Matrix jacobianRotationVector = new Matrix(N_ANGLES, 
                            N_ANGLES);
                    alpha = rotationVectorToRotationAxisAndAngle(rotationVector, 
                            axis, jacobianAlpha, jacobianRotationVector);
                    
                    //Qa
                    Matrix jacobianOfTheta = new Matrix(N_PARAMS, 1);
                    //Qu
                    Matrix jacobianOfAxis = new Matrix(N_PARAMS, N_ANGLES);
                    result.setFromAxisAndRotation(axis, alpha, jacobianOfTheta, 
                            jacobianOfAxis);
                    
                    //Qv = Qa * Av + Qu * Uv
                    jacobianOfTheta.multiply(jacobianAlpha); //Qa * Av
                    jacobianOfAxis.multiply(jacobianRotationVector); //Qu * Uv
                    
                    jacobian.copyFrom(jacobianOfTheta);
                    jacobian.add(jacobianOfAxis);
                    
                } catch (WrongSizeException e) {
                    throw new IllegalArgumentException(e);
                }
            }
        }
    }
    
    /**
     * Converts a rotation vector (rotation axis having a norm equal to the
     * rotation angle) into a quaternion.
     *
     * @param rotationVector    input rotation vector to be converted.
     * @param result            quaternioln where result will be stored.
     * @throws IllegalArgumentException if provided rotation vector is not 
     * length 3.
     * @see <a href="https://github.com/joansola/slamtb">v2q.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationVectorToQuaternion(double[] rotationVector, 
            Quaternion result) {
        rotationVectorToQuaternion(rotationVector, result, null);
    }
    
    /**
     * Sets values of this quaternion from provided rotation vector.
     * A rotation vector is a rotation axis having a norm equal to the rotation
     * angle.
     *
     * @param rotationVector    input rotation vector to obtain quaternion values
     *                          from.
     * @throws IllegalArgumentException if provided rotation vector does not 
     * have length 3.
     * @see <a href="https://github.com/joansola/slamtb">v2q.m at https://github.com/joansola/slamtb</a>
     */
    public void setFromRotationVector(double[] rotationVector) {
        rotationVectorToQuaternion(rotationVector, this);
    }

    /**
     * Converts a rotation vector into a rotation matrix.
     * A rotation vector is a rotation axis having a norm equal to the rotation
     * angle.
     *
     * @param rotationVector    a rotation vector to be converted into a 3D matrix
     *                          rotation.
     * @param result            matrix where result is stored.
     * @throws IllegalArgumentException if provided rotation vector does not
     * have length 3.
     * @see <a href="https://github.com/joansola/slamtb">v2R.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationVectorToMatrixRotation(double[] rotationVector, 
            Matrix result) {
        double[] axis = new double[AxisRotation3D.AXIS_PARAMS];
        double alpha = rotationVectorToRotationAxisAndAngle(rotationVector, 
                axis);
        AxisRotation3D r = new AxisRotation3D(axis, alpha);
        r.asInhomogeneousMatrix(result);
    }
    
    /**
     * Converts a rotation vector into a rotation matrix.
     * A rotation vector is a rotation axis having a norm equal to the rotation
     * angle.
     *
     * @param rotationVector    a rotation vector to be converted into a 3D matrix
     *                          rotation.
     * @param result            3D matrix rotation where result is stored.
     * @throws IllegalArgumentException if provided rotation vector does not 
     * have length 3.
     * @see <a href="https://github.com/joansola/slamtb">v2R.m at https://github.com/joansola/slamtb</a>
     */
    public static void rotationVectorToMatrixRotation(double[] rotationVector,
            MatrixRotation3D result) {
        rotationVectorToMatrixRotation(rotationVector, result.internalMatrix);
    }

    /**
     * Returns type of this rotation.
     *
     * @return Type of this rotation.
     */    
    @Override
    public Rotation3DType getType() {
        return Rotation3DType.QUATERNION;
    }

    /**
     * Sets the axis and rotation of this instance.
     * Once set, points will rotate around provided axis an amount equal to
     * provided rotation angle in radians.
     * Note: to avoid numerical instabilities and improve accuracy, axis
     * coordinates should be normalized (e.g. norm equal to 1).
     *
     * @param axisX X coordinate of rotation axis.
     * @param axisY Y coordinate of rotation axis.
     * @param axisZ Z coordinate of rotation axis.
     * @param theta Amount of rotation in radians.
     */        
    @Override
    public void setAxisAndRotation(double axisX, double axisY, double axisZ, 
            double theta) {
        setFromAxisAndRotation(axisX, axisY, axisZ, theta);
    }

    /**
     * Returns rotation axis corresponding to this instance.
     * Result is stored in provided axis array, which must have length 3.
     *
     * @param axis Array where axis coordinates will be stored.
     * @throws IllegalArgumentException Raised if provided array does not have
     * length 3.
     */        
    @Override
    public void rotationAxis(double[] axis) {
        toAxisAndRotationAngle(axis);
    }

    /**
     * Returns rotation amount or angle in radians around the rotation axis
     * associated to this instance.
     *
     * @return Rotation angle in radians.
     */        
    @Override
    public double getRotationAngle() {
        //norm of rotation axis
        double n = Math.sqrt(mB * mB + mC * mC + mD * mD);
        return 2.0 * Math.atan2(n, mA);
    }

    /**
     * Returns this 3D rotation instance expressed as a 3x3 inhomogeneous 
     * matrix.
     * This is equivalent to call getInternalMatrix().
     *
     * @return Rotation matrix expressed in inhomogeneous coordinates.
     */        
    @Override
    public Matrix asInhomogeneousMatrix() {
        Matrix m = null;
        try {
            m = new Matrix(MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
            toMatrixRotation(m);
        } catch (WrongSizeException ignore){ /* never thrown */ }
        return m;
    }

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 3x3 inhomogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     * have size 3x3.
     */        
    @Override
    public void asInhomogeneousMatrix(Matrix result) {
        toMatrixRotation(result);
    }

    /**
     * Returns this 3D rotation instance expressed as a 4x4 homogeneous matrix.
     *
     * @return Rotation matrix expressed in homogeneous coordinates.
     */            
    @Override
    public Matrix asHomogeneousMatrix() {        
        Matrix m = null;
        try {
            m = new Matrix(HOM_COORDS, HOM_COORDS);
            asHomogeneousMatrix(m);
        } catch (WrongSizeException ignore) { /* never thrown */ }
        return m;
    }

    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 4x4 homogeneous matrix.
     *
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     * have size 4x4.
     */        
    @Override
    public void asHomogeneousMatrix(Matrix result) {
        result.initialize(0.0);
        result.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);
        result.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, 
                asInhomogeneousMatrix());
    }

    /**
     * Sets amount of rotation from provided inhomogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be it's inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 3x3.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws IllegalArgumentException Raised if provided threshold is
     * negative.
     * {@link #isValidRotationMatrix(Matrix)}
     */            
    @Override
    public void fromInhomogeneousMatrix(Matrix m, double threshold) {
        setFromMatrixRotation(m);
    }

    /**
     * Sets amount of rotation from provided homogeneous rotation matrix.
     * Provided matrix must be orthogonal (i.e. squared, non-singular, it's
     * transpose must be it's inverse) and must have determinant equal to 1.
     * Provided matrix must also have size 4x4, and its last row and column must
     * be zero, except for element in last row and column which must be 1.
     *
     * @param m         Provided rotation matrix.
     * @param threshold Threshold to determine whether matrix is orthonormal.
     * @throws IllegalArgumentException Raised if provided threshold is
     * negative.
     * {@link #isValidRotationMatrix(Matrix)}
     */              
    @Override
    public void fromHomogeneousMatrix(Matrix m, double threshold) {
        setFromMatrixRotation(m.getSubmatrix(0, 0, INHOM_COORDS - 1, 
                INHOM_COORDS - 1));
    }
    
    /**
     * Inverts a quaternion so that q * q^-1 = 1.
     *
     * @param q         quaternion to be inverted.
     * @param result    quaternion instance where the result will be stored.
     */
    public static void inverse(Quaternion q, Quaternion result) {
        //the inverse is the conjugate divided by the quaternion square norm
        double sqrNorm = q.mA * q.mA + q.mB * q.mB + q.mC * q.mC + q.mD * q.mD;
        q.conjugate(result);
        result.mA /= sqrNorm;
        result.mB /= sqrNorm;
        result.mC /= sqrNorm;
        result.mD /= sqrNorm;     
        result.mNormalized = false;
    }
    
    /**
     * Inverts a quaternion so that q * q^-1 = 1.
     *
     * @param q quaternion to be inverted.
     * @return a new quaternion containing the inverse.
     */
    public static Quaternion inverseAndReturnNew(Quaternion q) {
        Quaternion result = new Quaternion();
        inverse(q, result);
        return result;
    }
    
    /**
     * Inverts this quaternion instance so that q * q^-1 = 1.
     *
     * @param result instance where quaternion inverse is stored.
     */
    public void inverse(Quaternion result) {
        inverse(this, result);
    }
    
    /**
     * Inverts this quaternion instance so that q * q^-1 = 1.
     *
     * @return a new quaternion containing the inverse of this quaternion.
     */
    public Quaternion inverseAndReturnNew() {
        Quaternion result = new Quaternion();
        inverse(result);
        return result;
    }
    
    /**
     * Inverts this quaternion.
     */
    public void inverse() {
        inverse(this);
    }
    
    /**
     * Returns a 3D rotation which is inverse to this instance.
     * In other words, the combination of this rotation with its inverse 
     * produces no change.
     *
     * @return Inverse 3D rotation.
     */    
    @Override
    public Rotation3D inverseRotationAndReturnNew() {
        Quaternion q = new Quaternion();
        inverseRotation(q);
        return q;
    }
    
    /**
     * Inverts this quaternion instance so that q * q^-1 = 1.
     *
     * @param result instance where quaternion inverse is stored.
     */
    public void inverseRotation(Quaternion result) {
        inverse(result);
    }

    /**
     * Sets into provided Rotation3D instance a rotation inverse to this 
     * instance.
     * The combination of this rotation with its inverse produces no change.
     *
     * @param result Instance where inverse rotation will be set.
     */        
    @Override
    public void inverseRotation(Rotation3D result) {
        Quaternion inverse = inverseAndReturnNew();
        result.fromRotation(inverse);
    }

    /**
     * Reverses the rotation of this instance.
     */        
    @Override
    public void inverseRotation() {
        inverse();
    }
    
    /**
     * Combines provided quaternions q1 and q2 to produce a resulting quaternion 
     * equivalent to the combined rotation of both quaternions.
     *
     * @param q1        1st quaternion.
     * @param q2        2nd quaternion.
     * @param result    combined quaternion where result is stored.
     */
    public static void combine(Quaternion q1, Quaternion q2, Quaternion result) {
        product(q1, q2, result);
    }
    
    /**
     * Combines provided quaternion with this quaternion and returns the result
     * as a new quaternion instance.
     *
     * @param q input quaternion to be combined.
     * @return combined quaternion, which is equal to the multiplication of
     * quaternions.
     */
    public Quaternion combineAndReturnNew(Quaternion q) {
        Quaternion result = new Quaternion();
        combine(this, q, result);
        return result;
    }
    
    /**
     * Combines provided quaternion into this quaternion, resulting in the
     * multiplication of both quaternion representations.
     *
     * @param q input quaternion to be combined.
     */
    public void combine(Quaternion q) {
        combine(this, q, this);
    }

    /**
     * Combines provided rotation with this quaternion and returns the result as
     * a new quaternion instance.
     *
     * @param rotation input rotation to be combined.
     * @return combined rotation, which is equal to the multiplication of the
     * internal quaternion representations.
     */
    @Override
    public Rotation3D combineAndReturnNew(Rotation3D rotation) {
        return combineAndReturnNew(rotation.toQuaternion());
    }

    /**
     * Combines provided rotation into this quaternion, resulting in the 
     * multiplication of both quaternion representations.
     *
     * @param rotation input rotation to be combined.
     */
    @Override
    public void combine(Rotation3D rotation) {
        combine(rotation.toQuaternion());
    }
    
    /**
     * Sets values of this rotation from a 3D matrix rotation.
     *
     * @param rot 3D matrix rotation to set values from.
     */
    @Override
    public void fromRotation(MatrixRotation3D rot) {
        setFromMatrixRotation(rot);
    }

    /**
     * Sets values of this rotation from a 3D axis rotation.
     *
     * @param rot an axis rotation to set values from.
     */
    @Override
    public void fromRotation(AxisRotation3D rot) {
        setFromAxisAndRotation(rot);
    }
    
    /**
     * Sets values of this rotation from a quaternion.
     *
     * @param q a quaternion to set values from.
     */
    @Override    
    public void fromRotation(Quaternion q) {
        mA = q.mA;
        mB = q.mB;
        mC = q.mC;
        mD = q.mD;
        mNormalized = q.mNormalized;
    }
    
    /**
     * Converts this 3D rotation into a quaternion storing the result into
     * provided instance.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void toQuaternion(Quaternion result) {
        result.fromQuaternion(this);
    } 
    
    /**
     * Indicates whether quaternion is already normalized or not.
     *
     * @return true if quaternion is normalized, false otherwise.
     */
    public boolean isNormalized() {
        return mNormalized;
    }
    
    /**
     * Normalizes this quaternion if not already normalized.
     */
    public void normalize() {
        if (!mNormalized) {
            double norm = Math.sqrt(mA * mA + mB * mB + mC * mC + mD * mD);
            internalNormalize(norm);
        }
    }
    
    /**
     * Normalizes this quaternion if not already normalized and stores the
     * corresponding jacobian into provided matrix (if provided).
     *
     * @param jacobian  matrix where jacobian will be stored (if provided). Must
     *                  be 4x4.
     * @throws IllegalArgumentException if provided jacobian is not 4x4.
     */
    public void normalize(Matrix jacobian) {
        if (jacobian != null && (jacobian.getRows() != N_PARAMS ||
                jacobian.getColumns() != N_PARAMS)) {
            throw new IllegalArgumentException("jacobian must be 4x4");
        }
        
        double a = mA;
        double b = mB;
        double c = mC;
        double d = mD;
        double norm = Math.sqrt(a * a + b * b + c * c + d * d);
        
        internalNormalize(norm);
        
        if (jacobian != null) {
            double norm3 = norm * norm * norm;
            
            jacobian.setElementAt(0, 0, (b * b + c * c + d * d) / norm3);
            jacobian.setElementAt(1, 0, -a / norm3 * b);
            jacobian.setElementAt(2, 0, -a / norm3 * c);
            jacobian.setElementAt(3, 0, -a / norm3 * d);
            
            jacobian.setElementAt(0, 1, -a / norm3 * b);
            jacobian.setElementAt(1, 1, (a * a + c * c + d * d) / norm3);
            jacobian.setElementAt(2, 1, -b / norm3 * c);
            jacobian.setElementAt(3, 1, -b / norm3 * d);
            
            jacobian.setElementAt(0, 2, -a / norm3 * c);
            jacobian.setElementAt(1, 2, -b / norm3 * c);
            jacobian.setElementAt(2, 2, (a * a + b * b + d * d) / norm3);
            jacobian.setElementAt(3, 2, -c / norm3 * d);
            
            jacobian.setElementAt(0, 3, -a / norm3 * d);
            jacobian.setElementAt(1, 3, -b / norm3 * d);
            jacobian.setElementAt(2, 3, -c / norm3 * d);
            jacobian.setElementAt(3, 3, (a * a + b * b + c * c) / norm3);
        }        
    }

    /**
     * Computes a linear interpolation between this quaternion and provided quaternion using
     * provided value as the interpolation ratio.
     *
     * @param q quaternion to interpolate.
     * @param t interpolation ratio. Must be a value between 0.0 and 1.0, both
     *          included. The closer the value is to 0.0, the more similar result will
     *          be to this instance. Conversely, the closer the value is to 1.0, the
     *          more similar result will be to q.
     * @return a new interpolated quaternion instance.
     * @throws IllegalArgumentException if provided interpolation ratio is not between 0.0
     * and 1.0, both included.
     */
    public Quaternion slerpAndReturnNew(Quaternion q, double t) {
        Quaternion result = new Quaternion();
        slerp(q, t, result);
        return result;
    }

    /**
     * Computes a linear interpolation between this quaternion and provided quaternion using
     * provided value as the interpolation ratio and stores the result into provided result
     * quaternion.
     *
     * @param q         quaternion to interpolate.
     * @param t         interpolation ratio. Must be a value between 0.0 and 1.0, both
     *                  included. The closer the value is to 0.0, the more similar result will
     *                  be to this instance. Conversely, the closer the value is to 1.0, the
     *                  more similar result will be to q.
     * @param result    instance where interpolated quaternion will be stored.
     * @throws IllegalArgumentException if provided interpolation ratio is not between 0.0
     * and 1.0, both included.
     */
    public void slerp(Quaternion q, double t, Quaternion result) {
        slerp(this, q, t, result);
    }

    /**
     * Computes a linear interpolation between provided quaternions using provided value
     * as the interpolation ratio.
     *
     * @param q1    1st quaternion to interpolate.
     * @param q2    2nd quaternion to interpolate.
     * @param t     interpolation ratio. Must be a value between 0.0 and 1.0, both
     *              included. The closer the value is to 0.0, the more similar result will
     *              be to q1. Conversely, the closer the value is to 1.0, the more similar
     *              result will be to q2.
     * @return      a new interpolated quaternion instance.
     * @throws IllegalArgumentException if provided interpolation ratio is not between 0.0
     * and 1.0, both included.
     */
    public static Quaternion slerpAndReturnNew(Quaternion q1, Quaternion q2, double t) {
        Quaternion result = new Quaternion();
        slerp(q1, q2, t, result);
        return result;
    }

    /**
     * Computes a linear interpolation between provided quaternions using provided value
     * as the interpolation ratio, and stores the result into provided result quaternion.
     *
     * @param q1    1st quaternion to interpolate.
     * @param q2    2nd quaternion to interpolate.
     * @param t     interpolation ratio. Must be a value between 0.0 and 1.0, both
     *              included. The closer the value is to 0.0, the more similar result will
     *              be to q1. Conversely, the closer the value is to 1.0, the more similar
     *              result will be to q2.
     * @param result instance where interpolated quaternion will be stored.
     * @throws IllegalArgumentException if provided interpolation ratio is not between 0.0
     * and 1.0, both included.
     */
    public static void slerp(Quaternion q1, Quaternion q2, double t, Quaternion result) {
        if (t < 0.0 || t > 1.0) {
            throw new IllegalArgumentException();
        }

        // only unit quaternions are valid rotations.
        // normalize to avoid undefined behavior.
        q1.normalize();
        q2.normalize();

        // calculate angle between input quaternions
        double dot = q1.mA * q2.mA +
                q1.mB * q2.mB +
                q1.mC * q2.mC +
                q1.mD * q2.mD;

        // if q1 = q2 or q1 = -q2 (both quaternions are equal), then the angle
        // between input quaternions is theta0 = 0.
        // To avoid singularity caused by sinTheta0, we return q1
        if (Math.abs(dot) >= 1.0) {
            result.mA = q1.mA;
            result.mB = q1.mB;
            result.mC = q1.mC;
            result.mD = q1.mD;
            return;
        }

        // if the dot product is negative, slerp won't take the shorter path.
        // note that q2 and -q2 are equivalent when the negation is applied to all four
        // components. Fix by reversing one quaternion.
        Quaternion q2b;
        if (dot < 0.0) {
            q2b = new Quaternion(-q2.mA, -q2.mB, -q2.mC, -q2.mD);
            dot = -dot;
        } else {
            q2b = q2;
        }

        double theta0 = Math.acos(dot);
        double theta = theta0 * t;
        double sinTheta = Math.sin(theta);
        double sinTheta0 = Math.sin(theta0);

        double s2 = sinTheta / sinTheta0;
        // line below is equal to sin(theta0 - theta) / sinTheta0
        double s1 = Math.cos(theta) - dot * s2;

        result.mA = s1 * q1.mA + s2 * q2b.mA;
        result.mB = s1 * q1.mB + s2 * q2b.mB;
        result.mC = s1 * q1.mC + s2 * q2b.mC;
        result.mD = s1 * q1.mD + s2 * q2b.mD;
    }
    
    /**
     * Normalizes this quaternion if not already normalized.
     *
     * @param norm norm to normalize this quaternion with.
     */
    private void internalNormalize(double norm) {
        if (!mNormalized) {
            mA /= norm;
            mB /= norm;
            mC /= norm;
            mD /= norm;
            mNormalized = true;
        }        
    }
}
