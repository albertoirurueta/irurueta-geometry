/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.AxisRotation3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 2, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import java.io.Serializable;

/**
 * This class defines the amount of rotation for 3D points or planes.
 * Rotation is defined internally as axis coordinates and rotation angle, 
 * following Rodrigues formulas.
 * This class is based in:
 * http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/sfrotation_java.htm
 */
public class AxisRotation3D extends Rotation3D implements Serializable{
    
    /**
     * Number of parameters defining a rotation axis.
     */
    public static final int AXIS_PARAMS = 3;
    
    /**
     * x element of axis angle.
     */
    private double mAxisX = 0.0;
    
    /**
     * y element of axis angle.
     */
    private double mAxisY = 0.0;
    
    /** 
     * z element of axis angle.
     */
    public double mAxisZ = 1.0;
   
    /** 
     * angle element of axis angle.
     */
    public double mTheta = 0.0;
            
    /**
     * Constant defining threshold to determine whether a matrix is orthogonal
     * or not and has determinant equal to 1. Rotation matrices must fulfill
     * those requirements.
     */
    public static final double MATRIX_VALID_THRESHOLD = 1e-12;
    
    /**
     * Constant defining machine precision.
     */
    public static final double EPS = 1e-12;       
      
    /**
     * Constructor which allows initial value to be supplied as axis and angle.
     * For better accuracy, axis values should be normalized.
     * @param axisX x dimension of normalized axis.
     * @param axisY y dimension of normalized axis.
     * @param axisZ z dimension of normalized axis.
     * @param theta angle in radians.
     */
    public AxisRotation3D(double axisX, double axisY, double axisZ, 
           double theta){
        setAxisAndRotation(axisX, axisY, axisZ, theta);
    }
   
    /**
     * Constructor where array of axis values and rotation angle are provided
     * For better accuracy, axis values should be normalized.
     * @param axis Array containing x,y and z values of axis.
     * @param theta rotation angle in radians.
     * @throws IllegalArgumentException if provided axis length is not 3.
     */
    public AxisRotation3D(double[] axis, double theta) 
           throws IllegalArgumentException{
        setAxisAndRotation(axis, theta);
    }
       
    /** constructor to create sfrotation from euler angles.
     * @param heading rotation about z axis.
     * @param attitude rotation about y axis.
     * @param bank rotation about x axis.
     */
    public AxisRotation3D(double heading, double attitude, 
           double bank){
       setAngles(heading, attitude, bank);
    }
   
    /** 
     * Copy constructor.
     * @param rotation instance to copy.
     */
    public AxisRotation3D(AxisRotation3D rotation) {
           fromRotation(rotation);
    }
    
    /**
     * Copy constructor.
     * @param rot Converts and copies provided rotation instance.
     */
    public AxisRotation3D(Rotation3D rot){
        try{
            fromMatrix(rot.asInhomogeneousMatrix());
        }catch(InvalidRotationMatrixException ignore){}
    }
   
    /** 
     * Empty constructor.
     */
    public AxisRotation3D() {}
    
    /**
     * Returns type of this rotation.
     * @return Type of this rotation.
     */
    @Override
    public Rotation3DType getType(){
        return Rotation3DType.AXIS_ROTATION3D;
    }
   
    /**
     * Sets rotation angles as heading, attitude and bank, which are commonly
     * used in areas such as aviation.
     * @param heading Heading angle expressed in radians.
     * @param attitude Attitude angle expressed in radians.
     * @param bank Bank angle expressed in radians.
     */
    public final void setAngles(double heading, double attitude, double bank){
        double c1 = Math.cos(heading/2);
        double s1 = Math.sin(heading/2);
        double c2 = Math.cos(attitude/2);
        double s2 = Math.sin(attitude/2);
        double c3 = Math.cos(bank/2);
        double s3 = Math.sin(bank/2);
        double c1c2 = c1 * c2;
        double s1s2 = s1 * s2;
        mTheta = c1c2 * c3 + s1s2 * s3;
        mAxisX = c1c2 * s3 - s1s2 * c3;
        mAxisY = c1 * s2 * c3 + s1 * c2 * s3;
        mAxisZ = s1 * c2 * c3 - c1 * s2 * s3;
    }
   
    /**
     * Sets rotation of this instance by copying provided rotation.
     * @param rot Rotation to be copied.
     */
    @Override
    public final void fromRotation(AxisRotation3D rot){
        mAxisX = (rot!=null) ? rot.mAxisX : 0;
        mAxisY = (rot!=null) ? rot.mAxisY : 0;
        mAxisZ = (rot!=null) ? rot.mAxisZ : 1;
        mTheta = (rot!=null) ? rot.mTheta : 0;
    }

    /**
     * Sets rotation axis of this instance while preserving the rotation angle.
     * Once set, points will rotate around provided axis.
     * @param axisX X coordinate of rotation axis.
     * @param axisY Y coordinate of rotation axis.
     * @param axisZ Z coordinate of rotation axis.
     */
    public void setAxis(double axisX, double axisY, double axisZ) {
        mTheta = Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);
        if(mTheta == 0.0){
            mAxisX = 1; 
            mAxisY = mAxisZ = 0.0; 
            return;
        }
        mAxisX = axisX / mTheta;
        mAxisY = axisY / mTheta;
        mAxisZ = axisZ / mTheta;
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
            double axisZ, double theta){
        mAxisX = axisX;
        mAxisY = axisY;
        mAxisZ = axisZ;
        mTheta = theta;
    }
           
    /** 
     * Returns X coordinate of rotation axis.
     * @return X coordinate of rotation axis.
     */
    public double getAxisX() {
        return mAxisX;
    }

    /**
     * Returns Y coordinate of rotation axis.
     * @return Y coordinate of rotation axis.
     */
    public double getAxisY() {
        return mAxisY;
    }

    /**
     * Returns Z coordinate of rotation axis.
     * @return Z coordinate of rotation axis.
     */
    public double getAxisZ() {
        return mAxisZ;
    }
       
    /**
     * Returns rotation axis corresponding to this instance.
     * Result is stored in provided axis array, which must have length 3
     * @param axis Array where axis coordinates will be stored.
     * @throws IllegalArgumentException Raised if provided array does not have
     * length 3.
     */    
    @Override
    public void rotationAxis(double[] axis) throws IllegalArgumentException{
        if(axis.length != INHOM_COORDS) throw new IllegalArgumentException();
        
        axis[0] = mAxisX;
        axis[1] = mAxisY;
        axis[2] = mAxisZ;        
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
    public double getRotationAngle() throws RotationException{
        return mTheta;
    }
   
    /**
     * Returns a 3D rotation which is inverse to this instance.
     * In other words, the combination of this rotation with its inverse 
     * produces no change.
     * @return Inverse 3D rotation.
     */    
    @Override
    public AxisRotation3D inverseRotationAndReturnNew(){
        AxisRotation3D rot = new AxisRotation3D();
        inverseRotation(rot);
        return rot;
    }
    
    /**
     * Sets into provided MatrixRotation3D instance a rotation inverse to this 
     * instance.
     * The combination of this rotation with its inverse produces no change.
     * @param result Instance where inverse rotation will be set.
     */
    public void inverseRotation(AxisRotation3D result){
        //copy this rotation into result
        result.fromRotation(this);
        //reverse angle
        result.mTheta = -mTheta;
    }
    
    /**
     * Sets into provided MatrixRotation3D instance a rotation inverse to this 
     * instance.
     * The combination of this rotation with its inverse produces no change.
     * @param result Instance where inverse rotation will be set.
     */    
    @Override
    public void inverseRotation(Rotation3D result){
        if(result instanceof AxisRotation3D){
            inverseRotation((AxisRotation3D)result);
            
        }else if(result instanceof MatrixRotation3D){
            AxisRotation3D rot = new AxisRotation3D();
            inverseRotation(rot);
            try{
                result.fromMatrix(rot.asInhomogeneousMatrix());
            }catch(InvalidRotationMatrixException ignore){}
        }
    }
    
    /**
     * Reverses the rotation of this instance.
     */    
    @Override
    public void inverseRotation(){
        inverseRotation(this);
    }
        
    /**
     * Returns this 3D rotation instance expressed as a 3x3 inhomogeneous 
     * matrix.
     * This is equivalent to call getInternalMatrix().
     * @return Rotation matrix expressed in inhomogeneous coordinates.
     */    
    @Override
    public Matrix asInhomogeneousMatrix(){
        Matrix result = null;
        try{
            result = new Matrix(INHOM_COORDS, INHOM_COORDS);
        }catch(WrongSizeException ignore){}
        asInhomogeneousMatrix(result);
        return result;
    }
    
    /**
     * Sets into provided Matrix instance this 3D rotation expressed as a
     * 3x3 inhomogeneous matrix.
     * @param result Matrix where rotation will be set.
     * @throws IllegalArgumentException Raised if provided instance does not
     * have size 3x3.
     */    
    @Override
    public void asInhomogeneousMatrix(Matrix result) 
            throws IllegalArgumentException{
        if(result.getRows() != INHOM_COORDS ||
                result.getColumns() != INHOM_COORDS)
            throw new IllegalArgumentException();
        
        double c = Math.cos(mTheta);
        double s = Math.sin(mTheta);
        double t = 1.0 - c;
	//normalise axis
	double magnitude = Math.sqrt(mAxisX * mAxisX + mAxisY * mAxisY + 
                mAxisZ * mAxisZ);
	if (magnitude > EPS){
            //normalize only if axis norm is large enough to avoid numerical
            //instability
            mAxisX /= magnitude;
            mAxisY /= magnitude;
            mAxisZ /= magnitude;
        }

        result.setElementAt(0, 0, c + mAxisX * mAxisX * t);
        result.setElementAt(1, 1, c + mAxisY * mAxisY * t);
        result.setElementAt(2, 2, c + mAxisZ * mAxisZ * t);


        double tmp1 = mAxisX * mAxisY * t;
        double tmp2 = mAxisZ * s;
        result.setElementAt(1, 0, tmp1 + tmp2);
        result.setElementAt(0, 1, tmp1 - tmp2);
        tmp1 = mAxisX * mAxisZ * t;
        tmp2 = mAxisY * s;
        result.setElementAt(2, 0, tmp1 - tmp2);
        result.setElementAt(0, 2, tmp1 + tmp2);
        tmp1 = mAxisY * mAxisZ * t;
        tmp2 = mAxisX * s;
        result.setElementAt(2, 1, tmp1 + tmp2);
        result.setElementAt(1, 2, tmp1 - tmp2);
    }
    
    /**
     * Returns this 3D rotation instance expressed as a 4x4 homogeneous matrix.
     * @return Rotation matrix expressed in homogeneous coordinates.
     */        
    @Override
    public Matrix asHomogeneousMatrix(){
        Matrix result = null;
        try{
            result = Matrix.identity(HOM_COORDS, HOM_COORDS);
            //sets into 3x3 top-left submatrix the internal matrix of this
            //instance, the remaining part will continue to be the identity
            result.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, 
                    asInhomogeneousMatrix());
        }catch(WrongSizeException ignore){}
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
    public void asHomogeneousMatrix(Matrix result) 
            throws IllegalArgumentException{
        if(result.getRows() != HOM_COORDS ||
                result.getColumns() != HOM_COORDS)
            throw new IllegalArgumentException();
        
        result.initialize(0.0);
        //sets into 3x3 top-left submatrix the internal matrix of this instance
        result.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, 
                asInhomogeneousMatrix());
        //set las element to 1.0 (to be like the identity
        result.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);        
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
     * @throws IllegalArgumentException Raised if provided threshold is negative
     * {@link #isValidRotationMatrix(Matrix)}
     */        
    @Override
    public void fromInhomogeneousMatrix(Matrix m, double threshold)
            throws InvalidRotationMatrixException, IllegalArgumentException{
        
        if(m.getRows() != INHOM_COORDS ||
                m.getColumns() != INHOM_COORDS)
            throw new InvalidRotationMatrixException();
        if(!MatrixRotation3D.isValidRotationMatrix(m, threshold)) 
            throw new InvalidRotationMatrixException();
                
        double angle, x, y, z; // variables for result
	double epsilon = 0.01; // margin to allow for rounding errors
	double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees

        if ((Math.abs(m.getElementAt(0, 1) - m.getElementAt(1, 0)) < epsilon)
	  && (Math.abs(m.getElementAt(0, 2) - m.getElementAt(2, 0)) < epsilon)
	  && (Math.abs(m.getElementAt(1, 2) - m.getElementAt(2, 1)) < epsilon)){
		// singularity found
		// first check for identity matrix which must have +1 for all terms
		//  in leading diagonaland zero in other terms
		if ((Math.abs(m.getElementAt(0, 1) + 
                        m.getElementAt(1, 0)) < epsilon2)
		  && (Math.abs(m.getElementAt(0, 2) + 
                        m.getElementAt(2, 0)) < epsilon2)
		  && (Math.abs(m.getElementAt(1, 2) + 
                        m.getElementAt(2, 1)) < epsilon2)
		  && (Math.abs(m.getElementAt(0, 0) + m.getElementAt(1, 1) + 
                        m.getElementAt(2, 2) - 3.0) < epsilon2)){
			// this singularity is identity matrix so angle = 0
			setAxisAndRotation(0.0,0.0,1.0,0.0); // zero angle, 
                                                            //arbitrary axis
                        return;
		}
		// otherwise this singularity is angle = 180
		angle = Math.PI;
		double xx = (m.getElementAt(0, 0) + 1.0) / 2.0;
		double yy = (m.getElementAt(1, 1) + 1.0) / 2.0;
		double zz = (m.getElementAt(2, 2) + 1.0) / 2.0;
		double xy = (m.getElementAt(0, 1) + m.getElementAt(1, 0)) / 4.0;
		double xz = (m.getElementAt(0, 2) + m.getElementAt(2, 0)) / 4.0;
		double yz = (m.getElementAt(1, 2) + m.getElementAt(2, 1)) / 4.0;
		if ((xx > yy) && (xx > zz)) { // m.getElementAt(0, 0) is the 
                                                //largest diagonal term
			if (xx< epsilon) {
				x = 0.0;
				y = Math.sqrt(2.0) / 2.0;
				z = Math.sqrt(2.0) / 2.0;
			} else {
				x = Math.sqrt(xx);
				y = xy / x;
				z = xz / x;
			}
		} else if (yy > zz) { // m.getElementAt(1, 1) is the largest 
                                        //diagonal term
			if (yy< epsilon) {
				x = Math.sqrt(2.0) / 2.0;
				y = 0.0;
				z = Math.sqrt(2.0) / 2.0;
			} else {
				y = Math.sqrt(yy);
				x = xy / y;
				z = yz / y;
			}	
		} else { // m.getElementAt(2, 2) is the largest diagonal term so
                        //base result on this
			if (zz< epsilon) {
				x = Math.sqrt(2.0) / 2.0;
				y = Math.sqrt(2.0) / 2.0;
				z = 0.0;
			} else {
				z = Math.sqrt(zz);
				x = xz / z;
				y = yz / z;
			}
		}
                setAxisAndRotation(x, y, z, angle);
                return; // return 180 deg rotation
	}
        
	// as we have reached here there are no singularities so we can handle 
        //normally
	double s = Math.sqrt((m.getElementAt(2, 1) - m.getElementAt(1, 2)) * 
                (m.getElementAt(2, 1) - m.getElementAt(1, 2))
		+(m.getElementAt(0, 2) - m.getElementAt(2, 0)) * 
                (m.getElementAt(0, 2) - m.getElementAt(2, 0))
		+(m.getElementAt(1, 0) - m.getElementAt(0, 1))*
                (m.getElementAt(1, 0) - m.getElementAt(0, 1))); // used to 
                                                                //normalise
	if (Math.abs(s) < 0.001) s = 1.0; 
	// prevent divide by zero, should not happen if matrix is orthogonal and
        //should be caught by singularity test above, but I've left it in just 
        //in case
	mTheta = Math.acos((m.getElementAt(0, 0) + m.getElementAt(1, 1) + 
                m.getElementAt(2, 2) - 1.0) / 2.0);
	mAxisX = (m.getElementAt(2, 1) - m.getElementAt(1, 2)) / s;
	mAxisY = (m.getElementAt(0, 2) - m.getElementAt(2, 0)) / s;
	mAxisZ = (m.getElementAt(1, 0) - m.getElementAt(0, 1)) / s;                
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
     * {@link #isValidRotationMatrix(Matrix)}
     */          
    @Override
    public void fromHomogeneousMatrix(Matrix m, double threshold)
            throws InvalidRotationMatrixException{
        if(m.getRows() != HOM_COORDS ||
                m.getColumns() != HOM_COORDS)
            throw new InvalidRotationMatrixException();
        if(!MatrixRotation3D.isValidRotationMatrix(m, threshold)) 
            throw new InvalidRotationMatrixException();        
        if(Math.abs(m.getElementAt(3, 0)) > threshold ||
                Math.abs(m.getElementAt(3, 1)) > threshold ||
                Math.abs(m.getElementAt(3, 2)) > threshold ||
                Math.abs(m.getElementAt(0, 3)) > threshold ||
                Math.abs(m.getElementAt(1, 3)) > threshold ||
                Math.abs(m.getElementAt(2, 3)) > threshold ||
                Math.abs(m.getElementAt(3, 3) - 1.0) > threshold)
            throw new InvalidRotationMatrixException();
        
        fromInhomogeneousMatrix(m.getSubmatrix(0, 0, INHOM_COORDS - 1, 
                INHOM_COORDS - 1), threshold);
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
    public void rotate(Point3D inputPoint, Point3D resultPoint){
        double s = Math.sin(mTheta / 2.0);
        double xh = mAxisX * s;
        double yh = mAxisY * s;
        double zh = mAxisZ * s;
        double wh = Math.cos(mTheta / 2.0);
        
        double inhomX = inputPoint.getInhomX();
        double inhomY = inputPoint.getInhomY();
        double inhomZ = inputPoint.getInhomZ();
        
        double resultX = wh * wh * inhomX + 2.0 * yh * wh * inhomZ - 
                2.0 * zh * wh * inhomY + xh * xh * inhomX + 
                2.0 * yh * xh * inhomY + 2 * zh * xh * inhomZ - 
                zh * zh * inhomX - yh * yh * inhomX;
        double resultY = 2.0 * xh * yh * inhomX + yh * yh * inhomY + 
                2.0 * zh * yh * inhomZ + 2.0 * wh * zh * inhomX - 
                zh * zh * inhomY + wh * wh * inhomY - 2 * xh * wh * inhomZ - 
                xh * xh * inhomY;
        double resultZ = 2.0 * xh * zh * inhomX + 2.0 * yh * zh * inhomY + 
                zh * zh * inhomZ - 2.0 * wh * yh * inhomX - 
                yh * yh * inhomZ + 2.0 * wh * xh * inhomY - xh * xh * inhomZ + 
                wh * wh * inhomZ;
        resultPoint.setInhomogeneousCoordinates(resultX, resultY, resultZ);        
    }
            
    /**
     * Combines provided rotation with this rotation and returns the result as
     * a new MatrixRotation3D instance.
     * @param rotation Input rotation to be combined.
     * @return Combined rotation, which is equal to the multiplication of the
     * internal matrix of provided rotation with the internal matrix of this 
     * instance.
     */        
    public AxisRotation3D combineAndReturnNew(
            AxisRotation3D rotation){
        AxisRotation3D result = new AxisRotation3D();
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
    public Rotation3D combineAndReturnNew(Rotation3D rotation){
        if(rotation instanceof AxisRotation3D){
            return combineAndReturnNew((AxisRotation3D)rotation);
        }else{
            return combineAndReturnNew(new AxisRotation3D(rotation));
        }
    }    
    
    /**
     * Combines provided rotation into this rotation resulting in the 
     * multiplication of the internal matrices of both rotations.
     * @param rotation  Input rotation to be combined.
     */    
    public void combine(AxisRotation3D rotation){
        combine(this, rotation, this);
    }
    
    /**
     * Combines provided rotation into this rotation resulting in the 
     * multiplication of the internal matrices of both rotations.
     * @param rotation  Input rotation to be combined.
     */        
    @Override
    public void combine(Rotation3D rotation){
        if(rotation instanceof AxisRotation3D){
            combine((AxisRotation3D)rotation);
        }else{
            combine(new AxisRotation3D(rotation));
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
    public static void combine(AxisRotation3D rot1, 
            AxisRotation3D rot2, AxisRotation3D result){
        
        Matrix m1 = rot1.asInhomogeneousMatrix();
        Matrix m2 = rot2.asInhomogeneousMatrix();
        try{
            m1.multiply(m2);
            result.fromMatrix(m1);
        }catch(InvalidRotationMatrixException ignore){            
        }catch(WrongSizeException ignore){}
    }       
    
    /**
     * Sets values of this rotation from a 3D matrix rotation.
     * @param rot 3D matrix rotation to set values from.
     */
    @Override
    public void fromRotation(MatrixRotation3D rot){
        try{
            fromInhomogeneousMatrix(rot.internalMatrix);
        }catch(InvalidRotationMatrixException ignore){ /* never thrown */ }
    }

    /**
     * Sets values of this rotation from a quaternion.
     * @param q a quaternion to set values from.
     */
    @Override
    public void fromRotation(Quaternion q){
        q.toAxisRotation(this);
    }    
    
    /**
     * Converts this 3D rotation into a matrix rotation storing the result
     * into provided instance.
     * @param result instance where result wil be stored.
     */
    @Override
    public void toMatrixRotation(MatrixRotation3D result){
        result.setAxisAndRotation(new double[]{ mAxisX, mAxisY, mAxisZ}, 
                mTheta);
    }    
    
    /**
     * Converts this 3D rotation into an axis rotation storing the result into
     * provided instance.
     * @param result instance where result will be stored.
     */
    @Override
    public void toAxisRotation(AxisRotation3D result){
        result.fromRotation(this);
    }  
    
    /**
     * Converts this 3D rotation into a quaterion storing the result into 
     * provided instance.
     * @param result instance where result will be stored.
     */
    @Override
    public void toQuaternion(Quaternion result){
        result.setFromAxisAndRotation(mAxisX, mAxisY, mAxisZ, mTheta);
    }    
}