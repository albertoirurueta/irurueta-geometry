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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;

/**
 * This class defines intrinsic parameters of a pinhole camera.
 * Extrinsic parameters on a pinhole camera are those external to the camera 
 * such as rotation and translation, whereas intrinsic parameters are more 
 * related to the inner workings of a camera.
 * Intrinsic parameters are those such as horizontal/vertical focal length,
 * skewness of axes or principal point of an image (which is usually related
 * to lens/sensor slanting.
 */
@SuppressWarnings("WeakerAccess")
public class PinholeCameraIntrinsicParameters implements Serializable{

    /**
     * Constant defining the required number of rows of an intrinsic parameters
     * matrix.
     */
    public static final int INTRINSIC_MATRIX_ROWS = 3;

    /**
     * Constant defining the required number of columns of an intrinsic
     * parameters matrix.
     */
    public static final int INTRINSIC_MATRIX_COLS = 3;

    /**
     * Threshold to determine whether a given intrinsic parameters matrix is
     * valid (is upper triangular).
     */
    public static final double DEFAULT_VALID_THRESHOLD = 1e-12;

    /**
     * Internal matrix defining the intrinsic parameters of a camera.
     */
    private Matrix mInternalMatrix;    

    /**
     * Constructor.
     * Creates canonical intrinsic parameters which has no effect on projected
     * 3D points into 2D points.
     */
    public PinholeCameraIntrinsicParameters() {
        try {
            mInternalMatrix = Matrix.identity(INTRINSIC_MATRIX_ROWS, 
                    INTRINSIC_MATRIX_COLS);
        } catch (WrongSizeException ignore) {
            //never happens
        }
    }
    
    /**
     * Creates a copy of provided intrinsic parameters.
     * @param params Intrinsic parameters to be copied.
     */
    public PinholeCameraIntrinsicParameters(
            PinholeCameraIntrinsicParameters params) {
        mInternalMatrix = params.mInternalMatrix.clone();
    }
    
    /**
     * Creates a new instance of camera intrinsic parameters using provided
     * matrix.
     * Provided matrix must be 3x3 and upper triangular.
     * Note: this constructor will attempt to normalize provided matrix, hence 
     * its values might change after calling this constructor.
     * @param internalMatrix Provided 3x3 and upper triangular matrix
     * @throws InvalidPinholeCameraIntrinsicParametersException thrown if
     * provided matrix is not 3x3 or upper triangular.
     */
    public PinholeCameraIntrinsicParameters(Matrix internalMatrix)
            throws InvalidPinholeCameraIntrinsicParametersException {
        setInternalMatrix(internalMatrix);
    }
    
    /**
     * Creates a new instance of camera intrinsic parameters using provided
     * matrix and provided threshold to determine whether it is a valid matrix.
     * Provided matrix must be 3x3 and upper triangular up to provided threshold
     * Note: this constructor will attempt to normalize provided matrix, hence 
     * its values might change after calling this constructor.
     * @param internalMatrix Provided 3x3 and upper triangular matrix.
     * @param threshold Threshold to determine whether provided matrix is upper
     * triangular. Matrix will be considered upper triangular if its lower 
     * triangular elements are larger than this threshold in absolute terms.
     * @throws InvalidPinholeCameraIntrinsicParametersException thrown if 
     * provided matrix is not 3x3 or upper triangular.
     * @throws IllegalArgumentException thrown if provided threshold is negative.
     */
    public PinholeCameraIntrinsicParameters(Matrix internalMatrix, 
            double threshold) 
            throws InvalidPinholeCameraIntrinsicParametersException {
        setInternalMatrix(internalMatrix, threshold);
    }
        
    /**
     * Creates a new instance of camera intrinsic parameters using provided
     * horizontal/vertical focal length, horizontal/vertical principal point and
     * skewness of axes.
     * @param horizontalFocalLength Horizontal focal length of camera. 
     * The larger the focal length the larger objects will appear, as focal 
     * length determines the amount of "zoom". The relation between horizontal
     * and vertical focal length determines the aspect ratio of images.
     * @param verticalFocalLength Vertical focal length of camera.
     * The larger the focal length the larger objects will appear, as focal
     * length determines the amount of "zoom". The relation between horizontal
     * and vertical focal length determines the aspect ratio of images.
     * @param horizontalPrincipalPoint Horizontal principal point of camera.
     * Determines where the origin of coordinates of 2D points will be located
     * horizontally on the retinal plane. This is usually the center of an 
     * image. If not specified, then the origin of coordinates on the retinal
     * plane is located at (0, 0).
     * @param verticalPrincipalPoint Vertical principal point of camera.
     * Determines where the origin of coordinates of 2D points will be located
     * vertically on the retinal plane. This is usually the center of an image.
     * If not specified, then the origin of coordinates on the retinal plane
     * is located at (0, 0).
     * @param skewness Skewness of axes. This usually zero or a value close to
     * zero. The larger the value in absolute terms the more skewed (i.e. 
     * slanted) x-y axes will be on projected images.
     */
    public PinholeCameraIntrinsicParameters(double horizontalFocalLength,
            double verticalFocalLength, double horizontalPrincipalPoint,
            double verticalPrincipalPoint, double skewness) {
        
        //create instance initialized to the identity
        try {
            mInternalMatrix = Matrix.identity(INTRINSIC_MATRIX_ROWS, 
                    INTRINSIC_MATRIX_COLS);                        
        } catch (WrongSizeException ignore) {
            //never happens
        }
        
        //set parameters
        setHorizontalFocalLength(horizontalFocalLength);
        setVerticalFocalLength(verticalFocalLength);
        setHorizontalPrincipalPoint(horizontalPrincipalPoint);
        setVerticalPrincipalPoint(verticalPrincipalPoint);
        setSkewness(skewness);
    }
    
    /**
     * Returns a copy of the internal matrix defining this instance parameters
     * @return A copy of the internal matrix of this instance.
     */
    public Matrix getInternalMatrix() {
        return mInternalMatrix.clone();
    }
    
    /**
     * Sets internal matrix of this instance.
     * Note: this method will attempt to normalize provided matrix, hence its 
     * values might change after calling this method.
     * @param internalMatrix Matrix to be set as the internal matrix of this
     * instance. This matrix needs to be 3x3 and upper triangular.
     * @throws InvalidPinholeCameraIntrinsicParametersException thrown if
     * provided matrix is not 3x3 or upper triangular.
     */
    public final void setInternalMatrix(Matrix internalMatrix)
            throws InvalidPinholeCameraIntrinsicParametersException {
        setInternalMatrix(internalMatrix, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Sets the internal matrix of this instance using provided threshold to
     * determine whether it is upper triangular.
     * Note: this method will attempt to normalize provided matrix, hence its 
     * values might change after calling this method.
     * @param internalMatrix Matrix to be set as the internal matrix of this
     * instance. This matrix needs to be 3x3 and upper triangular.
     * @param threshold Threshold to determine whether provided matrix is upper
     * triangular. Matrix will be considered upper triangular if its lower 
     * triangular elements are larger than this threshold in absolute terms.
     * @throws InvalidPinholeCameraIntrinsicParametersException thrown if
     * provided matrix is not 3x3 or upper triangular.
     * @throws IllegalArgumentException thrown if provided threshold is negative.
     */
    public final void setInternalMatrix(Matrix internalMatrix,
            double threshold)
            throws InvalidPinholeCameraIntrinsicParametersException {
        
        //normalizes provided matrix to ensure that lelemen 3,3 is 1
        normalize(internalMatrix);
        
        //check that provided matrix is upper-triangular and that lower right
        //element is 1.0 (using provided threshold as a measure of error
        //tolerance)
        boolean valid = isValidMatrix(internalMatrix, threshold);
        if (!valid) {
            throw new InvalidPinholeCameraIntrinsicParametersException();
        }
        
        mInternalMatrix = internalMatrix;
    }
    
    /**
     * Computes the inverse of internal matrix and returns the result.
     * Calling this method is more efficient than calling.
     * com.irurueta.algebra.Utils.inverse.
     * @return inverse of internal matrix.
     */
    public Matrix getInverseInternalMatrix() {
        Matrix result = null;
        try {
            result = new Matrix(INTRINSIC_MATRIX_ROWS, 
                    INTRINSIC_MATRIX_COLS);
            getInverseInternalMatrix(result);            
        } catch (WrongSizeException ignore) { /* never happens */ }
        
        return result;
    }
    
    /**
     * Computes the inverse of internal matrix and stores the result into 
     * provided matrix.
     * Calling this method is more efficient than calling
     * com.irurueta.algebra.Utils.inverse.
     * @param result instance where result will be stored.
     */
    public void getInverseInternalMatrix(Matrix result) {
        double horizontalFocalLength = getHorizontalFocalLength();
        double verticalFocalLength = getVerticalFocalLength();
        double skewness = getSkewness();
        double horizontalPrincipalPoint = getHorizontalPrincipalPoint();
        double verticalPrincipalPoint = getVerticalPrincipalPoint();
        
        result.setElementAt(0, 0, 1.0 / horizontalFocalLength);
        result.setElementAt(0, 1, -skewness / 
                (horizontalFocalLength * verticalFocalLength));
        result.setElementAt(1, 1, 1.0 / verticalFocalLength);
        result.setElementAt(0, 2, (skewness * verticalPrincipalPoint - 
                verticalFocalLength * horizontalPrincipalPoint) /
                (horizontalFocalLength * verticalFocalLength));
        result.setElementAt(1, 2, -verticalPrincipalPoint/ verticalFocalLength);
        result.setElementAt(2, 2, 1.0);        
    }
    
    /**
     * Returns the horizontal focal length of a camera. 
     * The larger the focal length the larger objects will appear, as focal 
     * length determines the amount of "zoom". The relation between horizontal
     * and vertical focal length determines the aspect ratio of images.
     * Note: Negative values will reverse projected points or geometric objects 
     * horizontally.
     * @return Horizontal focal length.
     */
    public double getHorizontalFocalLength() {
        return mInternalMatrix.getElementAt(0, 0);
    }
    
    /**
     * Sets the horizontal focal length of a camera.
     * The larger the focal length the larger objects will appear, as focal 
     * length determines the amount of "zoom". The relation between horizontal
     * and vertical focal length determines the aspect ratio of images
     * Note: Negative values will reverse projected points or geometric objects 
     * horizontally.
     * @param horizontalFocalLength Horizontal focal length to be set.
     */
    public final void setHorizontalFocalLength(double horizontalFocalLength) {
        mInternalMatrix.setElementAt(0, 0, horizontalFocalLength);
    }
    
    /**
     * Returns the vertical focal length of a camera.
     * The larger the focal length the larger objects will appear, as focal 
     * length determines the amount of "zoom". The relation between horizontal
     * and vertical focal length determines the aspect ratio of images
     * Note: Negative values will reverse projected points or geometric objects 
     * vertically.
     * @return Vertical focal length.
     */
    public double getVerticalFocalLength() {
        return mInternalMatrix.getElementAt(1, 1);
    }
    
    /**
     * Sets the vertical focal length of a camera.
     * The larger the focal length the larger objects will appear, as focal 
     * length determines the amount of "zoom". The relation between horizontal
     * and vertical focal length determines the aspect ratio of images
     * Note: Negative values will reverse projected points or geometric objects 
     * vertically.
     * @param verticalFocalLength Vertical focal length to be set.
     */    
    public final void setVerticalFocalLength(double verticalFocalLength) {
        mInternalMatrix.setElementAt(1, 1, verticalFocalLength);
    }
    
    /**
     * Returns aspect ratio.
     * Aspect ratio is the relation between vertical and horizontal focal 
     * lengths.
     * If objects are meant to be displayed with no horizontal/vertical scaling
     * distortion, then aspect ratio must be set to 1.0, which means that both
     * horizontal and vertical focal lengths are equal.
     * @return Aspect ratio.
     */
    public double getAspectRatio() {
        return mInternalMatrix.getElementAt(1, 1) / 
                mInternalMatrix.getElementAt(0, 0);
    }
    
    /**
     * Sets provided aspect ratio but keeping current horizontal focal length
     * value.
     * If objects are meant to be displayed with no horizontal/vertical scaling
     * distortion, then aspect ratio must be set to 1.0, which means that both
     * horizontal and vertical focal lengths are equal.
     * @param aspectRatio Aspect ratio to be set.
     */
    public void setAspectRatioKeepingHorizontalFocalLength(double aspectRatio) {
        mInternalMatrix.setElementAt(1, 1, 
                aspectRatio * mInternalMatrix.getElementAt(0, 0));
    }
    
    /**
     * Sets provided aspect ratio but keeping current vertical focal length
     * value.
     * If objects are meant to be displayed with no horizontal/vertical scaling
     * distortion, then aspect ratio must be set to 1.0, which means that both
     * horizontal and vertical focal lengths are equal.
     * @param aspectRatio Aspect ratio to be set.
     */
    public void setAspectRatioKeepingVerticalFocalLength(double aspectRatio) {
        mInternalMatrix.setElementAt(0, 0, 
                mInternalMatrix.getElementAt(1, 1) / aspectRatio);
    }
    
    /**
     * Returns horizontal principal point.
     * Determines where the origin of coordinates of 2D points will be located
     * horizontally on the retinal plane. This is usually the center of an 
     * image. If not specified, then the origin of coordinates on the retinal
     * plane is located at (0, 0) by default.
     * Principal point is usually related to the slanting between the camera 
     * sensor and lens. If properly aligned then principal point is usually
     * located at image center.
     * @return Horizontal principal point.
     */
    public double getHorizontalPrincipalPoint() {
        return mInternalMatrix.getElementAt(0, 2);
    }
    
    /**
     * Sets horizontal principal point.
     * Determines where the origin of coordinates of 2D points will be located
     * horizontally on the retinal plane. This is usually the center of an 
     * image. If not specified, then the origin of coordinates on the retinal
     * plane is located at (0, 0) by default.
     * Principal point is usually related to the slanting between the camera 
     * sensor and lens. If properly aligned then principal point is usually
     * located at image center.
     * @param horizontalPrincipalPoint Horizontal principal point to be set.
     */
    public final void setHorizontalPrincipalPoint(
            double horizontalPrincipalPoint) {
        mInternalMatrix.setElementAt(0, 2, horizontalPrincipalPoint);
    }
    
    /**
     * Returns vertical principal point.
     * Determines where the origin of coordinates of 2D points will be located
     * vertically on the retinal plane. This is usually the center of an 
     * image. If not specified, then the origin of coordinates on the retinal
     * plane is located at (0, 0) by default.
     * Principal point is usually related to the slanting between the camera 
     * sensor and lens. If properly aligned then principal point is usually
     * located at image center.
     * @return Vertical principal point.
     */    
    public double getVerticalPrincipalPoint() {
        return mInternalMatrix.getElementAt(1, 2);
    }
    
    /**
     * Sets vertical principal point.
     * Determines where the origin of coordinates of 2D points will be located
     * vertically on the retinal plane. This is usually the center of an 
     * image. If not specified, then the origin of coordinates on the retinal
     * plane is located at (0, 0) by default.
     * Principal point is usually related to the slanting between the camera 
     * sensor and lens. If properly aligned then principal point is usually
     * located at image center.
     * @param verticalPrincipalPoint Vertical principal point to be set.
     */    
    public final void setVerticalPrincipalPoint(double verticalPrincipalPoint) {
        mInternalMatrix.setElementAt(1, 2, verticalPrincipalPoint);
    }
    
    /**
     * Returns skewness of axes on the retinal plane. This usually zero or a 
     * value close to zero. The larger the value in absolute terms the more 
     * skewed (i.e. slanted) x-y axes will be on projected images.
     * @return Skewness of axes.
     */
    public double getSkewness() {
        return mInternalMatrix.getElementAt(0, 1);
    }
    
    /**
     * Sets skewness of axes on the retinal plane. This is usually zero or a
     * value close to zero. The larger the value in absolute terms the more
     * skewed (i.e. slanted) x-y axes will be on projected images.
     * @param skewness Skewness of axes to be set.
     */
    public final void setSkewness(double skewness) {
        mInternalMatrix.setElementAt(0, 1, skewness);
    }
    
    /**
     * Returns skewness angle in radians of retinal x-y axes associated to 
     * current skewness value.
     * This is usually zero or a value close to zero. The larger the value in 
     * absolute terms the more skewed (i.e. slanted) x-y axes will be on 
     * projected images.
     * @return Skewness angle in radians.
     */
    public double getSkewnessAngle() {
        return Math.atan2(mInternalMatrix.getElementAt(0, 1), 
                mInternalMatrix.getElementAt(1, 1));
    }
    
    /**
     * Sets skewness angle in radians of retinal x-y axes associated to current
     * skewness value.
     * This is usually zero or a value close to zero. The larger the value in
     * absolute terms the more skewed (i.e. slanted) x-y axes will be on
     * projected images.
     * @param skewnessAngle Skewness angle in radians to be set.
     */
    public void setSkewnessAngle(double skewnessAngle) {
        mInternalMatrix.setElementAt(0, 1, mInternalMatrix.getElementAt(1, 1) * 
                Math.tan(skewnessAngle));
    }
        
    /**
     * Creates a canonical intrinsic parameters instance which has no effect on 
     * projected 3D points into 2D points.
     * @return Canonical intrinsic parameters instance.
     */
    public static PinholeCameraIntrinsicParameters 
            createCanonicalIntrinsicParameters() {
        return new PinholeCameraIntrinsicParameters();
    }

    /**
     * Creates typical intrinsic parameters for an image of provided size.
     * Typical intrinsic parameters have principal point located at image center
     * and a focal length which results in a frustrum of approximately 45º.
     * @param imageWidth Width of image in pixels.
     * @param imageHeight Height of image in pixels.
     * @return Typical intrinsic parameters.
     * @throws IllegalArgumentException raised if provided width or height is
     * negative.
     */
    public static PinholeCameraIntrinsicParameters 
            createTypicalIntrinsicParameters(int imageWidth, int imageHeight) {
        //throw exception if width or height is negative
        if (imageWidth < 0 || imageHeight < 0) {
            throw new IllegalArgumentException();
        }
        
        double focalLength = (imageWidth + imageHeight) / 2.0;
        double horizontalPrincipalPoint = (double)imageWidth / 2.0;
        double verticalPrincipalPoint = (double)imageHeight / 2.0;
        double skewness = 0.0;

        return new PinholeCameraIntrinsicParameters(focalLength, focalLength, 
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);
    }
    
    /**
     * Determines whether provided matrix is considered a valid matrix for
     * pinhole camera intrinsic parameters.
     * Provided matrix must be 3x3, upper triangular and its last element (3,3)
     * must be 1.0.
     * @param m 3x3 and upper triangular matrix to be checked.
     * @return True if provided matrix is valid, false otherwise.
     */
    public static boolean isValidMatrix(Matrix m) {
        return isValidMatrix(m, DEFAULT_VALID_THRESHOLD);
    }
    
    /**
     * Determines whether provided matrix is considered a valid matrix for
     * pinhole camera intrinsic parameters.
     * Provided matrix must be 3x3, upper triangular (up to provided threshold)
     * and its last element (3,3) must be 1.0.
     * @param m 3x3 and upper triangular matrix to be checked.
     * @param threshold Threshold to determine whether matrix is upper 
     * triangular.
     * @return True if provided matrix is valid, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public static boolean isValidMatrix(Matrix m, double threshold) {
        
        if (threshold < 0) {
            throw new IllegalArgumentException();
        }
        
        if (m.getRows() != INTRINSIC_MATRIX_ROWS ||
                m.getColumns() != INTRINSIC_MATRIX_COLS) {
            return false;
        }
        
        //TODO: create isUpperTriangular and isLowerTriangular methods in Utils 
        //class within Algebra library
        
        //check that provided matrix is upper triangular
        for (int u = 0; u < INTRINSIC_MATRIX_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                if (Math.abs(m.getElementAt(u, v)) > threshold) {
                    return false;
                }
            }
        }
        
        //check that lower right element is 1.0
        return Math.abs(m.getElementAt(2, 2)) - 1.0 <= threshold;
    }    
    
    /**
     * Normalizes provided matrix so that element (3,3) becomes one.
     * @param m Matrix to be normalized.
     */
    private static void normalize(Matrix m) {
        double norm = m.getElementAt(2, 2);
        m.multiplyByScalar(1.0 / norm);
    }
    
    /**
     * Clones this instance of pinhole camera matrix.
     * @return A copy of this instance.
     */
    @Override
    @SuppressWarnings("all")
    public PinholeCameraIntrinsicParameters clone() {
        return new PinholeCameraIntrinsicParameters(this);
    }
    
    /**
     * Creates an instance of pinhole camera intrinsic parameters using
     * provided data. Created instance assumes that skewness is zero and that
     * principal point is located at origin of coordinates.
     * This method can be used if for instance actual camera sensor data such as
     * focal length expressed in millimeters and sensor size expressed in 
     * millimeters is known along with the size of the obtained image data.
     * @param focalLength focal length of camera expressed in millimeters.
     * @param sensorWidth camera sensor width expressed in millimeters.
     * @param sensorHeight camera sensor height expressed in millimeters.
     * @param imageWidth captured image width expressed in pixels.
     * @param imageHeight captured image height expressed in pixels.
     * @return an instance of pinhole camera intrinsic parameters.
     */
    public static PinholeCameraIntrinsicParameters create(double focalLength,
            double sensorWidth, double sensorHeight, int imageWidth, 
            int imageHeight) {
        
        //compute the size of a pixel taking into acount sensor and image sizes
        double pixelWidth = sensorWidth / (double)imageWidth; //mm/px
        double pixelHeight = sensorHeight / (double)imageHeight; //mm/px
        
        //compute focal lengths expressed in pixels
        double horizontalFocalLength = focalLength / pixelWidth;
        double verticalFocalLength = focalLength / pixelHeight;
        
        return new PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                verticalFocalLength, 0.0, 0.0, 0.0);
    }
}
