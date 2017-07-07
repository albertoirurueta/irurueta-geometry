/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.LMSEInhomogeneousSinglePoint3DTriangulator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 30, 2015
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import java.util.List;

/**
 * Triangulates matched 2D points into a single 3D one by using 2D point
 * correspondences on different views along with the corresponding cameras on
 * each of those views by finding an LMSE solution to homogeneous systems of
 * equations
 */
public class LMSEInhomogeneousSinglePoint3DTriangulator extends
        SinglePoint3DTriangulator{

    /**
     * Indicates if by default an LMSE (Least Mean Square Error) is allowed if
     * more correspondences than the minimum are provided
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;
    
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if 
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 6 first
     * correspondences will be used
     */
    private boolean mAllowLMSESolution;
    
    /**
     * Constructor
     */
    public LMSEInhomogeneousSinglePoint3DTriangulator(){
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list
     * @param cameras camera for each view where 2D points are represented
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation
     */
    public LMSEInhomogeneousSinglePoint3DTriangulator(List<Point2D> points2D, 
            List<PinholeCamera> cameras) throws IllegalArgumentException{
        super(points2D, cameras);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor
     * @param listener listener to notify events generated by instances of this
     * class
     */
    public LMSEInhomogeneousSinglePoint3DTriangulator(
            SinglePoint3DTriangulatorListener listener){
        super(listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list
     * @param cameras cameras for each view where 2D points are represented
     * @param listener listener to notify events generated by instances of this
     * class
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum
     * required to compute triangulation
     */
    public LMSEInhomogeneousSinglePoint3DTriangulator(List<Point2D> points2D, 
            List<PinholeCamera> cameras, 
            SinglePoint3DTriangulatorListener listener) 
            throws IllegalArgumentException{
        super(points2D, cameras, listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if 
     * more correspondences than the minimum are provided. If false, the 
     * exceeding correspondences will be ignored and only the 2 first matches
     * corresponding to the first 2 views will be used
     * @return true if LMSE solution is allowed, false otherwise
     */
    public boolean isLMSESolutionAllowed(){
        return mAllowLMSESolution;
    }
    
    /**
     * Specifies if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 2 first matches
     * corresponding to the first 2 views will be used
     * @param allowed true if LMSE solution is allowed, false otherwise
     * @throws LockedException if estimator is locked
     */
    public void setLMSESolutionAllowed(boolean allowed) throws LockedException{
        if(isLocked()) throw new LockedException();
        mAllowLMSESolution = allowed;
    }        

    /**
     * Returns type of triangulator
     * @return type of triangulator
     */
    @Override
    public Point3DTriangulatorType getType() {
        return Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR;
    }

    /**
     * Internal method to triangulate provided matched 2D points being projected 
     * by each corresponding camera into a single 3D point.
     * At least 2 matched 2D points and their corresponding 2 cameras are 
     * required to compute triangulation. If more views are provided, an 
     * averaged solution is found.
     * This method does not check whether instance is locked or ready
     * @param points2D matched 2D points. Each point in the list is assumed to
     * be projected by the corresponding camera in the list
     * @param cameras list of cameras associated to the matched 2D point on the
     * same position as the camera on the list
     * @param result instance where triangulated 3D point is stored
     * @throws Point3DTriangulationException if triangulation fails for some
     * other reason (i.e. degenerate geometry, numerical instabilities, etc)
     */    
    @Override
    protected void triangulate(List<Point2D> points2D, 
            List<PinholeCamera> cameras, Point3D result) 
            throws Point3DTriangulationException {
        try{
            mLocked = true;
            
            if(mListener != null){
                mListener.onTriangulateStart(this);
            }
            
            int numViews = cameras.size();
            
            Matrix a;
            double[] b;
            if(mAllowLMSESolution){
                //each view will add 2 equations to the linear system of
                //equations
                a = new Matrix(2 * numViews, 3);
                b = new double[2 * numViews];
            }else{
                a = new Matrix(2 * MIN_REQUIRED_VIEWS, 3);
                b = new double[2 * MIN_REQUIRED_VIEWS];
            }
            
            Point2D point;
            PinholeCamera camera;
            Plane horizontalAxisPlane = new Plane();
            Plane verticalAxisPlane = new Plane();
            Plane principalPlane = new Plane();
            int row = 0;
            double rowNorm;
            for(int i = 0; i < numViews; i++){
                point = points2D.get(i);
                camera = cameras.get(i);
                
                //to increase accuracy
                point.normalize();
                camera.normalize();
                
                double homX = point.getHomX();
                double homY = point.getHomY();
                double homW = point.getHomW();
                
                //pick rows of camera corresponding to different planes
                //(we do not normalize planes, as it would introduce errors)
                
                //1st camera row (p1T)
                camera.verticalAxisPlane(verticalAxisPlane);
                //2nd camera row (p2T)
                camera.horizontalAxisPlane(horizontalAxisPlane);
                //3rd camera row (p3T)
                camera.principalPlane(principalPlane);

                //1st equation
                a.setElementAt(row, 0, homX * principalPlane.getA() -
                        homW * verticalAxisPlane.getA());
                a.setElementAt(row, 1, homX * principalPlane.getB() -
                        homW * verticalAxisPlane.getB());
                a.setElementAt(row, 2, homX * principalPlane.getC() -
                        homW * verticalAxisPlane.getC());
                
                b[row] = homW * verticalAxisPlane.getD() -
                        homX * principalPlane.getD();
                
                //normalize equation to increase accuracy
                rowNorm = Math.sqrt(Math.pow(a.getElementAt(row, 0), 2.0) +
                        Math.pow(a.getElementAt(row, 1), 2.0) +
                        Math.pow(a.getElementAt(row, 2), 2.0));
                
                a.setElementAt(row, 0, a.getElementAt(row, 0) / rowNorm);
                a.setElementAt(row, 1, a.getElementAt(row, 1) / rowNorm);
                a.setElementAt(row, 2, a.getElementAt(row, 2) / rowNorm);
                b[row] /= rowNorm;
                
                //2nd equation
                row++;
                
                a.setElementAt(row, 0, homY * principalPlane.getA() -
                        homW * horizontalAxisPlane.getA());
                a.setElementAt(row, 1, homY * principalPlane.getB() -
                        homW * horizontalAxisPlane.getB());
                a.setElementAt(row, 2, homY * principalPlane.getC() -
                        homW * horizontalAxisPlane.getC());
                
                b[row] = homW * horizontalAxisPlane.getD() -
                        homY * principalPlane.getD();
                
                //normalize equation to increase accuracy
                rowNorm = Math.sqrt(Math.pow(a.getElementAt(row, 0), 2.0) +
                        Math.pow(a.getElementAt(row, 1), 2.0) +
                        Math.pow(a.getElementAt(row, 2), 2.0));
                
                a.setElementAt(row, 0, a.getElementAt(row, 0) / rowNorm);
                a.setElementAt(row, 1, a.getElementAt(row, 1) / rowNorm);
                a.setElementAt(row, 2, a.getElementAt(row, 2) / rowNorm);
                b[row] /= rowNorm;
                
                if(!mAllowLMSESolution && i == MIN_REQUIRED_VIEWS) break;
            }
            
            //make SVD to find solution of A * M = b
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
            decomposer.decompose();
            
            //solve linear system of equations to obtain inhomogeneous 
            //coordinates of triangulated point
            double[] solution = decomposer.solve(b);
            
            result.setInhomogeneousCoordinates(solution[0], solution[1], 
                    solution[2]);
            
            if(mListener != null){
                mListener.onTriangulateEnd(this);
            }
        }catch(AlgebraException e){
            throw new Point3DTriangulationException(e);
        }finally{
            mLocked = false;
        }
    }
    
}
