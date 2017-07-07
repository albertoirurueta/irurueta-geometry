/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.KnownBaselineTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 21, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point3D;
import java.util.ArrayList;
import java.util.List;

/**
 * Class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in two views and known camera baseline (camera
 * separation), so that both cameras and reconstructed points are obtained with
 * exact scale.
 */
public class KnownBaselineTwoViewsSparseReconstructor extends 
        BaseTwoViewsSparseReconstructor<
        KnownBaselineTwoViewsSparseReconstructorConfiguration, 
        KnownBaselineTwoViewsSparseReconstructor> {
        
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */    
    public KnownBaselineTwoViewsSparseReconstructor(
            KnownBaselineTwoViewsSparseReconstructorConfiguration configuration,
            KnownBaselineTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor with default configuration.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */    
    public KnownBaselineTwoViewsSparseReconstructor(
            KnownBaselineTwoViewsSparseReconstructorListener listener) 
            throws NullPointerException {
        this(new KnownBaselineTwoViewsSparseReconstructorConfiguration(), 
                listener);
    }
    
    /**
     * Starts reconstruction.
     * If reconstruction has already started and is running, calling this method
     * has no effect.
     * @throws FailedReconstructionException if reconstruction fails for some
     * reason.
     * @throws CancelledReconstructionException if reconstruction is cancelled.
     */
    @Override
    public void start() throws FailedReconstructionException,
            CancelledReconstructionException {
        super.start();
        if (!isRunning() && !isCancelled() && !hasFailed()) {
            try {
                //reconstruction succeeded, so we update scale of cameras and 
                //reconstructed points
                double baseline = mConfiguration.getBaseline();
            
                PinholeCamera camera1 = mEstimatedCamera1.getCamera();
                PinholeCamera camera2 = mEstimatedCamera2.getCamera();
            
                camera1.decompose();
                camera2.decompose();
            
                Point3D center1 = camera1.getCameraCenter();
                Point3D center2 = camera2.getCameraCenter();
            
                double estimatedBaseline = center1.distanceTo(center2);
            
                double scale = baseline / estimatedBaseline;
            
                MetricTransformation3D scaleTransformation = 
                        new MetricTransformation3D(scale);
            
                //update scale of cameras
                scaleTransformation.transform(camera1);
                scaleTransformation.transform(camera2);
            
                mEstimatedCamera1.setCamera(camera1);
                mEstimatedCamera2.setCamera(camera2);
            
                //update scale of reconstructed points
                int numPoints = mReconstructedPoints.size();
                List<Point3D> reconstructedPoints3D = new ArrayList<Point3D>();
                for (int i = 0; i < numPoints; i++) {
                    reconstructedPoints3D.add(mReconstructedPoints.get(i).
                            getPoint());
                }
            
                scaleTransformation.transformAndOverwritePoints(
                        reconstructedPoints3D);
            
                //set scaled points into result
                for (int i = 0; i < numPoints; i++) {
                    mReconstructedPoints.get(i).setPoint(
                            reconstructedPoints3D.get(i));
                }
            } catch (Exception e) {
                throw new CancelledReconstructionException(e);
            }
        }
    }    
}
