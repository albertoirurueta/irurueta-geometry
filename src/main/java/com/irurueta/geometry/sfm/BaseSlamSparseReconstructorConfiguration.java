/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.BaseCalibrationData;

import java.io.Serializable;

/**
 * Contains base configuration for a multiple view sparse reconstructor using SLAM (Simultaneous
 * Location And Mapping) to determine the scale of the scene (i.e. the baseline or separation
 * between cameras) by fusing both camera data and data from sensors like an accelerometer or
 * gyroscope.
 * @param <C> type defining calibration data.
 * @param <T> an actual implementation of a configuration class.
 */
public class BaseSlamSparseReconstructorConfiguration<C extends BaseCalibrationData,
        T extends BaseSlamSparseReconstructorConfiguration> extends BaseSparseReconstructorConfiguration<T>
        implements Serializable {

    /**
     * Calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     */
    private C mCalibrationData;

    /**
     * Constructor.
     */
    public BaseSlamSparseReconstructorConfiguration() { }

    /**
     * Gets calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     * @return calibration data or null.
     */
    public C getCalibrationData() {
        return mCalibrationData;
    }

    /**
     * Specifies calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If set to null, no calibration data will be used.
     * @param calibrationData calibration data or null.
     * @return this instance so that method can be easily chained.
     */
    public T setCalibrationData(C calibrationData) {
        mCalibrationData = calibrationData;
        return (T)this;
    }
}
