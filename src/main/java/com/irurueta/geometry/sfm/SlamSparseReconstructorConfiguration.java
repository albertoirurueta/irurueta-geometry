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

import com.irurueta.geometry.slam.SlamCalibrationData;

import java.io.Serializable;

/**
 * Contains configuration for a multiple view sparse reconstructor using SLAM (Simultaneous
 * Location And Mapping) to determine the scale of the scene (i.e. the baseline or separation
 * between initial cameras) by fusing both camera data and data from sensors like an
 * accelerometer or gyroscope.
 * This configuration assumes that an orientation relative to the first view will be
 * estimated.
 */
public class SlamSparseReconstructorConfiguration extends
        BaseSlamSparseReconstructorConfiguration<SlamCalibrationData, SlamSparseReconstructorConfiguration>
        implements Serializable {

    /**
     * Creates an instance of a multiple views sparse reconstructor configuration with slam
     * estimation.
     * @return configuration instance.
     */
    public static SlamSparseReconstructorConfiguration make() {
        return new SlamSparseReconstructorConfiguration();
    }
}
