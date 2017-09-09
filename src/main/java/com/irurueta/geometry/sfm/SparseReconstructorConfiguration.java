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


import java.io.Serializable;

/**
 * Contains configuration for a multiple view sparse reconstructor.
 * @author Alberto Irurueta (alberto@irurueta.com)
 */
public class SparseReconstructorConfiguration extends
        BaseSparseReconstructorConfiguration<SparseReconstructorConfiguration> implements Serializable {

    /**
     * Constructor.
     */
    public SparseReconstructorConfiguration() { }

    /**
     * Creates an instance of a sparse reconstructor configuration.
     * @return configuration instance.
     */
    public static SparseReconstructorConfiguration make() {
        return new SparseReconstructorConfiguration();
    }
}
