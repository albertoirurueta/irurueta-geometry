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

/**
 * Enumeration indicating camera type.
 */
public enum CameraType {
    /**
     * Pinhole camera. A pinhole camera is a linear mapping between 3D and 2D
     * worlds. Pinhole cameras only take into account translation, rotation and
     * camera intrinsic parameters such as focal length, aspect ratio, skewness
     * and principal point.
     * Pinhole cameras perform projective mappings between 3D and 2D worlds,
     * in other words, the farther an object is, the smaller is represented or
     * parallel lines converge into vanishing points.
     * Pinhole cameras cannot be used for orthographic projections (where
     * parallelism between lines is preserved and there are no vanishing points).
     */
    PINHOLE_CAMERA
}
