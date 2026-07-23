# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.6.0] - 2026-07-23

### Added

- New Antora-based documentation site (`docs/`) covering the library's core geometric entities in depth --
  transformations, rotations, points/lines/planes, conics/quadrics (including their duals), triangles/polygons,
  the pinhole camera model, and boxes/KD-trees -- along with the robust estimator families, each grounded in the
  actual source and cross-referenced against a shared bibliography.

### Changed

- Raised the Java compiler source/target and CI JDK version from 17 to 21.
- Replaced the standalone `sonar-scanner` CLI step in the GitHub workflows with the `sonar-maven-plugin`
  (`mvn sonar:sonar`), so SonarCloud analysis runs under the same JDK 21 toolchain as the rest of the build.
  Sonar organization/project key/host settings moved from `sonar-project.properties` (removed) into
  `pom.xml` properties.

## [1.5.0] - 2026-03-03

### Changed

- Updated the `irurueta-numerical` compile dependency to version 1.5.0.

## [1.4.0] - 2025-12-18

### Changed

- Updated the `irurueta-numerical` compile dependency to version 1.4.0.

## [1.3.2] - 2025-09-22

### Changed

- Bumped compile-scope dependencies `irurueta-algebra` and `irurueta-numerical` to version 1.3.2.

## [1.3.1] - 2025-09-19

No user-facing changes in this release — it consists solely of build tooling, CI workflow, and test-only updates.

## [1.3.0] - 2024-12-05

### Changed

- Raised the minimum required Java version from Java 7 to Java 17 (**breaking change** for consumers still on
  older JDKs).
- Bumped compile-scope dependencies: `irurueta-statistics` to 1.3.2, `irurueta-sorting` to 1.3.1,
  `irurueta-algebra` to 1.3.0, `irurueta-numerical` to 1.3.0.
- Large source-wide, non-behavioral refactor: adoption of local-variable type inference (`var`), removal of the
  `m`-prefix convention from private fields, and migration of the test suite from JUnit 4 to JUnit 5. No
  functional or public API signature changes were found.

## [1.2.0] - 2023-12-04

### Changed

- Renamed `RobustEstimatorMethod` enum constants `LMedS`/`PROMedS` to `LMEDS`/`PROMEDS` throughout every robust
  estimator in `com.irurueta.geometry.estimators` (**breaking change**).
- Renamed `PinholeCameraEstimatorType` enum constants `EPnP_PINHOLE_CAMERA_ESTIMATOR`/`UPnP_PINHOLE_CAMERA_ESTIMATOR`
  to `EPNP_PINHOLE_CAMERA_ESTIMATOR`/`UPNP_PINHOLE_CAMERA_ESTIMATOR` (**breaking change**).
- Bumped dependencies: `irurueta-statistics`, `irurueta-sorting`, `irurueta-algebra` to 1.2.0, `irurueta-numerical`
  to 1.2.1.
- `Rotation3D` now explicitly implements `Serializable`.

### Fixed

- `NotLocusException` changed from package-private to `public`, so it can now be referenced and caught outside
  `com.irurueta.geometry`, even though many public methods (e.g. `Circle.getTangentLineAt`,
  `Conic.getTangentLineAt`, `Ellipse.getTangentLineAt`) already declared it in their `throws` clause.

## [1.1.0] - 2021-12-11

Initial release.

### Added

- Core 2D/3D geometric primitives: points (homogeneous/inhomogeneous), lines, planes, circles, spheres, ellipses,
  ellipsoids, triangles, polygons, boxes, rectangles, conics/dual conics, quadrics/dual quadrics.
- Rotation types and utilities: 2D rotations, 3D rotations (matrix, axis-angle, quaternion) with conversions
  between representations.
- 2D and 3D geometric transformations: Euclidean, similarity/metric, affine, and projective transformations.
- Pinhole camera model: camera representation, intrinsic parameters, and camera estimation via DLT, EPnP, and
  UPnP point/line-plane correspondence estimators.
- Robust estimation framework for fitting geometric entities (points, lines, planes, circles, conics, quadrics,
  transformations, cameras) from noisy/outlier-contaminated data, supporting RANSAC, LMedS, MSAC, PROSAC, and
  PROMedS algorithms.
- Non-linear refiners to improve initial robust-estimator results for transformations, cameras, and points using
  inlier data.
- Point-in-polygon triangulation (Van Gogh triangulator) for 2D and 3D polygons.
- Spatial indexing via KD-trees (2D/3D) for nearest-neighbor queries.
- Accuracy/uncertainty representation classes for 2D/3D estimation results.
- Comprehensive exception hierarchy for geometric error conditions (coincident/colinear/coplanar points,
  degenerate configurations, locking, normalization, etc.).

[Unreleased]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.6.0...HEAD
[1.6.0]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.5.0...1.6.0
[1.5.0]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.4.0...1.5.0
[1.4.0]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.3.2...1.4.0
[1.3.2]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.3.1...1.3.2
[1.3.1]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.3.0...1.3.1
[1.3.0]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/albertoirurueta/irurueta-geometry/compare/1.1.0...1.2.0
[1.1.0]: https://github.com/albertoirurueta/irurueta-geometry/releases/tag/1.1.0
