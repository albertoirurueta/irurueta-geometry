# irurueta-geometry

Geometry structures and utilities

[![Build Status](https://github.com/albertoirurueta/irurueta-geometry/actions/workflows/master.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-geometry/actions/workflows/master.yml)
[![Build Status](https://github.com/albertoirurueta/irurueta-geometry/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-geometry/actions/workflows/develop.yml)
[![Maven Central](https://img.shields.io/maven-central/v/com.irurueta/irurueta-geometry.svg)](https://search.maven.org/artifact/com.irurueta/irurueta-geometry)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=bugs)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-geometry&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)

## Project Status

| | |
| --- | --- |
| Language | Java 21 |
| Build tool | Maven |
| Current development version | `1.6.0-SNAPSHOT` |
| Latest release | `1.5.0` |
| License | [Apache License 2.0](LICENSE.txt) |
| CI | GitHub Actions -- release builds (`master.yml`), `develop` branch builds (`develop.yml`), and a release-sync automation that bumps the development version after each release (`sync.yml`) |
| Quality | SonarCloud, JaCoCo (coverage), Checkstyle, SpotBugs, PMD |

## Documentation

* [Antora documentation site](https://albertoirurueta.github.io/irurueta-geometry)
* [Maven site report](https://albertoirurueta.github.io/irurueta-geometry/mvn-site) (Javadoc, coverage, test, and code-quality reports)
* [SonarCloud dashboard](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-geometry)
* [Changelog](CHANGELOG.md)

## Installation

Add the following dependency to your project:

Latest release:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-geometry</artifactId>
    <version>1.5.0</version>
    <scope>compile</scope>
</dependency>
```

Latest snapshot:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-geometry</artifactId>
    <version>1.6.0-SNAPSHOT</version>
    <scope>compile</scope>
</dependency>
```

## How It Works

irurueta-geometry provides 2D/3D geometric primitives (points, lines, planes, conics, transformations) built
around projective geometry and homogeneous coordinates, plus robust estimators (RANSAC, LMedS, MSAC, PROSAC,
PROMedS) that fit those entities -- including a pinhole camera model -- from noisy point/line correspondences.

A minimal example using two of the core primitives, `Point2D` and `Line2D`:

```java
Point2D pointA = new InhomogeneousPoint2D(0.0, 0.0);
Point2D pointB = new InhomogeneousPoint2D(1.0, 1.0);
Line2D line = new Line2D(pointA, pointB);

Point2D point = new InhomogeneousPoint2D(2.0, 0.0);
double distance = line.signedDistance(point);
```

See the [Antora documentation site](https://albertoirurueta.github.io/irurueta-geometry) for an overview of the
library's core concepts, including the robust estimation and camera-calibration classes.

## License

This project is licensed under the [Apache License 2.0](LICENSE.txt).
