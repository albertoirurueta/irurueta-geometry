# Implementation Plan

## Task summary

Fix the flaky unit test `PinholeCameraTest.testProjectBackProjectQuadricAndConic`
(`src/test/java/com/irurueta/geometry/PinholeCameraTest.java:2627`). It intermittently fails with
`CameraException` ← `DualQuadricNotAvailableException` ← `RankDeficientMatrixException` ← `SingularMatrixException`
at line 2795 (`camera.project(quadric)`).

Root cause (confirmed by reading the code): the `quadric` comes from `camera.backProject(conic)`, i.e.
`Q = C^T · conic · C`, which is always a rank-deficient cone (apex at the camera center, `Q · center = 0`).
`Camera.project(Quadric, Conic)` (`Camera.java:283`) calls `Quadric.getDualQuadric()` (`Quadric.java:261`), which
inverts that matrix via `Utils.inverse`/`LUDecomposer`. `LUDecomposer` only reports singular on an *exactly* zero
pivot, so the test passes on floating-point noise and fails when a pivot hits exactly `0.0`. Production code is
correct (a degenerate quadric has no dual) — the defect is entirely in the test.

Chosen approach (the issue's "preferred" fix, with a test-only change): stop projecting a known-degenerate cone.
- The back-projection test keeps its back-projection checks but no longer calls `camera.project(quadric)`.
- `Camera.project(Quadric)` / `project(Quadric, Conic)` keep their coverage through a new test that projects a
  genuinely non-degenerate quadric, built from 9 random points, and that checks a real geometric property
  (the projected 9 points are locus of the projected conic), not just that the two overloads agree. It uses a retry
  loop mirroring the existing dual-quadric test (`TIMES` attempts, `continue` on `CameraException`) so an unlucky
  random draw cannot fail it.
- Rejected: wrapping the existing test in a retry loop only — it would keep asserting something meaningless
  (inverting a singular matrix) and hide the real contract.

Source: GitHub issue #34
Base branch: develop

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Restructure the assertion so it doesn't depend on being able to invert a known-singular matrix" | Task 1 |
| 2 | derived: `testProjectBackProjectQuadricAndConic` no longer fails with `SingularMatrixException` for any random draw | Task 1, Task 3 |
| 3 | derived: `Camera.project(Quadric)` and `project(Quadric, Conic)` stay covered by a test using a non-degenerate quadric | Task 2, Task 3 |
| 4 | derived: no production code changes; build, license-header, Javadoc and code-quality checks stay green | Task 3 |

## Current code state

- `src/test/java/com/irurueta/geometry/PinholeCameraTest.java`
  - `testProjectBackProjectQuadricAndConic` (line 2627): builds a conic from 5 random points (retry while
    `rank(conicMatrix) < 5`), a random `PinholeCamera`, calls `camera.backProject(conic)` /
    `backProject(conic, quadric2)`, checks the back-projected points are locus of the quadric, then (lines 2794-2801)
    calls `camera.project(quadric)` and `project(quadric, conic3)`, normalizes `conic`, `conic2`, `conic3` and asserts
    only `conic2.asMatrix() == conic3.asMatrix()`. The comparison with the original `conic` is never asserted.
    Throws list: `WrongSizeException, DecomposerException, CoincidentPointsException, CameraException`.
  - `testProjectBackProjectDualQuadricAndDualConic` (~line 2480-2623) is the model for the retry pattern:
    `var numValid = 0; for (var t = 0; t < TIMES; t++) { ... try { ... } catch (final CameraException e) { continue; }
    numValid++; break; } assertTrue(numValid > 0);` (`TIMES = 10`).
  - Constants available: `ABSOLUTE_ERROR = 1e-6`, `LARGE_ABSOLUTE_ERROR = 5e-5`, `VERY_LARGE_ABSOLUTE_ERROR = 1e-4`,
    `MIN_RANDOM_VALUE = -100`, `MAX_RANDOM_VALUE = 100`, `HOM_2D_COORDS`, `PINHOLE_CAMERA_ROWS/COLS`.
- `src/main/java/com/irurueta/geometry/Camera.java:267-293`: `project(Quadric)` / `project(Quadric, Conic)`
  normalize the quadric, call `getDualQuadric()`, `project(DualQuadric, DualConic)`, then `dualConic.conic(result)`;
  `GeometryException` is wrapped in `CameraException`. No change needed.
- `src/main/java/com/irurueta/geometry/Quadric.java`: `Quadric(Point3D x9)` constructor (line 87) throws
  `CoincidentPointsException`; `isLocus(Point3D, double)` (line 104); `dualQuadric` at 261. No change needed.
- Docs (`docs/modules/ROOT/pages/pinhole-camera.adoc:513-527`) already describe the duality; no docs change.
  `CHANGELOG.md` has an empty `## [Unreleased]` section.
- Stack: Java library, Maven (no wrapper), JUnit 5.13.4, Checkstyle config at repo root.

## Implementation steps

### Group 1 — Fix the flaky test and keep `project(Quadric)` covered (Parallelizable: no — Task 2 edits the same test file and class as Task 1)

- [x] Task 1. Remove the singular-matrix projection from `testProjectBackProjectQuadricAndConic` _(java)_ — done: test renamed `testBackProjectConic`, camera-center (cone apex) `isLocus` assertion added via `computeCameraCenterSVD()`; `PinholeCameraTest` only. Full suite 4003 tests green; `CameraException` still needed (SVD call).
  - [x] Task 1.1. In `PinholeCameraTest.java`, delete the "project quadric into conic" block (the `camera.project(quadric)` /
    `camera.project(quadric, conic3)` calls, the three `normalize()` calls and the final `assertEquals`) from
    `testProjectBackProjectQuadricAndConic` (lines ~2794-2801).
  - [x] Task 1.2. Add assertions to that test that do not need inverting the cone: assert the camera center is locus of
    the back-projected quadric (`quadric.isLocus(camera.getCameraCenter(), ABSOLUTE_ERROR)` — the cone's apex, derived
    from `Q · center = 0`), keeping the existing `isLocus` checks on the 5 back-projected points. Rename the test to
    `testBackProjectConic` (or keep the name if the maintainer prefers; no other file references it), and drop the
    now-unused `CameraException` from its `throws` list only if the compiler no longer needs it.
- [x] Task 2. Add `testProjectQuadric` using a non-degenerate quadric _(java)_ — done in `PinholeCameraTest`; `Camera.project(Quadric)` lines 266-291 covered (Camera 92% line / 100% branch; only the GeometryException catch, lines 292-293, is uncovered); no new test-source quality tooling findings (Checkstyle/PMD/SpotBugs are not wired for test sources); license header already compliant.
  - Deviation (Task 2.3): the 'projected 9 points are locus of the projected conic' assertion was dropped. A projected quadric is its silhouette (outline) conic, so images of surface points lie inside it, not on it (failed immediately). Replaced with a genuine independent check: `conic.getDualConic()` equals `camera.project(quadric.getDualQuadric())` after normalize, tolerance `LARGE_ABSOLUTE_ERROR`; measured max element diff over 500 random runs was 1.6e-11.
  - [x] Task 2.1. In `PinholeCameraTest.java`, right after the back-projection test, add `@Test void testProjectQuadric()`
    following the retry pattern of the dual-quadric test (`numValid`, `for t < TIMES`, `try { ... } catch (final
    CameraException | CoincidentPointsException e) { continue; }`, `numValid++; break;`, `assertTrue(numValid > 0)`).
  - [x] Task 2.2. Inside each attempt: draw 9 random `InhomogeneousPoint3D` points (values in
    `MIN_RANDOM_VALUE..MAX_RANDOM_VALUE`, e.g. via `UniformRandomizer`), build `new Quadric(p1..p9)`, create a random
    `PinholeCamera` from `Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, ...)`.
  - [x] Task 2.3. Assertions: `camera.project(quadric)` equals `camera.project(quadric, conic3)` after `normalize()` on
    both (`assertEquals(conic2.asMatrix(), conic3.asMatrix())`), and each of the 9 points projected with
    `camera.project(point)` is locus of the projected conic (`conic2.isLocus(point2D, <tolerance>)`). Start with
    `LARGE_ABSOLUTE_ERROR`; if conditioning of the random draw makes it non-robust, restrict the geometric assertion to
    the points/tolerance that stay stable rather than widening the retry window blindly (any `AssertionError` from
    ill-conditioning would itself be a new flake, so verify in Task 3).
  - [x] Task 2.4. Confirm `project(Quadric)` no longer relies on a quadric that is singular by construction (the 9-point
    quadric is full rank with probability 1; a rank-deficient draw surfaces as `CameraException` and is retried).

### Group 2 — Validate (Parallelizable: yes — single task)

- [x] Task 3. Prove the test is no longer flaky and the build stays green _(java)_ -- validated: 0 failures, build green; no production code changed.
  - [x] Task 3.1. Delegate to `iru-gate-runner` (`Skill: iru-java-test`, selector
    `PinholeCameraTest#testBackProjectConic+testProjectQuadric` — or the renamed method names used) so test output stays
    out of the main context.
    _Result: 2 tests, 0 failures._
  - [x] Task 3.2. Stability check: temporarily annotate the two tests with `@RepeatedTest(500)` (or loop them) and run via
    `iru-gate-runner`; require 0 failures, then revert the temporary annotation before finishing.
    _Result: 1000 executions via temporary @RepeatedTest(500), 0 failures; annotation reverted, none in diff._
  - [x] Task 3.3. Run the full `PinholeCameraTest` class and the complete unit suite via `iru-gate-runner`; confirm nothing else
    regressed and `Camera.project(Quadric)` remains covered (`iru-java-coverage` on `PinholeCameraTest`).
    _Result: PinholeCameraTest 30 tests OK; full suite 4003 tests, 0 failures; Camera.project(Quadric) covered; Camera 92.4% line, PinholeCamera 92.9% line._
  - [x] Task 3.4. Run `iru-check-license` on `PinholeCameraTest.java` and `iru-java-code-quality` (Checkstyle) on the changed test
    file; fix any findings in the changed code.
    _Result: license header compliant; Checkstyle/PMD/SpotBugs are not wired for test sources in this pom, 0 findings on the test file._
- [x] Task 4. Record the fix in `CHANGELOG.md` _(untagged)_ -- CHANGELOG.md updated.
  - [x] Task 4.1. Under `## [Unreleased]` add `### Fixed` with one line: made `PinholeCameraTest` no longer project a
    back-projected (degenerate) quadric, which failed intermittently with `SingularMatrixException` (#34).
