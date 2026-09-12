# Implementation Plan

## Task summary

Add missing hyperlinks to the bibliography section of the Antora reference page
(`docs/modules/ROOT/pages/reference.adoc`, `[#bibliography]` anchor at line 49): official publisher/editorial
links for cited books, and official download links for cited papers. This must be done on the `release_1.6.0`
branch (already the base of `feature/18`) so the change appears in the Antora docs for the current release
version, per the issue's explicit instruction. Merging into `master` and then `develop` afterward is a
post-merge release step for the maintainer, outside the scope of this plan.

Source: GitHub issue #18

Of the 15 bibliography entries, 9 already carry a link (Hartley & Zisserman, the PhD thesis, Diebel, both Solà
entries, Kabsch, Peñate-Sánchez/UPnP, Lepetit/EPnP, Baker, Wikipedia Ellipse, Thomsen — 10 actually) and need no
change. The remaining 6 are missing one and are the scope of this plan. Every replacement URL below was verified
by live web search/fetch during planning (not guessed):

| Entry (anchor) | Type | Link to add | Verification |
|---|---|---|---|
| Numerical Recipes (`bib-numerical-recipes`) | Book | `https://numerical.recipes` (numerical.recipes) | The book's own official site (in partnership with Cambridge University Press; also offers free online access to the 3rd edition) — same link already used for this exact book in the sibling repository `irurueta-sorting` (`docs/modules/ROOT/pages/reference.adoc`). The Cambridge catalogue page (`cambridge.org/9780521880688`) was considered first but returns HTTP 403 (access restricted), so it was rejected |
| O'Rourke, *Computational Geometry in C* (`bib-orourke`) | Book | `http://www.science.smith.edu/~jorourke/books/compgeom.html` (author's official book page) | The author's (Joseph O'Rourke) own official page for the 2nd edition — confirmed live (HTTP 200) and its content lists the exact same ISBNs (hardback 0521640105 / paperback 0521649765) already cited. Both Cambridge catalogue URLs for this ISBN return HTTP 403 (access restricted), same failure mode as Numerical Recipes, so they were rejected. Only the plain-`http://` URL resolves — the page's HTTPS certificate is broken/expired (curl: "unable to verify the first certificate" even with `-k`/insecure mode, and connection failures on every HTTPS variant tried, including `cs.smith.edu/~orourke/`) |
| Shepperd, "Quaternion from Rotation Matrix" (`bib-shepperd`) | Paper | `https://arc.aiaa.org/doi/10.2514/3.55767b` (AIAA ARC, DOI) | AIAA's own archive confirms this DOI resolves to "Quaternion from Rotation Matrix," *Journal of Guidance, Control, and Dynamics* — no free copy exists (checked NASA NTRS record 19780048191: no downloadable PDF available there) |
| Fischler & Bolles, RANSAC paper (`bib-fischler-bolles`) | Paper | `https://doi.org/10.1145/358669.358692` (ACM Digital Library, DOI) | ACM's own DOI for "Random Sample Consensus...", *Communications of the ACM*, 24(6), 1981 — no stable free official copy found (only third-party course mirrors, which are not the official source) |
| Torr & Zisserman, MLESAC paper (`bib-torr-zisserman-mlesac`) | Paper | `https://www.robots.ox.ac.uk/~vgg/publications/2000/Torr00/torr00.pdf` (PDF, robots.ox.ac.uk) | Hosted on the paper's own author (Torr/Zisserman, Oxford VGG) publications page — confirmed downloadable (834 KB PDF fetched successfully) |
| Chum & Matas, PROSAC paper (`bib-chum-matas-prosac`) | Paper | `https://cmp.felk.cvut.cz/~matas/papers/chum-prosac-cvpr05.pdf` (PDF, cmp.felk.cvut.cz) | Hosted on co-author Matas's own institutional page (Czech Technical University, Center for Machine Perception) — confirmed downloadable (1.2 MB PDF fetched successfully) |

No architectural choices were needed — this is a same-file, same-format documentation edit, so nothing was
ambiguous enough to ask the user about.

## Current code state

- The bibliography lives entirely in `docs/modules/ROOT/pages/reference.adoc`, under the `[#bibliography]`
  section (line 49 onward), split into `=== Books` (line 62), `=== Papers` (line 192), and
  `=== Web resources and code` (line 264) subsections.
- Each entry is a single AsciiDoc list item (`* ...`) starting with an anchor (`[[bib-xxx]]`), followed by the
  full citation, and — for entries that already have one — an inline hyperlink using AsciiDoc's
  `https://url[link text]` syntax, e.g. line 66: `https://www.cambridge.org/9780521540513[Publisher page]`.
- No source code, build config, or other Antora pages need to change — this is a single-file text edit.

## Implementation steps

### Group 1 (Parallelizable: yes — a single task editing one file)

- [x] Task 1. Add the 6 missing bibliography links in `docs/modules/ROOT/pages/reference.adoc` — all 6 links
      added and verified rendering correctly in the built `reference.html` (Antora build via `iru-build-docs`);
      no tests applicable (documentation-only change); no code-quality gate applicable.
  - [x] Task 1.1. `bib-numerical-recipes` (line ~160-161): after "2007." and before the line break (`+`), insert
        `https://numerical.recipes[numerical.recipes].` — the book's own official site, matching the exact link
        already used for this book in the sibling repository `irurueta-sorting`
        (`docs/modules/ROOT/pages/reference.adoc`).
  - [x] Task 1.2. `bib-orourke` (line ~186-190): after the citation ("...1998 (chapter 1 covers polygon
        triangulation, including ear clipping)."), insert a line break (`+`) and
        `http://www.science.smith.edu/~jorourke/books/compgeom.html[Author's book page].` before the existing
        "Not cited in the source..." sentence. Note the plain `http://` scheme (not `https://`) — the page's
        HTTPS certificate is broken, confirmed by direct connectivity testing during planning.
  - [x] Task 1.3. `bib-shepperd` (line ~206-209): after "pp. 223-224.", insert a line break (`+`) and
        `https://arc.aiaa.org/doi/10.2514/3.55767b[AIAA ARC (DOI)].` before the existing "Implemented by..."
        sentence.
  - [x] Task 1.4. `bib-fischler-bolles` (line ~234-237): after "pp. 381-395.", insert a line break (`+`) and
        `https://doi.org/10.1145/358669.358692[ACM Digital Library (DOI)].` before the existing "The original
        RANSAC paper..." sentence.
  - [x] Task 1.5. `bib-torr-zisserman-mlesac` (line ~245-247): after "pp. 138-156.", insert a line break (`+`)
        and `https://www.robots.ox.ac.uk/~vgg/publications/2000/Torr00/torr00.pdf[PDF (robots.ox.ac.uk)].`
        before the existing "Formalizes the MSAC..." sentence.
  - [x] Task 1.6. `bib-chum-matas-prosac` (line ~254-256): after "pp. 220-226.", insert a line break (`+`) and
        `https://cmp.felk.cvut.cz/~matas/papers/chum-prosac-cvpr05.pdf[PDF (cmp.felk.cvut.cz)].` before the
        existing "The PROSAC paper..." sentence.
  - [x] Task 1.7. Verify no other bibliography entry was accidentally altered, and that AsciiDoc line-break
        (`+`) conventions and the surrounding sentence punctuation stay consistent with the existing entries
        (compare against an untouched entry like `bib-hartley-zisserman` or `bib-diebel`) — confirmed via
        `git diff`: only the 6 targeted entries changed, no other bibliography entry touched.

Verification: this is a documentation-only change with no code/tests affected, so no `iru-gate-runner` test run is
needed. Instead, build the Antora site locally (`Skill({skill: "iru-build-docs"})` or the repository's own Antora
build command) and visually confirm the 6 new links render correctly and are clickable on the built
`reference.html` page.
