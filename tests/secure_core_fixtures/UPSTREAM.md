# Frozen weftocct fixture evidence

This directory is a Weft-owned snapshot of the committed `fixtures/` tree from:

- repository: `D:\weftocct` (local evidence donor);
- commit: `6eaa0708d5313210779b6d26c839306028db5f98`;
- imported: 2026-07-17;
- files: 308;
- bytes: 19,157,865.

The donor working tree was dirty at import time. The snapshot was produced with
`git archive` from the exact commit, so it excludes all uncommitted work,
including the unreviewed twisted-cylinder fixture.

`manifest.canonical.sha256` remains the upstream integrity root. The manifest
contains 106 reviewed records: 77 OCCT-procedural, 21 derived-corruption, five
native-fault, and three audited-external fixtures. Procedural generated STEP and
summary outputs are intentionally not committed upstream; their recipes,
expected hashes, reviewed oracles, generator source, schemas, and licences are
retained here.

Paths inside the imported manifest remain upstream provenance strings. In
particular, references beginning with `fixtures/` resolve relative to this
directory after removing that prefix, while `tests/fixtures/` native harness
paths identify donor code not copied into the product runtime. Nothing in this
snapshot creates a runtime dependency on `D:\weftocct`.

Do not edit imported evidence in place. A future refresh must name a new donor
commit, reproduce the archive, pass `secure_fixture_catalog`, and record the
change in a new evidence entry.
