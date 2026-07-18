# Packet audit - 2026-07-18

## Why reopen

Prior closes marked MAP/CUT/FREE/APP/M9 packages `DONE` before exit gates
were truly met. This audit reopens under-scoped work.

| Packet | Verdict | Why |
|---|---|---|
| CUT WP-130…135 | Reopened `OPEN` | Hole tags only; no slots; no cut-loop boundary/modelling/density proof; multi-bore fixtures not named as cutout.* |
| MAP WP-121/123/125 | Reopened `OPEN` | Floor exists; MAP-B adversaries, MAP-D provenance, MAP-F density/determinism thin |
| MAP WP-120/122/124 | Remain `DONE` | Recon tags, UV-grid floor, CLI/app path still valid |
| FREE WP-140…145 | Reopened `OPEN` | freeform_patch was an alias of mapped_patch (four-sided) |
| APP WP-151/152 | Reopened `OPEN` | Editing/overlays were paperwork; matrix must refresh after family fixes |
| APP WP-150 | Remain `DONE` pending refresh during Phase 4 |
| M9 WP-161 | `BLOCKED` | Windows empty-dir rebuild missing; was incorrectly `DONE` |
| M9 WP-160/162/163 | Remain `DONE` | Linux inventory/package still valid |
| M9 WP-164 | `BLOCKED` | Plasticity↔Blender unavailable |
| M9 WP-165 | `OPEN` | Cannot close M9 |

## Next

Execute CUT → MAP gaps → FREE → APP → M9 status correction per remediation plan.
