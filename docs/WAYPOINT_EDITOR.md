# Offline warehouse waypoint editor (Stage 3)

From the repository root, run:

```sh
/usr/bin/python3 scripts/edit_warehouse_waypoints.py
```

Open **http://127.0.0.1:8765** in a local browser. Ctrl+C stops the server. No ROS environment, simulation, build, CDN, npm install or internet is required. The installed system Python needs Pillow and PyYAML. This tool imports no ROS transport and publishes no goals. It changes no canonical route, Nav2, SLAM, RPP, governor or overlay configuration.

The defaults read `/tmp/b2w-waypoint-map/warehouse.yaml`, its referenced image, `course_navigation/config/nav2.yaml` and `course_navigation/config/warehouse_route.yaml`. Paths can be selected explicitly with `--map`, `--nav-config` and `--route`; local and global configured padded footprints must agree. The tool uses configuration geometry, not a claim about a live session's loaded parameters. The actual inspected map is 800×800 at 0.05 m/pixel, origin `[-20,-20,0]`. Image SHA256: `e90f2dd81c89d41828ddb9f60732bdcae5cc7526dcf4a8919638a335d4a585f3`. Metadata is parsed on every launch; these values are not hard-coded coordinate assumptions.

Use the mouse wheel to zoom at the cursor; middle/right drag or Pan mode moves the view. The grid is in map meters (1 m, automatically 5 m when very far out). The cursor shows continuous image coordinates and containing OccupancyGrid cell coordinates. Orange outlines show the original poses; cyan shows the candidate. Select G1–G5, drag their centers, or drag the arrow tips to change yaw. Exact numeric inputs and the expandable original/candidate values show map-frame coordinates and radians. Select mode also drags polygon vertices. Drawing a corridor or restricted polygon uses one click per vertex and **Finish polygon**; self-intersections, duplicate vertices and zero-area polygons are rejected. Select a vertex to select that annotation, then **Delete selected annotation**. Use entrance/exit modes to place separate passage poses; they are never formal G goals. Select passage poses from the dropdown to edit or delete them.

Undo/redo includes poses, yaw, polygons and passage markers. Reset restores the initial set and clears annotations; reset itself is undoable. Escape cancels an unfinished drawing or drag. Ctrl+Z and Ctrl+Shift+Z work outside input controls. Edits live in browser memory until export; refreshing discards edits. An exported candidate is an inspection artifact; importing an earlier candidate for continued editing is not provided.

**Check all poses** checks every goal and passage marker using the complete filled padded footprint. Enable **Rotational sweep** to draw 73 poses over 360° at 5° spacing; Check also checks those samples at the selected pose. The current footprint is the configured rectangle ±0.70×±0.38 m plus Nav2 signed-coordinate padding 0.05 m, giving ±0.75×±0.43 m (1.50×0.86 m). Corridor polygons are annotations for visual alignment, not a route or an automatic feasibility constraint. Restricted polygons block intersecting footprints. There is no automatic route optimization, straight-line route test, between-goal collision guarantee, or automatic deployment.

## Coordinates and map identity

For image pixel center `(u,v)`, where integers name centers and `(0,0)` is the top-left center, first compute:

```text
local_x = (u + 0.5) * resolution
local_y = (image_height - v - 0.5) * resolution
map_xy = origin_xy + R(origin_yaw) * [local_x, local_y]
```

The inverse first applies `R(-origin_yaw)` and then reverses that formula. The map origin is the lower-left cell corner, not a pixel center. Canvas image edges use continuous pixel coordinates `-0.5` and `width/height-0.5`. OccupancyGrid indexes are bottom-up: `i=floor(u+0.5)`, `j=floor(height-v-0.5)`. Yaw is counterclockwise from map +x and serialized in radians with quaternion `z=sin(yaw/2), w=cos(yaw/2)`. Yaw is retained at Python/JavaScript double precision; it is not rounded or forcibly normalized. The map's origin yaw can be nonzero and is handled separately from the waypoint yaw. Only planar maps are supported.

Source YAML metadata, dimensions, source paths and SHA256 of the original YAML and image are included in each candidate. The manifest also hashes the selected Nav2 configuration, canonical route and any imported costmaps. Source changes during editing block validation/check/export and require restart. Identical `frame_id: map` strings do not establish identity across fresh SLAM sessions. The inherited canonical positions have **unverified registration** to this saved map and are displayed for inspection; their overlay does not prove registration. A new SLAM session needs independent localization/registration evidence before these coordinates could be used elsewhere. This tool performs no registration.

## Conservative occupancy policy

The source image is displayed unchanged. Only grayscale trinary maps with valid metadata are supported; other modes are rejected. Normally occupancy is `(255-gray)/255` (`gray/255` when negate=1), with strict YAML free/occupied thresholds. Values between the thresholds are unknown. All closed-cell intersections with occupied, unknown, map exterior or restricted polygons block; even a boundary/corner touch counts. Interior cells are checked, not merely footprint vertices or perimeter.

**Saved-map ambiguity discovered in the supplied map:** it contains 525,643 gray-205 pixels, 98,551 gray-254 pixels and 15,806 gray-0 pixels. YAML `free_thresh=0.25` would classify 205 as free (`50/255≈0.196`). Gray 205 is also a conventional saved trinary unknown shade. To avoid silently approving that ambiguous area, the editor conservatively treats gray 205 with `negate=0` as unknown regardless of the free threshold. This is an explicit stricter editor policy, not a claim that it exactly reproduces map_server threshold semantics. It is recorded in bootstrap, UI identity, candidate and manifest, with a regression test. Neither the image nor YAML is modified.

Checks are geometric screening only. They do not reproduce DWB critics, inflation scoring, continuous rotational swept volumes, physical wheel/leg/body envelopes, motion tracking, map uncertainty or dynamic obstacles. Green/`clear: true` never means physical collision-free feasibility. No footprint shrinking or unknown/lethal bypass is offered. Blocked candidates can still be exported for review, with their failed geometry results preserved.

## Optional local/global costmap import

Supply `--local-costmap /tmp/local.json` and/or `--global-costmap /tmp/global.json` at launch. The UI can show their rotated overlays, and checks include them even if their display is disabled. The importer accepts an explicitly registered, normalized offline JSON document:

```json
{
  "frame_id": "map",
  "source_map_image_sha256": "actual image SHA256",
  "source_map_yaml_sha256": "actual YAML SHA256",
  "encoding": "nav2_uint8",
  "width": 2,
  "height": 2,
  "resolution": 0.05,
  "origin": [-1.0, -2.0, 0.0],
  "data": [0, 253, 254, 255]
}
```

`data` is bottom-up row-major, exactly `width*height` integers: 255 unknown, 253 inscribed and 254 lethal all block. Values 0–252 retain their inflated-cost display but do not independently block. Costmap origin/resolution can differ from the occupancy map, including rotated origins; geometry is transformed in map coordinates. Both source map hashes and frame must match. These assertions must come from the capture/registration process: manually attaching today's hash to an unrelated historical costmap does not establish registration. The importer does not auto-convert raw CDR, ROS YAML dumps, a quaternion origin or an odom-frame rolling map. It does not attach to ROS or transform time-dependent snapshots. Input size is bounded to 4 million cells and 32 MB. Metadata and source hashes accompany export; overlays themselves are available in the UI, while annotated PNG shows the original occupancy image and edits.

## Exports and canonical schema

**Export** creates a unique `candidate-*` directory under `/tmp/b2w-waypoint-candidates` (`--output` can choose another dedicated directory below `/tmp`). Existing files are not overwritten. Repository destinations, `/tmp` itself and directories containing sources are rejected. Each directory contains:

- `candidate.yaml`: explicit candidate kind/version, `deployable: false`, exact goals and quaternions, passage markers, polygons, original canonical snapshot, source identity/config hashes, footprint and geometry results.
- `waypoints.csv`: exact x/y/yaw for G1–G5 plus any passage markers.
- `annotations.yaml`: corridor/restricted polygons and passage poses.
- `annotated.png`: full map extent at 3× native pixels, original/candidate footprints, headings, labels, polygons and markers.
- `manifest.yaml`: source/config identities and SHA256 for the other four output files.

The existing parser in `scripts/run_warehouse_route.py` and tests in `tests/test_warehouse_route.py` and `tests/test_warehouse_route_full.py` require canonical schema version 1, G1/G2 `orientation: corridor_heading`, and G3–G5 `orientation: null`; the runner derives headings. Independently edited yaw cannot be silently substituted into that contract. Therefore this editor exports a **separate candidate schema**, preserving canonical names and `position: [x,y]`, plus explicit `yaw_rad`/quaternion and a complete original canonical snapshot. Candidate YAML has no top-level canonical `schema_version` and cannot be consumed as a canonical route. A later reviewed schema/deployment change would be needed to use these poses in navigation.

## Bounded offline verification

```sh
/usr/bin/python3 -m pytest tests/test_waypoint_editor.py -q
node tests/test_waypoint_editor_ui.cjs
node --check scripts/waypoint_editor/app.mjs
# Optional real Chrome integration, installed google-chrome and Node 22+:
node tests/waypoint_editor_browser.cjs
# Existing parser/route regression, without starting any ROS node:
bash -c 'source /opt/ros/humble/setup.bash && /usr/bin/python3 -m pytest tests/test_warehouse_route.py tests/test_warehouse_route_full.py -q'
```

HTTP and browser tests need permission to bind loopback sockets if the execution sandbox blocks sockets. Chrome integration starts only owned test processes, uses ephemeral ports and a temporary profile, exercises dragging, yaw, history, polygons, markers, checks and export, and leaves PNG/export artifacts under the printed temporary directory. `B2W_EDITOR_TEST_ARTIFACTS=/tmp/b2w-editor-stage3` selects an evidence directory. No colcon build is needed for this standalone tool.
