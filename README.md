# Vehicle Tracking

A 2D Qt application for importing DXF layouts, configuring reusable forklift/AMR vehicle profiles, steering them interactively, tracking their movement envelope, and exporting an annotated DXF back alongside the source drawing.

## Run

```powershell
python main.py
```

Vehicle profiles are stored in `vehicles.json` and can be reused across drawings.

Complete multi-floor jobs can be saved as `.vtproject` files. A project contains the floor-to-DXF assignments, start positions, routes and maneuver data, all vehicle profiles, and the active floor/start. DXF references are stored relative to the project file where possible.

## Current Capabilities

- Import DXF geometry for visual reference.
- Render nested blocks, dimensions, bulged polylines, splines, ellipses, text, and other decomposable DXF geometry.
- Render DXF `TEXT`, `MTEXT`, `ATTRIB`, and `ATTDEF` as real Arial text objects, retaining insertion, alignment, rotation, width factor, multiline content, and character height in drawing units instead of flattening characters into placeholder rectangles.
- Lazy-load a floor DXF only when that floor becomes active, cache it for later switches, and show phased modal progress while reading, scanning, measuring, and converting large drawings. Floor assignments are not parsed while the management dialog is open.
- Pan across an expanded canvas, zoom with the mouse wheel, and use **Fit DXF** to return to the full drawing.
- Place persistent start and end positions directly on the DXF; placing the start resets the vehicle path there.
- Orient start and end poses by dragging in the required facing direction, or enter exact heading angles afterward.
- Drag the green start or red end handle to fine-tune its final DXF position; the vehicle previews and indicative path update live.
- Preview an orange indicative path: a steering-model projection without an end pose, or a heading-constrained connection to an oriented end pose.
- Generate a persistent planned route from the defined start pose to the finish pose, with a visible centreline and vehicle-width swept envelope that can be shown or hidden.
- Configure named building levels and multiple reusable start positions; each saved path retains its own level, named start, and exact start pose.
- Assign a separate DXF to every floor through the **Manage Floor DXFs** dialog; add/remove floors or browse and replace assignments in one table, and automatically load the correct drawing when changing floor.
- Use the top ribbon for floor drawings, start/end placement, path editing, playback, and exports, while the sidebar remains focused on vehicle configuration, floor/start selection, route status, and steering controls.
- Follow the operating-system light or dark appearance, including ribbon gaps, scroll areas, dialogs, fields, tables, menus, canvas, semantic status colours, disabled controls, and themed button icons; changes to the OS colour scheme are applied while the app is running.
- Use **Settings** on the Home ribbon to select System, Light, or Dark mode and optionally override the DXF canvas background colour; appearance preferences persist between launches.
- Open and save complete `.vtproject` files from the Home ribbon. Legacy DXFs can still be opened directly to start a new project.
- Insert draggable orange control points on the planned route to tighten or reshape individual sections; selected points can be removed or all points cleared.
- Draw the before-drop-off approach and after-drop-off exit independently: each section has its own **Draw Lines** action, uses isolated two-click CAD lines with polar snapping, and replaces only that section when its intersections and tangent fillets are applied.
- Drag the paired blue curve handlebars on every ordinary route point to control tangent direction and strength for both adjoining curve sections; custom tangents persist with routes and projects.
- Right-click a route point to enable a purple driven-wheel point turn. The planner rotates the vehicle at that position using the configured steering angle and checks that the profile has driven steerable wheels (or differential drive).
- Use **Place Reverse Action** and click the planned route to add a red gear-change point. Travel after the point is reversed; placing another reversing action changes back to forward travel.
- Treat route points as a traversal-ordered operation list: Path start, each route point in order, an optional payload drop-off/reverse pair, and the final position. Operations persist in routes and project files while remaining compatible with earlier point-turn and reversing-action data.
- Configure each ordinary route point as a **Straight section** or **Curved turn**. Straight sections use exact linear interpolation with constant motion heading; curved turns use the steering-constrained Hermite path and expose tangent handlebars.
- Place a dedicated headed **Drop-off Point** before the final position. The route approaches it inline, releases the payload, automatically reverses out, and finishes at the existing path endpoint without carrying the payload away.
- Mark a route endpoint as **Pick up payload** to match it against a saved drop-off point on the same floor. The checker validates payload-centre coincidence, vehicle/payload alignment within 2 degrees, and a straight inline final approach at least as long as the larger of vehicle or payload length.
- Animate an oriented vehicle preview from start to finish along the exact edited route used by the display and DXF export.
- Pause/resume route playback and drag the timeline slider to inspect any animation position.
- Export the selected path animation to a 1280x720 H.264 MP4.
- Check every planned-route section against the configured steering curvature and overlay impossible sections in red, with the available minimum radius shown in the editor.
- When a forward-only finish alignment is impossible but reversing the final approach is feasible, suggest using the final route point as a one-movement realignment position.
- Mark the selected block's oriented extremity corners and trace all four corners along the planned route.
- Generate a landscape PDF route report covering possible and impossible paths, level/start assignment, start/end coordinates, tracked distance, impossible-section count, and a tracking-centreline graphic.
- Configure a rigidly attached payload by vehicle-relative centre X/Y, length, width, and rotation; render it on live, endpoint, and animated vehicles.
- Show the payload as a dashed oriented bounding box with a centre cross and four visible corner markers above the vehicle block.
- Track the payload centre and all four payload extremities along both driven and planned routes.
- Preview a selected vehicle block, place or move wheel centres on it, and retain wheel coordinates relative to the block's DXF insertion point `(0, 0)`.
- Draw and save the selected block's forward travel direction in the wheel editor; wheel coordinates, block placement, movement, and export use that same vehicle axis.
- Show directional wheel rectangles in the editor, with red steerable wheels, blue fixed wheels, and solid fill for driven wheels.
- Render the selected vehicle block at exact imported-DXF coordinates and include its configured wheels in tracking exports.
- Detect DXF block inserts and use a block name as the vehicle symbol when exporting.
- Configure vehicle dimensions, wheel positions, steering type, steering angle, turning radius, speed, and pose spacing.
- Calculate and apply the theoretical minimum centre-path turning radius for each steering method: front/rear Ackermann use `wheelbase / tan(angle)`, equal-and-opposite four-wheel steering uses `wheelbase / (2 x tan(angle))`, and differential/omni running gear can reorient with a zero centre-path radius. The planner uses the larger of this value and any configured safety radius.
- Interactively steer with the toolbar, sliders, or keyboard:
  - `W` / `S`: forward / reverse
  - `A` / `D`: steer left / right
  - `Space`: stop
  - `R`: reset path
- Scale driving speed to the configured vehicle length so steering remains visible in metre- and millimetre-based DXFs.
- Show a green forward or red reverse arrow directly on the live vehicle, with matching direction text in the controls.
- Save multiple vehicle profiles to JSON.
- Export a sibling DXF containing:
  - one selectable DXF group per named planned path, plus a separate `VT_DRIVEN_PATH` group, while retaining the shared layer structure
  - red circle-and-cross reversing-action markers on `VT_ROUTE_ACTIONS`, contained in the applicable path group
  - application-generated geometry only; imported model-space/background entities are excluded
  - source DXF units and insertion-base metadata, with all generated geometry kept at its original absolute model-space coordinates
  - only the selected vehicle block definition when required by generated vehicle pose references
  - vehicle center path
  - swept envelope extremities
  - pose copies using the configured DXF block when available
  - planned start-to-finish route on `VT_PLANNED_ROUTE`
  - planned swept boundaries on `VT_PLANNED_SWEEP`
  - selected-block start/finish outlines and four extremity traces on `VT_BLOCK_OUTLINE`
  - driven payload centre, corner traces, and endpoint footprints on `VT_PAYLOAD_PATH`
  - planned payload centre, corner traces, and endpoint footprints on `VT_PLANNED_PAYLOAD`
