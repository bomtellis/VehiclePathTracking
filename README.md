# Vehicle Tracking

A 2D Qt application for importing DXF layouts, configuring reusable forklift/AMR vehicle profiles, steering them interactively, tracking their movement envelope, and exporting an annotated DXF back alongside the source drawing.

## Run

```powershell
python main.py
```

Vehicle profiles are stored in `vehicles.json` and can be reused across drawings.

## Current Capabilities

- Import DXF geometry for visual reference.
- Render nested blocks, dimensions, bulged polylines, splines, ellipses, text, and other decomposable DXF geometry.
- Pan across an expanded canvas, zoom with the mouse wheel, and use **Fit DXF** to return to the full drawing.
- Place persistent start and end positions directly on the DXF; placing the start resets the vehicle path there.
- Orient start and end poses by dragging in the required facing direction, or enter exact heading angles afterward.
- Drag the green start or red end handle to fine-tune its final DXF position; the vehicle previews and indicative path update live.
- Preview an orange indicative path: a steering-model projection without an end pose, or a heading-constrained connection to an oriented end pose.
- Generate a persistent planned route from the defined start pose to the finish pose, with a visible centreline and vehicle-width swept envelope that can be shown or hidden.
- Insert draggable orange control points on the planned route to tighten or reshape individual sections; selected points can be removed or all points cleared.
- Animate an oriented vehicle preview from start to finish along the exact edited route used by the display and DXF export.
- Check every planned-route section against the configured steering curvature and overlay impossible sections in red, with the available minimum radius shown in the editor.
- Mark the selected block's oriented extremity corners and trace all four corners along the planned route.
- Configure a rigidly attached payload by vehicle-relative centre X/Y, length, width, and rotation; render it on live, endpoint, and animated vehicles.
- Show the payload as a dashed oriented bounding box with a centre cross and four visible corner markers above the vehicle block.
- Track the payload centre and all four payload extremities along both driven and planned routes.
- Preview a selected vehicle block, place or move wheel centres on it, and retain wheel coordinates relative to the block's DXF insertion point `(0, 0)`.
- Draw and save the selected block's forward travel direction in the wheel editor; wheel coordinates, block placement, movement, and export use that same vehicle axis.
- Show directional wheel rectangles in the editor, with red steerable wheels, blue fixed wheels, and solid fill for driven wheels.
- Render the selected vehicle block at exact imported-DXF coordinates and include its configured wheels in tracking exports.
- Detect DXF block inserts and use a block name as the vehicle symbol when exporting.
- Configure vehicle dimensions, wheel positions, steering type, steering angle, turning radius, speed, and pose spacing.
- Interactively steer with the toolbar, sliders, or keyboard:
  - `W` / `S`: forward / reverse
  - `A` / `D`: steer left / right
  - `Space`: stop
  - `R`: reset path
- Scale driving speed to the configured vehicle length so steering remains visible in metre- and millimetre-based DXFs.
- Show a green forward or red reverse arrow directly on the live vehicle, with matching direction text in the controls.
- Save multiple vehicle profiles to JSON.
- Export a sibling DXF containing:
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
