# Graphics
Rana Gargawi and Shahar Metodi
# mini — 3D Terrain Viewer, Feature Picking & ePnP Pose Estimation

This program renders a 3D terrain mesh from a grayscale heightmap (OpenCV) using legacy OpenGL/GLUT, lets you **capture screenshots** from a “main” camera, **extract and pick ORB features** for 10 screenshots (pre-stage), and later (run-stage) **match new features to stored 3D points** and estimate the camera pose with **solvePnPRansac (EPNP)**. A synchronized **global view** window shows the terrain, camera path, and pose markers.

---

## Build & Run

### Dependencies

* C++17
* OpenGL + GLUT (e.g., freeglut)
* OpenCV (core, imgproc, features2d, calib3d, highgui)
* GLM
* CMake (recommended)

### Example (Linux)

```bash
sudo apt-get install build-essential freeglut3-dev libopencv-dev
# GLM is header-only; install via your package manager if needed

mkdir -p build && cd build
cmake ..    # or compile with your IDE/toolchain
make -j
./mini path/to/heightmap.png
```

### Windows (MSVC)

* Add/include: OpenCV include/lib dirs, GLUT/freeglut, GLM include dir.
* Enable C++17.
* Add linker inputs: OpenCV libs and GLUT/freeglut libs.

---

## How it Works (High-Level)

* **Heightmap → Mesh**: The grayscale image is smoothed/equalized, then converted into a grid of vertices. Triangles and per-vertex normals are generated.
* **Main view**: Free-fly camera (W/A/S/D + Q/E, arrows for yaw/pitch) renders the terrain with a blue→green→yellow→red height colormap.
* **Global view**: A fixed overhead camera renders the terrain, the main camera path, and markers for screenshots and EPNP-computed poses.
* **Pre-stage**: Each press of **`B`** captures a screenshot, extracts top-`pp` ORB features (spatially de-clustered), and displays them. You then **pick triangles** in the global view to associate each 2D feature with a 3D point (triangle centroid). After 10 screenshots, features + descriptors + 3D points are saved to `picked_points/img_<i>_features.yml`.
* **Run-stage**: Press **`B`** to capture a new screenshot; the code matches its features to the stored sets, chooses the best matching reference image, and runs **`solvePnPRansac(EPNP)`** to compute pose. The global view shows the estimated pose (red triangle) and a red polyline through poses.

> **Modes:** Controlled by the global boolean `preStage`.
>
> * `preStage = true` → collecting datasets (10 screenshots)
> * `preStage = false` → running pose estimation with the saved datasets
>   Set it at the top of the file to the mode you want before building.

---

## Controls

* **Mouse (global view)**: Left-click to pick a triangle ID (encoded color picking) during **pre-stage**.
* **Keyboard (main view window active)**:

  * `W/A/S/D` — strafe/forward/back
  * `Q/E` — move up/down
  * **Arrow keys** — yaw/pitch
  * **`B`** — take a screenshot

    * Pre-stage: extract & display ORB features (top `pp`), then pick corresponding 3D points in the global view (one pick per feature, order matters).
    * Run-stage: match, estimate pose via PnP, show reprojection debug, draw red pose markers/line in global view.
  * **`Esc`** — exit (when not showing an OpenCV window)

---

## Files & Outputs

* **Input:** `argv[1]` is a grayscale heightmap image (e.g., PNG).
* **Output dir:** `picked_points/` (created automatically).
* **Per-image dataset (pre-stage):**
  `picked_points/img_<1..10>_features.yml` contains:

  * `features`: array of `{x,y,size,angle,response,octave,class_id,pt3d_x,pt3d_y,pt3d_z}`
  * `descriptors`: ORB descriptors matrix

---

## Global Data Structures (key ones)

* `heightMap` — `cv::Mat` grayscale image.
* `vertices`, `indices`, `normals` — terrain mesh.
* `camX, camY, camZ, yaw, pitch` — main camera pose.
* `cameraTrack` — `std::vector<glm::vec3>` of the main camera positions (drawn in white).
* `screenshots`, `screenshotPositions` — captured frames + (x,y,z,yaw) when pressing `B`.
* **Pre-stage storage (10 images):**

  * `imgsFeatures[i]` — vector of top ORB keypoints for image *i*
  * `imgsDescriptors[i]` — corresponding ORB descriptors (rows aligned with `imgsFeatures[i]`)
  * `pickedGlobalIDs[i]` — triangle IDs picked in **global** view for image *i* (one per feature)
  * `pickedPoints3D[i]` — 3D points (triangle centroids), one per picked feature
  * `realPickedPoints3D[i]` — ground-truth 3D for displayed features (from depth unprojection; used for debugging)
* **Run-stage matching:**

  * `matched2dlocations` — 2D points from the new screenshot (selected & matched)
  * `matched3dlocations` — corresponding 3D points (from stored datasets)
  * `ePnPPositions` — vector of `(x,y,z,yaw)` pose estimates (drawn in red)

---

## Function-by-Function Documentation

### `buildMesh(float heightScale = 2.0f)`

Generates the terrain mesh from the global `heightMap`.

* **Inputs:** reads `heightMap` (grayscale); converts each pixel to a vertex `(x, z, y)` where `z` scales intensity (0.2f in code).
* **Outputs:** fills `vertices` (grid), `indices` (two triangles per cell), and `normals` (per-vertex normals accumulated from face normals).
* **Notes:** Colors are not set here—just geometry. Normals are normalized at the end.

### `display()`

Renders the **main view** window.

* Sets viewport & clears buffers; builds a look-at from `camX/Y/Z, yaw, pitch`.
* Enables basic lighting; draws the terrain as triangles:

  * Per-vertex color uses a height colormap: blue (low) → green → yellow → red (high).
* **Pre-stage overlay:** draws large colored points at 2D feature locations of the **latest** screenshot in screen-space (for visual ordering & picking guidance).
* Updates `cameraTrack` with the current camera position if moved sufficiently.
* Swaps buffers.

### `ensureOutputDir(const std::string& dir) -> bool`

Ensures an output directory exists (creates it if needed) using `std::filesystem`.

* Returns `true` on success (exists or created), `false` otherwise.

### `savePickedDataForAllImages()`

Writes **10 yml files** (one per screenshot) to `picked_points/`:

* For each image `imgIdx`:

  * Saves `features` array with 2D metadata and the associated **3D centroid** `(pt3d_x/y/z)`.
  * Saves ORB `descriptors` matrix aligned with features.
* Creates output directory if missing (`ensureOutputDir`).
* **Called automatically** after the last feature of the 10th image is picked in the **global** view; then switches out of `preStage`.

### `displayGlobalView()`

Renders the **global** window.

* Fixed projection and camera (`gluLookAt(800,850,600 → 602,460,532)`).
* Draws terrain with the same colormap.
* **Pre-stage:** draws colored points at the **3D triangle centroids** corresponding to the features picked so far for the current image.

  * When the last feature is picked for an image:

    * If there are <10 images collected, initializes containers for the next image.
    * If that was the 10th image, calls `savePickedDataForAllImages()` and toggles to run-stage.
* **Run-stage:**

  * Draws the **camera path** (`cameraTrack`) as a **white** line strip.
  * Draws **white triangles** at each `screenshotPositions` (actual capture poses).
  * Draws **red triangles** at each **ePnP** pose in `ePnPPositions`, and a **red** line strip through them.

### `reshape(int w, int h)`

Main window reshape callback. Updates viewport, projection (45° perspective), and switches back to modelview.

### `renderForPicking()`

Main-view **ID buffer** rendering for picking (not used directly in the current left-click branch).

* Renders each triangle with a unique RGB color encoding its ID (i/3).
* Used to decode which triangle is under a pixel via `glReadPixels`.

### `renderForPickingGlobal()`

**Global-view** ID buffer rendering used by the mouse picking handler.

* Same idea as above: encodes triangle IDs into color so a single pixel read yields the triangle index at the cursor.

### `mouseClick(int button, int state, int x, int y)`

Handles **left-click picking in the global window** during **pre-stage**.

* Renders the ID buffer (`renderForPickingGlobal`), reads back the pixel at `(x,y)` and decodes the triangle ID.
* Appends the ID to `pickedGlobalIDs.back()` if valid and in the correct order.
* **Associates a 3D point**: computes the triangle centroid and appends to `pickedPoints3D.back()`.
* When the first feature of the very first image is picked, initializes the per-image vectors.
* Triggers redraws of both windows.
* (End-of-image / end-of-dataset transitions are handled in `displayGlobalView()`.)

### `keyboard(unsigned char key, int x, int y)`

Main keyboard handler (active window set to main view).

* **Movement**: `W/A/S/D` (planar), `Q/E` (vertical).
* **Screenshot `B`**:

  * Reads current frame via `glReadPixels`, flips, converts RGB→BGR, stores in `screenshots` and logs `(x,y,z,yaw)` to `screenshotPositions`.
  * **Pre-stage path**:

    * Extracts ORB on the captured image; sorts by response; de-clusters by a min distance; keeps top `pp` (default `20`).
    * Visualizes keypoints; also **unprojects** each feature via `gluUnProject` (using the depth buffer) to get **debug** 3D points → `realPickedPoints3D`.
    * You then switch to the **global view** and click triangles in the same order to associate 3D centroids with these features (stored in `pickedPoints3D`).
  * **Run-stage path**:

    * Extracts & de-clusters ORB features; computes descriptors.
    * Matches against each of the 10 reference descriptor sets; counts **ratio-test** inliers per reference image and **chooses the best**.
    * Builds **2D–3D correspondences** (`matched2dlocations`, `matched3dlocations`) from the chosen reference only, with train-index de-duplication and sanity bounds on 3D.
    * If ≥4 matches: computes camera pose with `cv::solvePnPRansac(..., SOLVEPNP_EPNP)` using a camera matrix consistent with the OpenGL projection (`fovy=45°`, square pixels, principal point at center).
    * **Debug view**: shows red observed 2D points and green reprojected 3D points with connecting lines.
    * Converts `rvec` to `R`, computes camera **position** as `-Rᵀ tvec`, derives yaw, and appends `(x, -y, z, yaw)` to `ePnPPositions`.
    * Clears the temp match buffers for the next capture.
* **`Esc`**: exits (when not in a blocking OpenCV window).

### `specialKeys(int key, int x, int y)`

Arrow-key handler:

* `LEFT/RIGHT` adjust **yaw** (`turnSpeed` degrees per press).
* `UP/DOWN` adjust **pitch** (clamped to `[-89°, +89°]`).
* Requests redraw.

### `main(int argc, char** argv) -> int`

Program entry point.

1. Loads the heightmap (`argv[1]`) as grayscale. Flips vertically, equalizes histogram, applies a 3×3 Gaussian blur.
2. **If run-stage (`preStage == false`)**: loads the 10 `picked_points/img_<i>_features.yml` files:

   * Rebuilds `imgsFeatures[i]`, `pickedPoints3D[i]`, and `imgsDescriptors[i]` from disk.
3. Creates the main and global GLUT windows, sets callbacks:

   * **Main**: `display`, `reshape`, `mouseClick`, `keyboard`, `specialKeys`
   * **Global**: `displayGlobalView`, `mouseClick`, and an `idle` that posts redisplay requests.
4. Calls `buildMesh()` and starts the GLUT main loop.

---

## Key Types & Helpers

### `struct MyVec3f`

Lightweight 3D vector used for geometry and normals.

* **Fields:** `x, y, z`
* **Ops:** `operator-`, `operator+`, `cross(const MyVec3f&)`, `normalize()`
* Used to accumulate and normalize per-vertex normals.

---

## Tips & Gotchas

* **Mode selection:** Set `preStage` to `true` when you want to **build** your 10-image dataset first. Then set it to `false` to enter **run-stage** and consume the saved files.
* **Order matters:** In pre-stage, you pick triangle IDs in the **same order** as the displayed features (the code draws them in a distinct color sequence to help).
* **Coordinate frames:** OpenGL uses `(x, y, z)`. For PnP preparation, the code **flips** 2D vertically and flips the **sign of Y** in 3D to match the assumed camera model before calling OpenCV.
* **Colormap:** The color mapping is tied to `v.y / 51.0f`; adjust if you change the height scale.
* **Performance:** This uses immediate-mode OpenGL and per-frame CPU transforms; adequate for small meshes but not optimized.

---
