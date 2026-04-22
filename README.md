# Valhalla Spike

---

## 0. Executive summary

This document covers three areas:

1. **Data model** - why the map is sliced into three tile levels and how that governs which parts of the graph the router inspects.
2. **Path-finding** - A → Bidirectional A → Hierarchical Bidirectional, with the decision logic that selects between them per request.
3. **Live traffic blending** - the 1-hour fade that weights live speed against historical speed, plus an open upstream bug ([issue #5616](https://github.com/valhalla/valhalla/issues/5616), opened October 2025) that causes the reverse frontier of Bidirectional A to apply un-faded live traffic.

A concrete exposure analysis for our production request shape is provided in §5.11 and §5.12.

---

## 1. The Map Is a Three-Layer Filing Cabinet

Valhalla does not store "a road network." It stores **three separate road networks, stacked**.


| Level            | Tile size     | Size at equator  | Road classes inside         | Shortcut edges? |
| ---------------- | ------------- | ---------------- | --------------------------- | --------------- |
| **0 - Highway**  | 4° × 4°       | ~440 km × 440 km | Motorway, Trunk, Primary    | Yes             |
| **1 - Arterial** | 1° × 1°       | ~110 km × 110 km | Secondary, Tertiary         | Yes             |
| **2 - Local**    | 0.25° × 0.25° | ~28 km × 28 km   | Residential, Service, Paths | No              |


**Source of truth:** [src/baldr/tilehierarchy.cc#L14-L30](https://github.com/valhalla/valhalla/blob/master/src/baldr/tilehierarchy.cc#L14-L30)

```cpp
TileLevel{0, stringToRoadClass("Primary"),      "highway",  Tiles{..., 4}},
TileLevel{1, stringToRoadClass("Tertiary"),     "arterial", Tiles{..., 1}},
TileLevel{2, stringToRoadClass("ServiceOther"), "local",    Tiles{..., .25}},
```

**Common myth to correct:** "13 L0 tiles, 63 L1, 658 L2." Those are our **Vietnam extract**, not the globe. The Earth is 180° tall (latitude) and 360° wide (longitude). The math calculates the total grid cells for the entire planet:

- **L0 (4° tiles):** (180 / 4) × (360 / 4) = 45 × 90 = 4,050 tiles
- **L1 (1° tiles):** (180 / 1) × (360 / 1) = 180 × 360 = 64,800 tiles
- **L2 (0.25° tiles):** (180 / 0.25) × (360 / 0.25) = 720 × 1,440 = 1,036,800 tiles (~1M)

### Why three levels exist

This isn't just about organizing data into categories. It is a **speed optimization** to help the routing engine calculate directions much faster by ignoring small roads when they aren't needed.

They are **Tile Levels** based on road importance.

- **Level 0 (L0):** Highways and major inter-city roads.
- **Level 1 (L1):** Arterial roads (main city streets).
- **Level 2 (L2):** Local roads (alleys, neighborhood streets).

**Example: District 1 to Tan Binh (Ho Chi Minh City)**

- **Without levels (Flat Map):** The routing algorithm acts like water spilling out from District 1. It has to explore *every single tiny alley (hẻm)*, dead-end, and neighborhood street between Q1 and Tan Binh just to find the way. This wastes massive CPU power and time.
- **With 3 levels:** The algorithm starts in the alleys of Q1 (Level 2). Once it finds a main road like Cach Mang Thang 8 or Nam Ky Khoi Nghia (Level 1), it "promotes" to Level 1. From that point on, it completely **ignores** all the thousands of tiny alleys along the way. It only drops back down to Level 2 when it gets very close to the destination in Tan Binh.

#### The "Promote and Drop Down" Mechanism (Unidirectional A)

This logic is centralized in [valhalla/sif/hierarchylimits.h#L44-L47](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h#L44-L47).

```cpp
inline bool StopExpanding(const HierarchyLimits& hl, const float dist) {
  return (hl.up_transition_count() > hl.max_up_transitions() &&
          dist > hl.expand_within_dist());
}
```

**Notes:** "If we have taken too many upward transitions already AND we are still far from destination, stop expanding this level." The AND is the safety net - near the endpoint we always re-allow local streets so we can reach the actual drop-off address.

**How it is used in the A Loop:**
This function is evaluated at *every single intersection* for *every connected road*. From [src/thor/unidirectional_astar.cc#L598](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc#L598):

```cpp
// dist2dest is the straight-line distance to the destination
// If StopExpanding returns TRUE, we skip (prune) this road.
// If StopExpanding returns FALSE, we are allowed to drive on it!
if (StopExpanding(hierarchy_limits_[pred.endnode().level()], dist2dest)) {
    continue; // This means "skip this edge and look at the next one"
}
```

**Step-by-Step Execution of the "Drop Down" Rule (Q1 to Tan Binh):**

*(Note: The total distance from Q1 to Tan Binh is ~7.5 km. Because 7.5 km is already much smaller than the 100 km limit for Level 1, the algorithm is never blocked from using Level 1. Level 1 is open the entire time. The drop down we are waiting for below is the drop from Level 1 into the Level 2 alleys).*


| Step             | Current Location          | `dist2dest` | `up_transitions` | Edge Being Evaluated     | Calculation                                                                               | Result / Action                                                                                                                     |
| ---------------- | ------------------------- | ----------- | ---------------- | ------------------------ | ----------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| **1. Start**     | Deep in Q1 Alley          | **7,500m**  | 12               | **Alley (Level 2)**      | `(12 > 100)` is **FALSE** `(7500 > 5000)` is **TRUE** ➔ `FALSE AND TRUE` = **FALSE**      | **Continue discover this level.** Searches local streets to find a way out.                                                         |
| **2. Promote**   | Reach CMT8                | **7,000m**  | 101              | **Main Road (Level 1)**  | `(101 > 400)` is **FALSE** `(7000 > 100000)` is **FALSE** ➔ `FALSE AND FALSE` = **FALSE** | **Continue discover this level.** L1 is open. The search moves onto the arterial.                                                   |
| **3. Bypass**    | Cruising CMT8 (Phu Nhuan) | **6,000m**  | 101              | **Side Alley (Level 2)** | `(101 > 100)` is **TRUE** `(6000 > 5000)` is **TRUE** ➔ `TRUE AND TRUE` = **TRUE**        | **Prune this edge.** At every intersection, it looks at the side alleys, checks the L2 limits, and skips them.                      |
| **4. Continue**  | Cruising CMT8 (Phu Nhuan) | **6,000m**  | 101              | **Main Road (Level 1)**  | `(101 > 400)` is **FALSE** `(6000 > 100000)` is **FALSE** ➔ `FALSE AND FALSE` = **FALSE** | **Continue discover this level.** At the exact same intersection, it checks the L1 limits for continuing straight, and keeps going. |
| **5. Drop Down** | Cross into Tan Binh       | **4,999m**  | 101              | **Side Alley (Level 2)** | `(101 > 100)` is **TRUE** `(4999 > 5000)` is **FALSE** ➔ `TRUE AND FALSE` = **FALSE**     | **Continue discover this level.** The 5km safety net triggers. The side alleys are finally unlocked!                                |
| **6. Arrive**    | Turn into Tan Binh Alley  | **800m**    | 101              | **Alley (Level 2)**      | `(101 > 100)` is **TRUE** `(800 > 5000)` is **FALSE** ➔ `TRUE AND FALSE` = **FALSE**      | **Continue discover this level.** Uses Level 2 alleys to reach the exact destination.                                               |


**What about Level 0 (Highways) in Ho Chi Minh City?**

You might notice Level 0 has an `expand_within_dist` of `∞` (Infinity). This means Level 0 is *never* pruned by distance. Once the algorithm finds a Level 0 road, it stays on it.

If the algorithm is driving on a Level 0 road, does it still check Level 1 roads? **Yes, but only when it gets close to the destination!** If the car is on a Level 0 highway 200 km away, it will check the Level 1 off-ramp, see that `dist2dest > 100km` is TRUE, and prune it. But once the car is 90 km away, the Level 1 check (`90km > 100km`) becomes FALSE, the safety net triggers, and it starts discovering Level 1 roads to drop down off the highway.

In Valhalla, Level 0 includes OSM classes `motorway`, `trunk`, and `primary`. For Ho Chi Minh City, this means if your route hits any of the following major arteries, it promotes to Level 0:

- **Motorways (Cao tốc):** TP.HCM - Long Thành - Dầu Giây, TP.HCM - Trung Lương, Bến Lức - Long Thành.
- **Trunks (Đại lộ / Quốc lộ):** Phạm Văn Đồng, Võ Văn Kiệt, Mai Chí Thọ, Nguyễn Văn Linh, Xa lộ Hà Nội (Võ Nguyên Giáp), QL1A, QL13, QL22.
- **Primary streets:** Trường Chinh, Cộng Hòa, Điện Biên Phủ, Nguyễn Hữu Cảnh, Huỳnh Tấn Phát, Tôn Đức Thắng, Nguyễn Tất Thành.

*(Wait, if Motorways and Primary streets are both on Level 0, how does the router choose between them? While they share the same **Tile Level** for memory loading, they keep their distinct **Road Class** in the graph. The router distinguishes them using two things: **Speed** (Cao tốc is much faster) and **Costing Factors** (like `use_highways` and `use_tolls`). If a user requests a toll-free route, the algorithm will heavily penalize the Motorway and choose the Primary street instead, even though both are open on Level 0).*

In our Q1 to Tan Binh example, the route primarily uses Cách Mạng Tháng 8 (which is `secondary` or `tertiary` in OSM, mapping to Level 1). But if the algorithm routed via Điện Biên Phủ or Trường Chinh, it would promote to Level 0 and cruise on infinity.

**Default Limits for Both Algorithms**

From [valhalla/sif/hierarchylimits.h#L21-L29](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h#L21-L29):


| Level         | `max_up_transitions` | `expand_within_dist` (unidir) | `expand_within_dist` (bidir) |
| ------------- | -------------------- | ----------------------------- | ---------------------------- |
| L0 (Highway)  | 0 (cannot go up)     | ∞ (always expand)             | ∞                            |
| L1 (Arterial) | 400                  | 100 km                        | 20 km                        |
| L2 (Local)    | 100                  | 5 km                          | 5 km                         |


**Key insight:** Bidirectional shrinks the "always expand arterial" zone from 100 km down to 20 km. That is a big part of why bidirectional is faster - we bail out of arterials much sooner. Unidirectional keeps arterial detail (L1) for 100 km out from the origin, because it has no opposing frontier to meet.

**Analogy for product people:** Think of Google search. When you type a query, Google does not scan every webpage - it hits an index that already collapsed 100 similar pages into one entry. L0 shortcuts are that index, for highways.

### `.gph` tile = flat binary blob

A single tile file is a fixed-layout binary, not a database. Layout (same order on disk):

```
GraphTileHeader → NodeInfo[] → DirectedEdge[] → EdgeInfo[] → Signs → Restrictions → Admins → …
```

You `mmap` the file, cast pointers to structs, and index by offset. **Zero parsing cost** - this is how Valhalla loads Earth-scale data in milliseconds. The `mmap` approach also means multiple worker processes on the same machine share the same physical memory pages: the OS pays for the map once, every worker gets zero-copy access.

### Real example: 28bis Mạc Đĩnh Chi, Q1, HCMC

What does that binary blob actually contain for a real Vietnamese address? We `POST /locate` with `{lat: 10.7889, lon: 106.7005}` against the running tileset. Every field below is returned verbatim by Valhalla - no invention, no rounding.

The point snaps to a 40 m alley behind the building - `Hẻm 16 Đinh Tiên Hoàng` - which is the physical access road for 28bis. Two `DirectedEdge` records are returned, because every OSM way is stored as a forward/backward pair.

```
INPUT              lat=10.7889  lon=106.7005
SNAP               correlated=(10.788679, 106.700722)   distance=34.6 m   side=right
                   → percent_along = 17.3% of the way from edge start

TILE FILE          2/000/581/466.gph        ← Level 2 (0.25° × 0.25°)
                   tile_id = 581466         covers a 28 km × 28 km box near HCMC

GRAPH-ID (edge)    value = 2,809,483,688,658  (64-bit)
                   = (id << 25) | (tile_id << 3) | level
                   = (83729 << 25) | (581466 << 3) | 2
                   → level=2, tile=581466, id=83729
                   → fetch: edges_base + 83729 * sizeof(DirectedEdge)   (pure pointer math)

EDGE-INFO          way_id   = 605149488              (original OSM way)
                   names    = ["Hẻm 16 Đinh Tiên Hoàng"]
                   shape    = "kfnqSwlnojEwNeO"     (3-vertex encoded polyline)
                   speed_limit = 0                   (not tagged in OSM)

CLASSIFICATION     classification = service_other
                   use            = alley
                   surface        = paved_smooth
                   link = false, internal = false

GEO-ATTRIBUTES     length       = 40 m
                   weighted_grade / max_up / max_down = 0.0   (flat)
                   curvature    = 0

END-NODE           value = 1,340,806,200,018
                   level=2, tile=581466, id=39959
                   → same packing formula, different slot in NodeInfo[]

SPEEDS             type             = classified        ← no live/predicted profile
                   default          = 20 km/h           ← alley default
                   free_flow        = 0  (not built)
                   constrained_flow = 0  (not built)
                   predicted        = false

LIVE-SPEED         overall_speed = 20 km/h
                   speed_0/1/2   = 20 / 20 / 20
                   congestion_0/1/2 = 0.62 / 0.62 / 0.62   (~62% red)
                   breakpoint_0/1   = 1.0                  (one segment, no split)
                   ← loaded from a SEPARATE traffic.tar, keyed by the same edge_id

ACCESS (bit mask)  car, truck, motorcycle, moped, bicycle, bus, taxi,
                   pedestrian, wheelchair, HOV = true
                   emergency = false

FLAGS              destination_only      = true     ← "only enter if you live here"
                   destination_only_hgv  = true
                   forward               = false    ← this is the REVERSE half of the way
                   lane_count            = 1
                   toll / bridge / tunnel / round_about / traffic_signal = false
                   sidewalk_left / sidewalk_right = false

REACH              outbound_reach = 50    inbound_reach = 50
                   → Loki uses these to decide if the candidate is "connected enough"
                     to actually start/end a route from.
```

The second edge in the same response is the twin: `edge_id.id = 83734`, `forward = true`, `live_speed = 16 km/h` (the opposite direction through the alley is slower right now).

#### What a student should take away


| Observation                                            | Why it matters                                                                                                                                                                                                                      |
| ------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **One building → 1 OSM way → 2 DirectedEdges**         | Forward/backward halves store turn restrictions, speeds, and access flags independently. That is why one-way streets and split lane counts work without any "direction" field in the routing code.                                  |
| `**2/000/581/466.gph` is one file**                    | It is a fixed-layout binary. Finding edge 83729 is literally `mmap_base + header.edge_offset + 83729 * sizeof(DirectedEdge)`. Zero SQL, zero parse, zero allocation per lookup.                                                     |
| **GraphId is reversible arithmetic**                   | Given the 64-bit value, anyone can recover `(level, tile_id, id)` with three bit-ops. No lookup table, no index. That is why `GetGraphTile(id)` is O(1) and safe across threads.                                                    |
| `**live_speed` is an overlay**                         | The `.gph` file never changes during the day. The traffic tile (`traffic.tar`) is a parallel byte array indexed by the same edge slot - update traffic, pointer lands in the same offset, no re-link.                               |
| `**destination_only = true` on an alley**              | Valhalla will only cross this edge if the origin or destination is on it (or after a second-pass fallback). Great for privacy of residential lanes, occasionally hostile to real Saigon shortcuts - see §11 for the Vietnam tuning. |
| `**speed_limit = 0`, `default = 20`, `free_flow = 0`** | OSM did not tag this alley, so the router falls back to the classified default. No predicted profile was built either. In HCMC peak hour, 20 km/h through a 1-lane hẻm is optimistic - another knob §11 discusses.                  |


#### Contrast: the same query 60 m west hits Level 0

Same neighbourhood, coordinate `(10.78796, 106.70022)` - now on Đinh Tiên Hoàng itself:

```
TILE FILE     0/002/321.gph            ← Level 0 (highway tier), 4° × 4° tile
GRAPH-ID      value = 3,277,596,936,328   → level=0, tile=2321, id=97680
WAY-ID        231983460    names = ["Đinh Tiên Hoàng"]
CLASSIFY      classification=primary   use=road   surface=paved_smooth
SPEEDS        type=tagged  default=60  speed_limit=60   ← OSM tagged
LANES         lane_count=3   sidewalk_left=true  sidewalk_right=true
FLAGS         has_sign=true  (this edge has a junction sign record)
```

Two buildings 60 m apart, two completely different tile files, two completely different GraphIds, two completely different defaults (alley 20 km/h vs primary 60 km/h). That is the §1 hierarchy, made concrete.

### The math: how one edge knows its twin, how one node finds its neighbors

Students usually stop here and ask two questions:

> **Q1.** "If each record is *directed*, where is the reverse of that alley?"
> **Q2.** "Given just a node, how do I discover what is connected to it?"

Both answers are pure pointer arithmetic. No joins, no indexes, no graph traversal.

#### Model: one OSM way = one pair of DirectedEdges

```
OSM way W  ────►  ( e+ , e- )          (ordered pair of DirectedEdge records)
```

Three hard invariants tie the pair together:

```
(1)   e+.endnode            =  e-.startnode
(2)   e-.endnode            =  e+.startnode
(3)   e+.edge_info_offset   =  e-.edge_info_offset
```

What each line means:

- **(1) + (2)** — a DirectedEdge only stores its *end* node. To get the *start* node, you go through the twin. Start of `e+` = end of `e-`, and vice-versa.
- **(3)** — the human-readable payload (names, shape, way_id, speed_limit) lives *once* in the `EdgeInfo[]` array. Both halves point to the same offset, so "Hẻm 16 Đinh Tiên Hoàng" is never duplicated on disk.

**Twin lookup is one line of arithmetic:**

```
twin(e)  =  edges[ e.endnode.edge_index  +  e.opp_index ]
                   │                        │
                   │                        └─ "my slot within my endnode's outgoing list"
                   └─ "first outgoing edge of my endnode"
```

Three field reads. That is the whole operation.

#### Live proof: the alley at 28bis

`/locate` on way `605149488` returned two DirectedEdge records - the twin pair:

```
e+  =  edge 83734   forward=true    endnode = 39958   live = 16 km/h
e-  =  edge 83729   forward=false   endnode = 39959   live = 20 km/h
```

Apply invariants (1) and (2):

```
start(e+)  =  e-.endnode  =  39959
start(e-)  =  e+.endnode  =  39958
```

So the same 40 m of asphalt is stored as two independent records:

```
        forward (e+, id=83734, live=16)
   39959 ───────────────────────────────► 39958
   39958 ◄─────────────────────────────── 39959
        reverse (e-, id=83729, live=20)
```

Two different live speeds (16 vs 20) because entering the alley and leaving it are separate traffic measurements. That is exactly why Valhalla splits them - the graph carries direction-specific state without any `if (direction == ...)` branch in the routing code.

#### Node discovery (Q2)

Every `NodeInfo` record stores two tiny scalars that do all the work:

```
N.edge_index  =  first outgoing edge id in this tile
N.edge_count  =  k  (how many outgoing edges, same tile + same level)
```

All outgoing edges of N live **contiguously in memory**:

```
Out(N)  =  { edges[N.edge_index + j]   for j = 0, 1, ..., k-1 }
```

Neighbors drop out for free:

```
Neighbors(N)  =  { e.endnode   for e in Out(N) }
```

Cost of listing every neighbor of a node = **k struct reads**. No traversal, no hash map, no lookup table.

#### Live proof: node 40041 in tile 581466

`/locate` at `(10.7889, 106.69996)` with `node_snap_tolerance = 30` snapped to a real intersection:

```
NODE              value = 1,343,557,663,442   →  level=2, tile=581466, id=40041
POSITION          (10.788928, 106.700024)
type              street_intersection          intersection_type = regular
local_edge_count  = 3                          ← |Out(N)| at this level
transition_count  = 1                          ← 1 upward link (L2 → L1 copy of this node)
density           = 15                         ← ~15 edges/km² neighbourhood
traffic_signal    = false
drive_on_right    = false                      ← ⚠ surprising for HCMC, flagged in §11
```

Edges that the same `/locate` call returned near node 40041 (two twins on Hẻm 21 plus two Nguyễn Đình Chiểu halves picked up by the wider radius):


| edge id | way        | name                           | forward | endnode   | len  |
| ------- | ---------- | ------------------------------ | ------- | --------- | ---- |
| 83961   | 1127702787 | Hẻm 21 Đường Nguyễn Đình Chiểu | false   | **40041** | 28 m |
| 83903   | 1127702787 | Hẻm 21 Đường Nguyễn Đình Chiểu | true    | 40070     | 28 m |
| 87660   | 1173852133 | Nguyễn Đình Chiểu              | true    | 39617     | 70 m |
| 90663   | 1173852133 | Nguyễn Đình Chiểu              | true    | 39636     | 27 m |


Reading this like a student:

- Edge **83961** has `endnode = 40041` — it flows *into* our node. Its twin, edge 83903, flows *out* of 40041 toward node 40070. So from 40041 you can reach 40070 down the alley.
- Edges **87660** and **90663** run along Nguyễn Đình Chiểu, the cross street. Their endpoints 39617 and 39636 are the two adjacent intersections on that road, ~70 m and ~27 m away.

Put as a set:

```
Neighbors(40041)  =  { 40070,  39617,  39636 }       (3 nodes, matches local_edge_count = 3)
```

Pictorially:

```
            39617  (Nguyễn Đình Chiểu, 70 m west)
                ▲
                │
                │
   40070 ◄──────40041──────► 39636  (Nguyễn Đình Chiểu, 27 m east)
   (alley        │
   continues     │
   south)        ▼
            Hẻm 21
```

#### Putting it together: the A* inner loop, in plain terms

Given current node `N` and goal `G`, every pop of the open set does this:

```
for each e in Out(N):
    N'  =  e.endnode                          ← 1 field read
    g'  =  g[N] + edge_cost(e)                ← arithmetic
    f'  =  g' + h(N', G)                      ← heuristic call
    push (N', f') onto open set               ← heap push

If e crosses a tile boundary:
    tile = GetGraphTile(N'.tile_id, N'.level) ← O(1) hash; mmap if not cached
```

That is it. No string parsing, no database query, no allocation per step. **A forward scan across `edges[edge_index .. edge_index + edge_count]`, a field read for `endnode`, arithmetic for g and f, done.** This is why a 2 km city route can settle tens of thousands of nodes in under 30 ms — every node expansion is a handful of L1 cache lines.

### Concrete evidence from debug log

A single short route (9.84 km Hanoi inner-city) touched 7 tiles:

```
[fwd] TILE LOAD #2 tile=2/640503/0 L2 (2/000/640/503.gph) nodes=137361 edges=321546
[fwd] TILE LOAD #3 tile=1/40245/0  L1 (1/040/245.gph)    nodes=73718  edges=164357
[fwd] TILE LOAD #5 tile=0/2501/0   L0 (0/002/501.gph)    nodes=108728 edges=241144
```

All three levels, loaded on demand as the frontier crossed tile boundaries. A single L2 local tile in Hanoi already holds 320k road segments. An L0 highway tile covering 4° × 4° holds 240k - fewer, but each segment is a much longer road.

---

## 4. Algorithms - Easy → Medium → Hard

### 4.0 Which algorithm runs when? (decision tree)

Thor is the pathfinder, but it is not one algorithm. It picks based on the shape of the request:


| Request type                                      | Algorithm                             | Implementation file                                                                                                     |
| ------------------------------------------------- | ------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `isochrone` or `reach`                            | Dijkstra (no goal, fan out)           | [src/thor/isochrone.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/isochrone.cc)                       |
| `sources_to_targets` (N×M matrix)                 | Many-to-many Dijkstra                 | [src/thor/costmatrix.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/costmatrix.cc)                     |
| Two-location route, no time                       | **Bidirectional A** (default)         | [src/thor/bidirectional_astar.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc)   |
| Two-location route, `depart_at` or `arrive_by`    | **Unidirectional A** (time-dependent) | [src/thor/unidirectional_astar.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc) |
| Multimodal (walk + transit)                       | MultiModal A                          | [src/thor/multimodal.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/multimodal.cc)                     |
| Trivial (origin and destination on the same edge) | Short-circuit inline                  | Inside `route_action`                                                                                                   |


**Key implication:** when a request passes a `depart_at` timestamp, Valhalla switches off bidirectional and runs unidirectional A instead. This changes both the performance profile (2× slower on a long trip) and the live-traffic behaviour (see §5.10–5.11). Relevant to any feature that schedules routes for a future departure.

### 4.1 EASY - A (unidirectional)

**One-line model:** Dijkstra, but each candidate node carries a cheat-sheet estimate of remaining cost to the goal, so we stop exploring in the wrong direction.

```
f(n) = g(n) + h(n)
       ^^^^   ^^^^
       cost   admissible estimate
       so far of remaining cost
```

- `g(n)` = accumulated cost from the origin to this node (what we have actually spent).
- `h(n)` = an optimistic estimate of remaining cost from this node to the goal (the heuristic).
- `f(n)` = our best guess of "cost of the cheapest path through `n`." A always pops the edge with the smallest `f`.

#### Walkthrough of Image #5 (the A → F single-direction diagram)

The diagram shows 6 nodes, A (start) → F (goal). `h(n)` is straight-line distance × cost factor (see §4.4).


| Step | Pop           | Expand (neighbours)                                   | Open set after   | Why                                              |
| ---- | ------------- | ----------------------------------------------------- | ---------------- | ------------------------------------------------ |
| 1    | A(f=5.1)      | B(g=3,h=4.12→f=7.12), C(g=2,h=4.47→**f=6.47**)        | C: 6.47, B: 7.12 | Start. Add A's neighbours with their `g+h`.      |
| 2    | **C**(f=6.47) | B already open (no improvement), E(g=7,h=2.82→f=9.82) | B: 7.12, E: 9.82 | C had lower `f` than B → explore C first.        |
| 3    | **B**(f=7.12) | A closed, C closed, D(g=7,h=2.23→f=9.23)              | D: 9.23, E: 9.82 | B's new options.                                 |
| 4    | **D**(f=9.23) | E no improvement, F(g=10,h=0→**f=10**)                | E: 9.82, F: 10   | Found F! But E is still cheaper, don't stop yet. |
| 5    | **E**(f=9.82) | C closed, D closed, **F(g=9,h=0→f=9)**                | F: 9             | E gives a better path to F via C→E→F.            |
| 6    | **F**(f=9)    | Goal reached. Reconstruct: F←E←C←A. Total cost = 9.   |                  |                                                  |


**Critical lesson:** A does NOT stop the first time it touches F. It stops when F is the cheapest thing in the open set. This is why admissibility of `h(n)` matters - if we overestimate, we might accept a suboptimal F before a cheaper path via E is found.

#### Evidence in Valhalla

- **Heuristic class:** [valhalla/thor/astarheuristic.h#L62-L64](https://github.com/valhalla/valhalla/blob/master/valhalla/thor/astarheuristic.h#L62-L64)
  ```cpp
  float Get(const midgard::PointLL& ll) const {
    return sqrtf(distapprox_.DistanceSquared(ll)) * costfactor_;
  }
  ```
  Straight-line distance × factor. Literally the formula above.
- **Admissibility is an invariant, not a suggestion:** [valhalla/thor/astarheuristic.h#L60](https://github.com/valhalla/valhalla/blob/master/valhalla/thor/astarheuristic.h#L60) says: `For A* shortest path this MUST UNDERESTIMATE the true cost.`
- **Unidirectional main loop:** [src/thor/unidirectional_astar.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc). The heart is a `while` loop that pops the edge with the smallest `f` and expands its neighbours.

---

### 4.2 MEDIUM - Bidirectional A

**One-line model:** Run A from the origin going forward AND another A from the destination going backward, simultaneously. Stop when they meet. For long routes this roughly halves the explored area - two small balloons instead of one huge one.

#### Geometric intuition

Unidirectional A fans out until it reaches the goal. In the worst case (no heuristic guidance) it explores a disc of radius `d` = origin-to-destination distance:

```
area_unidir ≈ π × d²
```

Bidirectional A runs two discs of radius `d/2` until they meet in the middle:

```
area_bidir ≈ 2 × π × (d/2)²  =  π × d² / 2
```

**Half the area, half the work - and the savings grow with distance.** This is why long intercity routes benefit the most from bidirectional search.

#### Walkthrough of Image #4 (the A ↔ F bidirectional diagram)

Three subtle points on the diagram:

1. `**mu`** is "best known total path cost so far". It starts at `+∞` and improves every time the two frontiers meet at a node. Each meeting gives a candidate path with cost `g_fwd(node) + g_bwd(node)`; if that beats the current `mu`, it replaces it.
2. **Alternating step rule:** pop from whichever side has the cheaper frontier minimum. Keeps the work balanced so neither side runs too far ahead.
3. **Do not stop at first meeting.** Keep going until `min(f_fwd) + min(f_bwd) ≥ mu + threshold_delta`. Only then can we guarantee no alternative meeting point is cheaper. Valhalla's default `threshold_delta` is `420.0` cost units.

In Image #4: C is the first meeting node with `mu = 2+7 = 9`. The diagram then shows both frontiers being checked before declaring `A → C → E → F, cost = 9` as the final answer.

#### Evidence in Valhalla

- **Initial `mu = +∞`:** [src/thor/bidirectional_astar.cc#L146](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L146)
  ```cpp
  cost_threshold_ = std::numeric_limits<float>::max();
  ```
- **Meeting detection** ([src/thor/bidirectional_astar.cc#L623-L624](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L623-L624)):
  ```cpp
  const auto opp_status = edgestatus_reverse_.Get(fwd_pred.opp_edgeid());
  if (opp_status.set() == EdgeSet::kPermanent || ...)
    SetForwardConnection(graphreader, fwd_pred);
  ```
  A forward edge whose opposing direction was already permanently settled by the reverse search means "we've met here."
- **Keep searching past first meet** ([src/thor/bidirectional_astar.cc#L894-L901](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L894-L901)) - `SetForwardConnection` updates the budget ceiling to `c + threshold_delta_` and continues, because shortcuts plus hierarchy make the heuristic **non-monotone**; the first meet is not automatically optimal.
- **Forward termination** ([src/thor/bidirectional_astar.cc#L768-L778](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L768-L778)):
  ```cpp
  if (cost_threshold_ != std::numeric_limits<float>::max() &&
      route_lower_bound > cost_threshold_) { return FormPath(...); }
  ```

**Note:** textbook bidirectional A stops at first meeting. Valhalla keeps searching a configurable `threshold_delta` past it - this is why `/expansion` output sometimes shows edges being explored after a visible meet. It is not a bug; it compensates for the heuristic losing admissibility across shortcut boundaries.

#### Hanoi concrete trace (from log)

From `debug.txt`, for a 9.84 km Hanoi route:

```
step 5: FIRST CONNECTION (forward met reverse)
  meeting edge=0/2501/73729
  fwd accumulated: cost=893.48  sec=632.03
  rev accumulated: cost=922.33  sec=512.77
  connection_cost = 1786.199
  cost_threshold = 1786.199 + 420.0 = 2206.199      ← budget ceiling
  labels so far: fwd=10253 rev=3522
  continues expanding: any edge with sortcost < 2206.199 could yield a better path
...
step 6: TERMINATE - rev sortcost exceeds threshold
  rev sortcost = 2214.04 > threshold = 2206.20
...
connections found: 20 | desired_paths: 1
  connection[0]: edge=0/2501/73729 cost=1786.199 (BEST)
  connection[1]: edge=0/2501/73757 cost=1786.199 (+0.000)   ← tie
  connection[2]: edge=0/2501/209609 cost=1863.97 (+77.77)
  ... 17 more, each worse
```

**Interpretation:**

- "First connection" is not the final answer. It sets the budget ceiling at **meeting_cost + 420**.
- The search continues. In this trace, **20 candidate meetings** were found before the cheapest unexplored reverse edge exceeded the ceiling.
- Only the best is kept. The other 19 would be returned if the request asked for alternates.

### 4.2b REACH PRUNE - how Valhalla kills hopeless edges early

Beyond meeting detection, bidirectional A aggressively prunes edges whose **optimistic lower bound** cannot beat the current `cost_threshold`. Our debug log shows 1,544 such prunes in a single 9.84 km route:

```
[fwd] REACH PRUNE edge=2/640503/117717 lb=2208.40 > thr=2206.20
  (pred=1096.17 + tc=4.20 + rev_sc=1231.15 - h)
```

**The formula for a forward-side lower bound:**

```
lb = predecessor_cost + transition_cost + opposite_frontier_sortcost − heuristic
```

 Notes: "Best case, this edge connects to the reverse frontier at the cheapest known reverse sortcost. If even that optimistic math is already above our threshold, skip it." This is why bidirectional scales - most edges near the frontier boundary are killed without ever being expanded.

Implementation lives in [src/thor/bidirectional_astar.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc) - search for `PruneEdgeByReach`. Note it only kicks in **after** the first meeting, because you need an opposite-side sortcost to prune against.

---

### 4.3 HARD - Hierarchical Bidirectional A with Shortcuts

This is how we route Hanoi → HCMC (~1,653 km) in under a second.

> **Scope of this section.** Everything below describes **bidirectional** A. Unidirectional A (the algorithm used for time-dependent `/route` requests - `depart_at` / `arrive_by`) reuses the **hierarchy** machinery (level pruning and transition counting) but **skips shortcut edges entirely** (shortcut usage and shortcut recovery do not apply). See [§4.3b](#43b-what-carries-over-to-unidirectional-a-time-dependent-routing) for the precise overlap and its performance consequences.

Bidirectional alone would still explore too many side streets mid-country. The hierarchical optimizations, in order:

#### Optimization 1 - Promote up, never down (usually)

As each frontier moves away from its endpoint, it stops exploring L2, then L1.

**Mechanism:** `StopExpanding()` at [valhalla/sif/hierarchylimits.h#L44-L47](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h#L44-L47)

```cpp
inline bool StopExpanding(const HierarchyLimits& hl, const float dist) {
  return (hl.up_transition_count() > hl.max_up_transitions() &&
          dist > hl.expand_within_dist());
}
```

**Notes:** "If we have taken too many upward transitions already AND we are still far from destination, stop expanding this level." The AND is the safety net - near the endpoint we always re-allow local streets so we can reach the actual drop-off address.

**From our log:**

```
[fwd] HIERARCHY PRUNE L2: up_transitions=194 > max=100 AND dist=5008.71m > expand_within=5000.00m
  meaning: forward exhausted L2 edges, promoting to higher levels only
```

#### Optimization 2 - Default limits

From [valhalla/sif/hierarchylimits.h#L21-L29](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h#L21-L29):


| Level         | `max_up_transitions` | `expand_within_dist` (unidir) | `expand_within_dist` (bidir) |
| ------------- | -------------------- | ----------------------------- | ---------------------------- |
| L0 (Highway)  | 0 (cannot go up)     | ∞ (always expand)             | ∞                            |
| L1 (Arterial) | 400                  | 100 km                        | 20 km                        |
| L2 (Local)    | 100                  | 5 km                          | 5 km                         |


**Key insight:** bidirectional shrinks the "always expand arterial" zone from 100 km down to 20 km. That is a big part of why bidirectional is faster - we bail out of arterials much sooner.

#### Optimization 3 - Shortcut edges on L0/L1

A shortcut edge is a synthetic `DirectedEdge` that represents "traverse this chain of 12 real edges in one hop." It is created at tile-build time by the [Mjolnir](https://github.com/valhalla/valhalla/tree/master/src/mjolnir) preprocessor and used only by the router.


|                                 | Standard edge     | Shortcut edge                    |
| ------------------------------- | ----------------- | -------------------------------- |
| Source                          | OSM 1:1           | Generated during preprocessing   |
| Used for                        | Turn instructions | Fast traversal only              |
| Visible to Odin (the narrator)? | Yes               | No - must be expanded back first |
| Typical home                    | All levels        | L0, L1                           |


#### Optimization 4 - Recover shortcuts before narrating

The bidirectional path may include shortcut edges. We cannot tell the user "in 127 km, turn right" - we need every maneuver on the real OSM edges. So:

- [valhalla/baldr/graphreader.h#L713](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphreader.h#L713) → `RecoverShortcut(GraphId)` expands one shortcut into its underlying edges.
- Odin (the narrator module) only ever sees standard edges.

**Analogy:** Shortcut edges are summaries of the underlying edge chain. They are used during search for speed, but must be expanded back to their real underlying edges before turn-by-turn narration is produced.

#### Concrete: short vs long route efficiency (from production logs)


| Route            | Distance | Path edges | Labels explored | Labels per path edge | Tiles loaded          |
| ---------------- | -------- | ---------- | --------------- | -------------------- | --------------------- |
| Hanoi inner-city | 9.84 km  | 254        | 31,812          | **125 : 1**          | 7 (L0=1, L1=2, L2=4)  |
| HCMC → Hanoi     | 1,653 km | 3,301      | 197,027         | **60 : 1**           | 18 (L0=9, L1=5, L2=4) |


**Counter-intuitive observation:** the long route has a better labels-per-edge ratio. L0 shortcut edges allow the search to jump 50–100 km per label, while the short urban route gets no such leverage - every neighbourhood street is one label. **Hierarchy plus shortcuts are worth more the longer the trip.**

Also notice: the long route touches 9 L0 tiles versus 1 for the short route. That is the highway network carrying the middle 1,400 km - where no L2 tile is loaded at all, because both frontiers stopped expanding L2 within 5 km of each endpoint.

#### End-to-end walkthrough of a long intercity request


| Step | User-visible state                      | What is happening internally                                            |
| ---- | --------------------------------------- | ----------------------------------------------------------------------- |
| 1    | "Loading your map…"                     | Three-level tile index mmap'd (milliseconds)                            |
| 2    | "Starting from origin address"          | Bidirectional forward launched on L2, origin snapped to candidate edges |
| 3    | "Leaving the local area"                | `up_transition_count > 100 AND dist > 5 km` → stop L2, promote to L1    |
| 4    | "Entering the highway"                  | Same test trips on L1 → promote to L0                                   |
| 5    | "Skipping cities in between"            | L0 shortcut edges skip 50–100 km chunks                                 |
| 6    | "Meanwhile from destination…"           | Reverse A running concurrently, same rules                              |
| 7    | "Routes met"                            | `edgestatus_reverse_.Get(fwd_pred.opp_edgeid()) == kPermanent`          |
| 8    | "Here are your turn-by-turn directions" | `RecoverShortcut()` expands shortcuts, Odin generates narration         |


---

### 4.3b What carries over to unidirectional A (time-dependent routing)

Time-dependent `/route` calls (`date_time_type = depart_at` or `arrive_by`) run on `UnidirectionalAStar`, not the bidirectional algorithm described above. The overlap with §4.3 is partial, and the differences matter for ETA accuracy and latency.


| Hierarchical mechanism                                                 | Bidirectional (§4.3)        | Unidirectional (time-dependent)                                                                 | Evidence                                                                                                                                                                                                                                                                                                                                   |
| ---------------------------------------------------------------------- | --------------------------- | ----------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `StopExpanding` pruning by `max_up_transitions` + `expand_within_dist` | Applied                     | **Applied** (same code path)                                                                    | [src/thor/unidirectional_astar.cc#L121](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc#L121), limits read at [#L654](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc#L654)                                                                                        |
| Up-transition counting between tile levels                             | Applied                     | **Applied**                                                                                     | [src/thor/unidirectional_astar.cc#L126-L127](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc#L126-L127)                                                                                                                                                                                                |
| `expand_within_dist` values                                            | L1 = 20 km, L2 = 5 km       | **L1 = 100 km**, L2 = 5 km                                                                      | [scripts/valhalla_build_config#L254](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config) vs [#L245](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                        |
| Shortcut edges                                                         | Used - 50-100 km hops on L0 | **Not used** - shortcut edges are filtered out at the start of expansion                        | [src/thor/unidirectional_astar.cc#L177-L181](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc#L177-L181) - literal `if (meta.edge->is_shortcut()) return false;` with the comment `// Skip shortcut edges for time dependent routes` (the upstream TODO asks "why?" but the behaviour is unconditional) |
| `RecoverShortcut()` pre-narration step                                 | Required                    | **Not required** - no shortcut edges are ever on the path, so Odin sees the real edges directly | Follows from the previous row                                                                                                                                                                                                                                                                                                              |


**Why unidirectional skips shortcuts.** A shortcut edge stores its weight as a single precomputed traversal cost, but a time-dependent cost depends on the moment of traversal of **every component edge** of the shortcut. Evaluating a shortcut as a single edge would force a single `seconds_from_now` value to stand in for the entire chain, which is wrong for any non-constant live or predicted speed profile. The safe option is to never consider shortcuts when time dependency is on - which is exactly what the code does. (The upstream TODO invites a future optimisation that expands the shortcut, prices each underlying edge with its own time offset, and reassembles - but that optimisation has not been implemented.)

**Performance consequences.**

1. **Long-route cost is linear in real edges.** Without L0 shortcuts, a 1,500 km time-dependent route must traverse every intermediate highway node individually. Labels-explored scales with the number of real edges, not with the number of shortcut hops - the "60 labels per path edge" headline in §4.3 is a bidirectional achievement that unidirectional cannot match.
2. **The 100 km vs 20 km L1 zone partially compensates.** Unidirectional keeps arterial detail (L1) for 100 km out from the origin, while bidirectional only keeps it for 20 km before meeting the reverse frontier. This gives unidirectional a better chance at finding a good arterial route when it can no longer use highway shortcuts.
3. **This is the second reason time-dependent routing is slower.** §4.5 explains the first reason (losing the bidirectional 2× speed-up because reverse has no knowledge of arrival time). The shortcut skip is a separate, additive cost on long routes.

**Under-500 km production request shape.** Our production team always sends `date_time.type = 1` (current). A request under 500 km beeline goes through `timedep_forward` (unidirectional) - it therefore:

- Uses hierarchy (L1 opens at 5 km, L0 opens at 100 km).
- Does **not** use shortcut edges.
- Keeps arterial detail longer than bidirectional would.

See §5.12 for the full algorithm-selection reference and §11.1 for how this interacts with Vietnam-urban tuning.

---

### 4.4 The A Cost Factor - Why admissibility is global, not per-tile

The heuristic needs a scalar to convert "metres remaining" into "seconds remaining." That scalar is `AStarCostFactor()`:

- **Contract:** [valhalla/sif/dynamiccost.h](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/dynamiccost.h) - every costing model (auto, bicycle, pedestrian, truck, bus…) must implement it.
- **Auto costing:** [src/sif/autocost.cc#L298-L300](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L298-L300)
  ```cpp
  float AStarCostFactor() const override {
    return kSpeedFactor[top_speed_] * min_linear_cost_factor_;
  }
  ```

From `debug.txt` for auto (top_speed = 140 kph):

```
AStarCostFactor: 0.025714
  = kSpeedFactor[top_speed] * min_linear_cost_factor
  = (3600 * 0.001 / top_speed_kph)    ← sec/m at max road speed
  purpose: underestimates real cost so A* heuristic is admissible
```

**Why globally, not per-tile?**

1. **Admissibility.** A is only guaranteed optimal if `h(n) ≤ true_cost(n → goal)` for every node. If we lowered the factor in a slow-traffic area, we would overestimate there and under-estimate elsewhere; crossing tiles would produce non-monotone `f`, which breaks the proof and in practice produces bad routes.
2. **Stability across tile boundaries.** Using one global "best-case speed" keeps `h` smooth as the search jumps tiles.

Traffic cannot be applied to the heuristic `h(n)` without breaking admissibility. Traffic is applied only in `g(n)` - the actual observed cost of the edges the search has settled. Section 5 describes how traffic enters `g(n)`.

---

### 4.5 Why `depart_at` / `arrive_by` forces unidirectional (the circular dependency)

Time-dependent costing means: "the speed of this edge depends on when I reach it." Forward search knows this trivially - `seconds_from_now = accumulated_time_so_far`. But a **reverse** search starting at destination has a chicken-and-egg problem:

```
cost_of_edge(e) needs arrival_time(e)
arrival_time(e) = depart_time + time_to_reach(e)    ← what we are trying to solve for
```

Reverse cannot resolve this without assuming the final time, which defeats the purpose. So Thor silently picks unidirectional when `date_time_type ∈ {2: depart_at, 3: arrive_by}`.

**Side effect 1 - no bidirectional:** a time-dependent query loses the 2× speed-up from bidirectional. Unidirectional is ~2× slower on the same network just from not having two converging frontiers.

**Side effect 2 - no shortcuts:** unidirectional also skips shortcut edges entirely ([§4.3b](#43b-what-carries-over-to-unidirectional-a-time-dependent-routing)). On long routes this compounds: every highway node is a distinct label rather than being collapsed into a 50-100 km shortcut hop. Expect 2-3× slower latency on short/medium routes and a larger slowdown on intercity routes where shortcuts would have been doing the most work. See the matrix comparison in §7 for the practical cost.

---

### 4.6 Understanding "total cost" - the number A actually sorts by

A common point of confusion in the debug logs is the difference between `cost` and `time`. This section clarifies the distinction.

#### The rule in one sentence

> **Cost is NOT time in seconds. Cost is "time plus penalties" - a synthetic number that Valhalla minimises to pick the most *preferred* route, not strictly the fastest.**

Two separate numbers live on every edge and path:


| Quantity | What it is                                                 | Used for                                  |
| -------- | ---------------------------------------------------------- | ----------------------------------------- |
| `secs`   | Pure physical time. `length / speed`.                      | Displayed to the user as ETA.             |
| `cost`   | `secs × factors + penalties`. An abstract "badness" score. | Priority queue. A pops the lowest `cost`. |


They are deliberately different. If cost were equal to time, the router would always pick the fastest path even if it involved 40 turns through alleyways, private gates, and a toll road - all things users hate. Adding penalties to `cost` lets us tune *preferences* without lying about the ETA.

#### The formula (Công thức tính toán)

> **Note:** Every piece of information below directly impacts how the routing algorithm picks the best path. 

Total path cost is the sum of **two things per edge** 
The relationship between them is sequential: you travel *along* a road (`edge_cost`), then you turn/transition *onto* the next road (`transition_cost`).

*(Note to avoid confusion: In Valhalla, the word "transition" is used in two different ways. Here, `transition_cost` means the penalty for turning at an intersection (e.g., waiting at a red light or making a sharp left turn). Earlier in the document, `up_transition_count` meant promoting from a lower-level road to a higher-level road (e.g., Alley to Arterial). One edge can have multiple `transition_costs` if it crosses many intersections, but it only triggers an `up_transition` when the road class actually upgrades! Evidence: `up_transition_count` is only incremented when the algorithm encounters a special `NodeTransition` object where `trans->up() == true`, at `src/thor/unidirectional_astar.cc#L126-L127`. Most normal intersections have `nodeinfo->transition_count() == 0`, meaning they don't have any `NodeTransition` objects at all, which is why the `up_transition_count` grows so slowly!)*

```text
path_cost = Σ (edge_cost_i + transition_cost_i)
path_time = Σ (edge_secs_i + transition_secs_i)

(Tổng chi phí = Σ (Chi phí đoạn đường_i + Chi phí chuyển hướng_i))
(Tổng thời gian = Σ (Thời gian đi đoạn đường_i + Thời gian chuyển hướng_i))
```

Where for each edge (Chi tiết cho từng đoạn đường):

```text
edge_cost  = (secs × density_factor × highway_factor × surface_factor
              × speed_penalty × toll_factor × alley_factor
              × track_factor × living_street_factor × service_factor
              × turn_channel_factor × edge_factor × closure_factor)
             + any_edge_level_penalty
secs       = length / speed (Note: 'speed' is dynamically fetched from Live Traffic first, then Historical Traffic, then static OSM maxspeed. Thus, traffic jams reduce speed, increase 'secs', and multiply the overall edge_cost)

(Chi phí đoạn đường = (Thời gian đi × Hệ số mật độ × Hệ số đường cao tốc × Hệ số mặt đường
                       × Phạt tốc độ × Hệ số trạm thu phí × Hệ số ngõ hẻm
                       × Hệ số đường mòn × Hệ số đường khu dân cư × Hệ số đường nội bộ
                       × Hệ số làn rẽ × Hệ số đoạn đường × Hệ số đường bị đóng)
                      + Các hình phạt phụ thêm trên đoạn đường này)
(Thời gian đi = Chiều dài / Tốc độ)
```

And for each node transition (turn from one edge onto the next) (Chi tiết cho mỗi lần chuyển hướng - rẽ từ đường này sang đường khác):

```text
transition_cost = turn_penalty[angle]         // sharp turns cost more (Góc rẽ càng gắt càng tốn chi phí)
                + maneuver_penalty            // baseline for any turn (Chi phí cơ bản cho mọi cú rẽ)
                + gate_cost + gate_penalty    // passing a gate (Khi đi qua cổng)
                + toll_booth_cost             // passing a toll booth (Khi đi qua trạm thu phí)
                + country_crossing_cost       // border crossing (Khi qua cửa khẩu/biên giới)
                + ferry_cost                  // taking a ferry (Khi đi phà)
                + rail_ferry_cost             // taking a rail ferry (Khi đi phà đường sắt)
                + private_access_cost         // private access gates/bollards (Khi qua cổng tư nhân)
                + bike_share_cost             // bike share stations (Khi qua trạm xe đạp công cộng)
                + destination_only_penalty    // destination only roads (Phạt đi vào đường chỉ dành cho dân cư)
                + alley_penalty               // entering an alley (Phạt đi vào ngõ hẻm)
                + living_street_penalty       // entering a living street (Phạt đi vào đường khu dân cư)
                + track_penalty               // entering a track (Phạt đi vào đường mòn)
                + service_penalty             // entering a service road (Phạt đi vào đường nội bộ/dịch vụ)
                + stop_impact                 // penalty for crossing a busier road (Phạt dừng xe khi cắt ngang đường đông đúc hơn)
transition_secs = the actual seconds lost (usually a subset of the above) (Số giây thực tế bị mất, thường là một phần của các chi phí trên)
```

> **Q: What if a feature like a bike share station, gate, or toll booth is in the middle of a road?**
> A: In the routing graph, roads are explicitly split at these features. A feature in the middle of a road becomes a "node" connecting two shorter edges. Therefore, you only ever pay these costs at the nodes (transitions), never in the middle of an edge.

References (Tài liệu tham khảo): [src/sif/autocost.cc#L938-L1008](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc) (EdgeCost), [src/sif/autocost.cc#L1010-L1099](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc) (TransitionCost), [valhalla/sif/dynamiccost.h](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/dynamiccost.h) (penalty constants - các hằng số phạt).

#### Default penalties for `auto` costing

The values our production service sends in the Loki request:


| Penalty                    | Value | When it applies                                                | Giải thích (Vietnamese)                                                |
| -------------------------- | ----- | -------------------------------------------------------------- | ---------------------------------------------------------------------- |
| `maneuver_penalty`         | 5     | Any non-trivial turn (per edge transition)                     | Phạt thao tác rẽ (áp dụng cho mọi ngã rẽ đáng kể)                      |
| `gate_cost`                | 30 s  | Passing a gate, adds to both cost and time                     | Thời gian qua cổng (cộng vào cả chi phí và thời gian)                  |
| `gate_penalty`             | 300   | Extra cost on gates (not added to time)                        | Điểm phạt thêm khi qua cổng (chỉ tăng chi phí, không tăng thời gian)   |
| `toll_booth_cost`          | 15 s  | Passing a toll booth                                           | Thời gian qua trạm thu phí                                             |
| `alley_penalty`            | 5     | Any edge flagged as an alley                                   | Phạt đi vào ngõ hẻm                                                    |
| `country_crossing_cost`    | 600 s | Border crossing                                                | Thời gian qua cửa khẩu/biên giới                                       |
| `ferry_cost`               | 300 s | Taking a ferry                                                 | Thời gian đi phà                                                       |
| `service_penalty`          | 75    | Service roads (parking-lot paths etc.)                         | Phạt đi vào đường nội bộ/dịch vụ (ví dụ: bãi đỗ xe)                    |
| `private_access_penalty`   | 450   | Private roads                                                  | Phạt đi vào đường tư nhân                                              |
| `destination_only_penalty` | 600   | "Residents only" streets                                       | Phạt đi vào đường chỉ dành cho dân cư khu vực đó                       |
| `closure_factor`           | 9     | Multiplier for closures - route through only if no alternative | Hệ số nhân khi đường bị đóng (chỉ đi qua nếu không còn đường nào khác) |


Reading these, you can see the design: the router will gladly add a minute to your ETA to avoid a toll-booth waste-of-time, skip a "residents only" cut-through, or detour around a closed road.

#### Worked example 1 - a Hanoi turn (short edge)

From our debug log, edge #2 on the 9.84 km Hanoi route:

```
edge[2/254] 1/40245/146964 | edge_cost=12.600  edge_secs=1.246
```


| What we see   | Number  | Interpretation                                                                                                       |
| ------------- | ------- | -------------------------------------------------------------------------------------------------------------------- |
| `edge_secs`   | 1.25 s  | Physical time - this is an L1 arterial segment, traversed in 1.25 seconds                                            |
| `edge_cost`   | 12.60   | Cost - roughly 10× the time                                                                                          |
| `cost - secs` | ≈ 11.35 | ← This is the penalty bulk: likely `maneuver_penalty` (5) plus a turn-class penalty on the transition into this edge |


**Takeaway:** the router pays "12.6 units" to make a small turn at the start of a short segment. The user sees 1.25 seconds of ETA for this edge, but A sees it as 10× more expensive because of the turn.

#### Worked example 2 - a highway shortcut (long edge)

From the long Hanoi→HCMC route:

```
edge[1500/3301] 0/2412/13525 | edge_cost=0.000  edge_secs=0.000
running_total: cost=24335.89 secs=25055.22 dist=641418m
```


| Observation                             | Why                                                                                                                                               |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| This specific edge shows 0 cost, 0 secs | It is a synthetic zero-length connector at a tile boundary. These exist in Valhalla's graph model - no time, no cost, no penalty. Safe to ignore. |
| The running totals tell the real story  | After 641 km: cost = 24,336, secs = 25,055. Ratio `cost/secs ≈ 0.97` - cost slightly LOWER than time.                                             |
| Why?                                    | Highway edges get a `highway_factor` discount (we want to prefer highways), so each highway second costs about 0.97 units.                        |


#### Worked example 3 - a penalty-heavy edge (edge #253)

```
edge[253/254] 2/639062/86723 | edge_cost=108.0  edge_secs=5.02
```


| Number               | Interpretation |
| -------------------- | -------------- |
| 5 seconds of driving | but...         |
| 108 units of cost    | 21.5× the time |


That is roughly the signature of a **gate** (`gate_cost=30s + gate_penalty=300` → divided across the transition) or a **destination-only** segment (`destination_only_penalty = 600`). This is the kind of edge the router would avoid if any alternative existed.

#### The overall cost/time ratio - what it tells you about the trip


| Route                      | Total cost | Total time (s) | Ratio `cost/time` | What it means                                                 |
| -------------------------- | ---------- | -------------- | ----------------- | ------------------------------------------------------------- |
| Hanoi inner-city (9.84 km) | 1,786      | 1,120          | **1.59**          | Urban: many turns, alleys, maneuver penalties - cost inflates |
| Hanoi → HCMC (1,653 km)    | 63,027     | 65,708         | **0.96**          | Highway-dominated: highway_factor discount, few penalties     |


**This ratio is a diagnostic.** A ratio above 1.5 says "urban-heavy, lots of penalties." Below 1.0 says "highway-dominated." Anomalies (5.0+, for example) suggest the route had to route through something painful - a ferry, a restricted road, or a closure with `closure_factor = 9` applied.

#### How to visualise cost in real time

Recommendations for dashboards and operational views:

1. **Do not colour edges by raw `cost`.** It scales with length and is unreadable - a 2 km highway segment and a 50 m alley could both be "cost = 50" for different reasons.
2. **Colour by `cost / length`** (seconds-per-metre equivalent) - a normalised "badness per metre" that is comparable across edges.
3. **Or colour by `speed`** - easier for non-engineering audiences to interpret.
4. **Query the `/expansion` endpoint** with `expansion_properties=["cost","duration","distance","edge_status","expansion_type"]` to get every edge the algorithm touched, with its cost, in GeoJSON.

Example request:

```json
POST /expansion
{
  "locations": [...],
  "costing": "auto",
  "expansion_action": "route",
  "expansion_properties": ["cost", "duration", "distance", "edge_status", "expansion_type"]
}
```

Response is a FeatureCollection where every LineString has `properties.cost`, `properties.duration`, `properties.edge_status` (`"s"` = settled/optimal, `"r"` = reached/in-queue), and `properties.expansion_type` (`0` = forward, `1` = reverse). This stream is what `tile_browser.html` consumes to replay A expansions frame by frame.

#### Summary

> `**secs` is what is shown to the user. `cost` is what A optimises. They differ because the router prefers fast-and-pleasant routes over strictly-fastest ones.**

---

## 5. Live Traffic - `traffic.tar` and the 1-Hour Fade

This section documents the live-traffic subsystem that is currently deployed in production.

### 5.1 On-disk format

`traffic.tar` is a plain tar archive of flat binary files, **one file per routing tile**. There is no database.

Each file:

```
TrafficTileHeader  (32 bytes - sizeof(uint64_t) * 4)
N × TrafficSpeed   (8 bytes each - sizeof(uint64_t))
```

**Source:** [valhalla/baldr/traffictile.h#L146-L150](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L146-L150) - `static_assert`s that enforce these sizes. The code refuses to compile if they drift.

**Note:** the source-header comment calls the header "24 bytes," but the `static_assert` enforces `sizeof(uint64_t) * 4 = 32`. The assertion is authoritative; the comment is stale.

### 5.2 Header

[valhalla/baldr/traffictile.h#L133-L140](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L133-L140)

```cpp
struct TrafficTileHeader {
  uint64_t tile_id;
  uint64_t last_update;          // seconds since epoch - freshness
  uint32_t directed_edge_count;
  uint32_t traffic_tile_version;
  uint32_t spare2;
  uint32_t spare3;
};
```

`last_update` is how you monitor freshness - see §8.2.

### 5.3 The `TrafficSpeed` bit-packed struct

[valhalla/baldr/traffictile.h#L45-L57](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L45-L57)

```cpp
struct TrafficSpeed {
  uint64_t overall_encoded_speed : 7;  // 2 kph resolution, max 252 kph
  uint64_t encoded_speed1 : 7;         // subsegment 1
  uint64_t encoded_speed2 : 7;         // subsegment 2
  uint64_t encoded_speed3 : 7;         // subsegment 3
  uint64_t breakpoint1 : 8;            // position = length * (breakpoint1 / 255)
  uint64_t breakpoint2 : 8;
  uint64_t congestion1 : 6;            // 0..63
  uint64_t congestion2 : 6;
  uint64_t congestion3 : 6;
  uint64_t has_incidents : 1;
  uint64_t spare : 1;
};
```

One 64-bit word. An edge can be split into up to three subsegments, each with its own speed and congestion - essential for long arterials where the first kilometre crawls and the last kilometre is clear.

**Why so much bit-packing?** `TrafficSpeed` is dereferenced inside the hottest inner loop of the router (`EdgeCost` → `GetSpeed`). Saving 8 bytes per edge × 1M edges per tile × 360 tiles amounts to roughly 2.8 GB of RAM for the Vietnam extract alone. The packed layout is necessary, not optional.

### 5.4 How it overlays onto routing tiles

```cpp
const volatile TrafficSpeed& trafficspeed(const uint32_t directed_edge_offset) const {
  ...
  return *(speeds + directed_edge_offset);
}
```

It is an **array index, not a lookup**. Edge #12345 in the routing tile corresponds to `TrafficSpeed[12345]` in the traffic tile. One-to-one, same ordering, same `GraphId → offset` arithmetic.

### 5.5 Live updates without restart - the `mmap` + `volatile` trick

See the `volatile` at [valhalla/baldr/traffictile.h#L221-L222](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L221-L222):

```cpp
volatile TrafficTileHeader* header;
volatile TrafficSpeed* speeds;
```

`volatile` tells the compiler: "another thread or process may change this memory at any time - do not cache reads."

**Flow:**

1. The router process opens `traffic.tar` read-only via `mmap`.
2. A separate traffic-updater process opens the same file writable.
3. The updater writes new 8-byte `TrafficSpeed` values directly into the mapped pages.
4. The kernel makes those pages visible to the router **immediately** - no reload, no signal, no restart.

This is why the ingestion pipeline can push updates at 1 Hz without the routing service needing to restart. The PR that introduced this mechanism: [PR #2268 "Support live traffic data"](https://github.com/valhalla/valhalla/pull/2268).

### 5.6 Blend formula - the heart of the thing

[valhalla/baldr/graphtile.h#L860-L863](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L860-L863)

```cpp
return static_cast<uint32_t>(
  partial_live_speed * partial_live_pct +
  (1 - partial_live_pct) * (std::max(speed, 0.5f) + 0.5f)
);
```

**version:**

```
final_speed = (live_speed × live_weight) + (historical_speed × (1 − live_weight))
```

### 5.7 Where does `live_weight` come from? The 1-hour fade

[valhalla/baldr/graphtile.h#L811-L816](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L811-L816)

```cpp
constexpr double LIVE_SPEED_FADE = 1. / 3600.;
float live_traffic_multiplier =
  1. - std::min(seconds_from_now * LIVE_SPEED_FADE, 1.);
```

`seconds_from_now` is "when will I reach this specific edge, in seconds from now." The router computes this per-edge during search.

**Derived weight table** (using live = 15 kph and historical = 50 kph for illustration):


| Time to reach edge | `live_weight` | Math              | Final speed                  |
| ------------------ | ------------- | ----------------- | ---------------------------- |
| Now (0 s)          | 1.00          | 15×1.0 + 50×0.0   | **15 kph** - 100% live       |
| 30 min (1800 s)    | 0.50          | 15×0.5 + 50×0.5   | **32.5 kph** - 50/50         |
| 45 min (2700 s)    | 0.25          | 15×0.25 + 50×0.75 | **41.25 kph**                |
| **1 hr (3600 s)**  | **0.00**      | 15×0.0 + 50×1.0   | **50 kph - 100% historical** |
| 8 hr               | 0.00          | -                 | 50 kph                       |
| 3 days             | 0.00          | -                 | 50 kph                       |


**Rule:** any edge the router expects to reach more than 60 minutes from `now` receives zero live-traffic influence. Speed is then purely historical or predicted. One compile-time constant (`1/3600`) governs this. Changing it has admissibility implications, as it biases `h(n)` indirectly via the cost surface.

### 5.8 What counts as "historical speed"?

[valhalla/baldr/graphtile.h#L857-L876](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L857-L876). Priority order:

1. **Predicted speed** - if `has_predicted_speed()` is true and a time-of-week was passed, use the predicted profile for that hour-of-week.
2. **Constrained flow** - 7am–7pm bucket, otherwise:
3. **Free flow** - 7pm–7am bucket.
4. **Base speed** on the `DirectedEdge` itself - baked from OSM `maxspeed` at tile-build time.

### 5.9 Gotchas

**5.9.1 `DateTimeType` enum values** ([proto/options.proto#L415-L421](https://github.com/valhalla/valhalla/blob/master/proto/options.proto#L415-L421)):

- `0 = no_time` - zero is not "current"; it means "do not use time at all"
- `1 = current`
- `2 = depart_at`
- `3 = arrive_by`
- `4 = invariant`

Passing `date_time_type = 0` results in **no live traffic, no predicted, only constrained/freeflow**. This is a frequent client-side misconfiguration.

**5.9.2 `UNKNOWN_TRAFFIC_SPEED_KPH = 254`.** When the tile browser renders an edge in dim grey, no live signal exists for that edge. These are gaps in the ingestion pipeline's coverage.

**5.9.3 `flow_mask`.** The production Loki request sets `flow_mask: 3`, a bitfield meaning `CURRENT | PREDICTED`. Setting it to `0` disables live traffic for that request; useful for A/B comparison.

### 5.10 The known bug - Issue #5616 (scope is narrower than it first appears)

There are two separate pieces of evidence, and they have to be reconciled carefully.

**Evidence 1 - the docstring.** In [valhalla/baldr/graphtile.h#L788-L793](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L788-L793) the `seconds_from_now` parameter of the speed lookup is documented as:

> "Absolute number of seconds from now till the moment the edge is passed. Be careful when setting the value in reverse direction algorithms to use proper value. It affects the percentage of live-traffic usage on the edge. The bigger `seconds_from_now` is set the less percentage is taken. **Currently this parameter is set to 0 when building a route with reverse and bidirectional a.**"

**Evidence 2 - the code.** In [src/thor/bidirectional_astar.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc) the reverse expansion DOES advance time via `time_info.reverse(pred.cost().secs, …)` and passes the updated `time_info` into `EdgeCost`. [valhalla/baldr/time_info.h#L212-L250](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L212-L250) confirms `reverse()` subtracts the offset from `seconds_from_now` (with a sign bit for past/future).

**The reconciliation** (important - do not skip this): the reverse tree's TimeInfo is constructed from the *destination* location ([bidirectional_astar.cc#L546-L561](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L546-L561)). In a typical Valhalla request the destination has no `date_time` field set - only the origin does. `TimeInfo::make(destination, …)` therefore starts the reverse tree with `seconds_from_now = 0` (wall-clock now). The `.reverse()` method then subtracts `pred.cost().secs` from that initial 0, producing a small-magnitude value that, once the fade treats it as `|sfn|`, behaves close to 0 for the entire reverse frontier. **That is what the docstring is summarising.** The forward tree is unaffected.

**Evidence 3 - Issue #5616.** [Issue #5616](https://github.com/valhalla/valhalla/issues/5616) ("Bidirectional A will use live traffic on the reverse expansion, regardless of date time", opened Oct 17 2025 by `chrstnbwnkl`) documents the user-visible symptom. The reporter's full quote, verbatim:

> "Assuming you are planning a route with a departure time more than an hour in the future: even if Valhalla has live traffic hot and loaded, unidirectional A won't use those speeds anywhere, since the time exceeds the live speed fading threshold. If you force Valhalla to use the bidirectional algorithm instead, the reverse expansion will use an invalid `TimeInfo` object which has `seconds_from_now` set to 0, and so it gets the unfaded live speed."

Things the reporter does **not** say:

- does not claim CostMatrix is affected (not mentioned in the issue body)
- does not claim map-matching is affected
- does not extend the claim to the forward tree

**Scope, stated precisely:**


| Algorithm                                                               | Forward side             | Reverse side                                                                                  | Impact on us                     |
| ----------------------------------------------------------------------- | ------------------------ | --------------------------------------------------------------------------------------------- | -------------------------------- |
| `timedep_forward` (unidirectional, used for `depart_at`)                | `sfn` advances correctly | n/a                                                                                           | Fade works.                      |
| `timedep_reverse` (unidirectional reverse, used for `arrive_by`)        | n/a                      | same construction issue as bidir reverse - the docstring's "reverse" caveat also applies here | Fade does not work on arrive_by. |
| `bidir_astar` (default when no `date_time` is set, or when `invariant`) | `sfn` advances correctly | starts at 0, barely advances                                                                  | Reverse frontier sees 100% live. |


**Important note:** `[route_action.cc::get_path_algorithm` lines 377–448]([https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L377-L448](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L377-L448)) routes `depart_at` with an origin `date_time` to `timedep_forward`, not to bidirectional. Bidirectional is only the default for requests where `date_time` is empty or `invariant` is set, or when the client explicitly forces it via `prioritize_bidirectional`.

No official fix has been merged. The reporter's suggested workaround - drop the `current` flow bit - costs live-closure information.

### 5.11 Exposure analysis - a 50-minute production route, end-to-end

This section walks a concrete 50-minute production route end-to-end, with every claim backed by a direct line-of-code link in the open-source repository. The goal is to establish precisely whether and when the live-traffic bug affects our deployment.

**Production request shape** (confirmed against the client source): the Go client sends `{"date_time": {"type": 1, "value": "<now formatted as ISO minutes>"}}` on every route request. To understand what Valhalla does with that input, an off-by-one between the JSON wire enum and the internal proto enum must be unpacked first.

#### Step 0 - the JSON-vs-proto enum shift

The HTTP/JSON API and the internal proto enum use **different numbering**. Valhalla's JSON parser at [src/worker.cc#L830-L832](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L830-L832) reads the JSON `date_time.type` and adds 1 before casting into the proto:

```cpp
830 auto date_time_type = rapidjson::get_optional<unsigned int>(doc, "/date_time/type");
831 if (date_time_type && Options::DateTimeType_IsValid(*date_time_type + 1)) {
832   options.set_date_time_type(static_cast<Options::DateTimeType>(*date_time_type + 1));
```

That `*date_time_type + 1` is the hinge. Mapping it out against the proto enum at [proto/options.proto#L415-L421](https://github.com/valhalla/valhalla/blob/master/proto/options.proto#L415-L421):


| JSON API wire value (what clients send) | After `+1` in parser | Proto enum (what the rest of Valhalla sees) | Meaning                                           |
| --------------------------------------- | -------------------- | ------------------------------------------- | ------------------------------------------------- |
| `0`                                     | `1`                  | `current`                                   | Server uses wall-clock now, ignores `value`       |
| `1`                                     | `2`                  | `depart_at`                                 | Server uses the `value` string as departure time  |
| `2`                                     | `3`                  | `arrive_by`                                 | Server uses the `value` string as arrival time    |
| `3`                                     | `4`                  | `invariant`                                 | Time is frozen (no historical/predicted variance) |


Second, the parser treats `value` asymmetrically depending on type ([src/worker.cc#L840-L843](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L840-L843)):

```cpp
840 auto date_time_value =
841     v != Options::current
842         ? rapidjson::get<std::string>(doc, "/date_time/value", options.date_time())
843         : std::string("current");
```

 Notes: **if type (proto) is `current`, the server ignores whatever `value` you passed and replaces it with the literal string "current".** For `depart_at` / `arrive_by` / `invariant`, the parser reads your `value` string and validates it with [src/worker.cc#L853](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L853).

Downstream, [valhalla/baldr/time_info.h#L95-L96](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L95-L96) states: *"if the string is 'current' it will be replaced with the now time"*, so even after parse the "current" string becomes `system_clock::now()` at [time_info.h#L124-L131](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L124-L131).

Net effect per mode:


| Proto type  | Server uses your `value` string?            |
| ----------- | ------------------------------------------- |
| `current`   | No - overwritten to `system_clock::now()`   |
| `depart_at` | Yes - must be valid ISO, else exception 162 |
| `arrive_by` | Yes - same rule                             |
| `invariant` | Yes - same rule                             |


#### Step 0b - what this means for the Go client

The current Go enum:

```go
CurrentDeparture   TravelTimeType = 0
SpecifiedDeparture TravelTimeType = 1
SpecifiedArrival   TravelTimeType = 2
InvariantSpecified TravelTimeType = 3
```

These values map to the JSON API, not the proto. The naming is correct against the JSON layer: wire `1` = `SpecifiedDeparture` = JSON-level `depart_at`, which the parser then shifts to proto `depart_at = 2`. The naming is not incorrect - the two-layer design is simply easy to misread.

The client always sends wire `1` (`SpecifiedDeparture`) with `Value = time.Now()` (per an in-code comment: `// hotfix: seem departureTime always nil`). From Valhalla's perspective this is therefore a `depart_at` request with a timestamp equal to wall-clock now. This is **not** the same as `current` on the wire; it only behaves the same because the value happens to be now.

**Production request shape, restated precisely:** proto `depart_at = 2`, `value = ISO-minute string of wall-clock now`, via JSON wire `type=1`.

#### Step 1 - which algorithm does Valhalla actually run?

The path-algorithm decision lives in [src/thor/route_action.cc#L409-L418](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L409-L418). Verbatim (master line numbers):

```cpp
409 if (!origin.date_time().empty() && options.date_time_type() != Options::invariant &&
410     !options.prioritize_bidirectional()) {
411   PointLL ll1(origin.ll().lng(), origin.ll().lat());
412   PointLL ll2(destination.ll().lng(), destination.ll().lat());
413   if (ll1.Distance(ll2) < max_timedep_distance) {
414     return &timedep_forward;
415   } else {
416     add_warning(request, 402);
417   }
418 }
```

The cutoff `max_timedep_distance` is set in `[scripts/valhalla_build_config` L405]([https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config#L405](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config#L405)) to **500,000 meters (500 km beeline)**. Above it, Valhalla emits warning 402 ("time dependency not supported at this distance") and falls through to the default at [line 448](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L448): `return &bidir_astar`.

Evidence-grounded, this gives a hard split by route length:


| Beeline distance | Algorithm actually run               | Fade behaviour  |
| ---------------- | ------------------------------------ | --------------- |
| < 500 km         | `timedep_forward` (unidirectional A) | Correct         |
| ≥ 500 km         | `bidir_astar` + warning 402          | Buggy per #5616 |


A 50-minute drive is 30–80 km of beeline distance, well inside the "< 500 km" band. **These routes run on `timedep_forward`, not bidirectional.**

#### Step 2 - how fade behaves in `timedep_forward` (Case A)

In unidirectional forward mode, `TimeInfo` is built from the origin's `date_time` (populated with the production timestamp) in [valhalla/baldr/time_info.h#L64-L85](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L64-L85). Because `date_time` is non-empty, `TimeInfo::make` returns a **valid** TimeInfo (L70-L71: the early `invalid()` return is skipped).

As expansion progresses, [src/thor/unidirectional_astar.cc#L79](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc#L79) calls `time_info.forward(pred.cost().secs, …)`. The forward method advances `seconds_from_now` by the predecessor's elapsed seconds (`[valhalla/baldr/time_info.h` forward() at L166-L208]([https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L166-L208](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L166-L208))).

The fade formula from [valhalla/baldr/graphtile.h#L811-L816](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L811-L816) is:

```cpp
811 constexpr double LIVE_SPEED_FADE = 1. / 3600.;
...
816 float live_traffic_multiplier = 1. - std::min(seconds_from_now * LIVE_SPEED_FADE, 1.);
```

Put together, for a 50-minute trip (3,000 s end-to-end):


| Where on the route  | Time to reach (seconds_from_now) | `live_traffic_multiplier` | Blend of live vs historical |
| ------------------- | -------------------------------- | ------------------------- | --------------------------- |
| Near origin         | 0 s                              | 1.00                      | 100% live, 0% historical    |
| 10 min in           | 600 s                            | 0.83                      | 83% live, 17% historical    |
| 25 min in (halfway) | 1,500 s                          | 0.58                      | 58% live, 42% historical    |
| 40 min in           | 2,400 s                          | 0.33                      | 33% live, 67% historical    |
| 50 min in (end)     | 3,000 s                          | 0.17                      | 17% live, 83% historical    |


This is the textbook behaviour the fade was designed for, and is what our 50-minute production routes currently receive.

#### Step 3 - where the #5616 bug lives, and when it applies

The bug has two pieces of evidence, which have to be read together:

1. **The docstring** ([valhalla/baldr/graphtile.h#L788-L793](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L788-L793)) says verbatim: *"Currently this parameter is set to 0 when building a route with reverse and bidirectional a."*
2. **The [Issue #5616](https://github.com/valhalla/valhalla/issues/5616) report** by `chrstnbwnkl` (opened Oct 17 2025, still open) describes the user-visible symptom and provides this example, verbatim:
  > "Assuming you are planning a route with a departure time more than an hour in the future: even if Valhalla has live traffic hot and loaded, unidirectional A won't use those speeds anywhere, since the time exceeds the live speed fading threshold. If you force Valhalla to use the bidirectional algorithm instead, the reverse expansion will use an invalid `TimeInfo` object which has `seconds_from_now` set to 0, and so it gets the unfaded live speed."

The reporter does NOT claim CostMatrix, map-matching, or the forward tree is affected. The scope is strictly the reverse frontier of bidirectional A and the reverse of unidirectional (`timedep_reverse`, used for `arrive_by`).

Mapping that to our production:


| Request type                                                        | Algorithm                                                                                                            | Affected by #5616?                                                         |
| ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| `date_time.type=1`, beeline < 500 km (typical ride)                 | `timedep_forward`                                                                                                    | **No.** Fade works correctly.                                              |
| `date_time.type=1`, beeline ≥ 500 km (long intercity)               | `bidir_astar` via fallback at [L416](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L416) | **Yes.** Warning 402 emitted; reverse frontier uses un-faded live traffic. |
| `date_time.type=3` (`arrive_by`) - not currently used in production | `timedep_reverse`                                                                                                    | **Yes.** Same reverse-side bug per docstring.                              |
| No `date_time` set at all (misconfigured caller)                    | `bidir_astar` default                                                                                                | **Yes.**                                                                   |
| `date_time.type=4` (`invariant`)                                    | `bidir_astar`                                                                                                        | **Yes.**                                                                   |


#### Step 4 - what the bug looks like in practice (Case B, beeline ≥ 500 km)

When the request falls through to `bidir_astar`, the reverse `TimeInfo` is constructed from the destination ([src/thor/bidirectional_astar.cc#L546-L561](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L546-L561)). In our request shape the destination has no `date_time` field, which sends `TimeInfo::make` to the invalid branch at [time_info.h#L70-L71](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L70-L71). An invalid TimeInfo has `seconds_from_now = 0`, and `.reverse()` early-returns without advancing it (see the `if (!valid) return *this;` guard at the top of `[reverse()` L212-L214]([https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L212-L214](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L212-L214))).

Net effect on the reverse frontier: `seconds_from_now = 0` on every edge, so `live_traffic_multiplier = 1.0` everywhere. The forward frontier behaves correctly; whichever frontier settles a given edge determines how that edge is priced.

For a representative HCMC ↔ Hanoi 1,653 km route, the reverse frontier settles the entire back half. That back half is priced with 100% live traffic as if the current snapshot persists for the full 18-hour drive. On a stable-traffic day this is harmless; during a rush-hour snapshot it will systematically bias the ETA.

#### Step 5 - conclusions

**For the 50-minute case:** no exposure. The request runs `timedep_forward`, `seconds_from_now` advances correctly, and the fade tapers from 1.00 at origin to 0.17 near the end. No reverse frontier is involved.

**Residual exposure is limited to:**

1. **Long-haul routes (≥ 500 km beeline).** These requests silently fall back to `bidir_astar`. Warning 402 is emitted in the response and should be logged and acted on at the API boundary.
2. **Requests that omit `date_time.type=1`.** These default to bidirectional A. The client-side contract should assert `type=1` on every outgoing request.
3. **Future scheduled-ride features.** A `depart_at` ≥ 500 km, or any `arrive_by`, lands on the buggy side of the split and should be treated as a known constraint in product specifications.

#### Monitoring and mitigation

**Recommended logging and metrics:**

- Per-request log of `options.date_time_type`, `options.date_time.value`, and the origin-to-destination beeline distance.
- Counter on Valhalla warning codes, particularly **402** (time-dependent algorithm dropped due to distance) and **214** (destination-side equivalent). A rising 402 rate is a leading indicator of Case B exposure.
- ETA-vs-actual drift bucketed by beeline distance (<100 km, 100–500 km, ≥500 km). Divergence in the ≥500 km bucket indicates the bug is materialising.
- `last_update` freshness of traffic tiles. Stale tiles worsen Case B significantly and Case A mildly.

**Mitigations, in increasing order of effort:**

1. **Assert `date_time.type = 1` on every outgoing Valhalla request.** A gateway-level contract test is sufficient.
2. **Handle warning 402 explicitly for ≥ 500 km routes.** Options: (a) strip `CURRENT` from `flow_mask` (send `flow_mask = 2`, predicted-only) to avoid the stale-live overweight; (b) accept the approximation and widen the ETA confidence band surfaced to the caller.
3. **Patch upstream via fork.** Modify `bidirectional_astar.cc` so the reverse frontier seeds its `TimeInfo` from an estimated-ETA offset (e.g. `total_route_estimate − pred.cost().secs`) rather than from destination wall-clock. Medium engineering effort, ongoing rebase cost, candidate PR against #5616.

Mitigations 1 and 2 address the majority of the exposure at low engineering cost. Mitigation 3 becomes warranted if #5616 remains open and the product begins to include scheduled-ride or >500 km routing features.

---

### 5.12 Algorithm-selection reference - `/route` vs `/sources_to_targets`

One-page reference: for every API endpoint and input shape, the exact algorithm selected and its exposure to issue #5616. Every row is verified against master; every line number is a direct link into the source.

#### 5.12.0 How `date_time` lands on individual locations (both endpoints)

Before the selector runs, `add_date_to_locations()` decides *which* location(s) get their `date_time` field populated. The routing algorithms downstream only check `origin.date_time()` / `destination.date_time()`, so this step is what actually controls which branches fire.

Function: `add_date_to_locations()` in [src/worker.cc#L111-L145](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L111-L145).


| `options.action()`       | `date_time_type` (proto) | Which locations get `date_time` set                                        | Evidence                                                                                              |
| ------------------------ | ------------------------ | -------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| not `sources_to_targets` | `current` (=1)           | **origin only**, string `"current"` (later replaced by `TimeInfo::make()`) | [src/worker.cc#L118-L120](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L118-L120) |
| not `sources_to_targets` | `depart_at` (=2)         | **origin only**, user's value string                                       | [src/worker.cc#L121-L123](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L121-L123) |
| not `sources_to_targets` | `arrive_by` (=3)         | **destination only**, user's value string                                  | [src/worker.cc#L124-L126](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L124-L126) |
| not `sources_to_targets` | `invariant` (=4)         | **all** locations, user's value                                            | [src/worker.cc#L127-L129](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L127-L129) |
| `sources_to_targets`     | `arrive_by`              | **all targets**                                                            | [src/worker.cc#L133-L139](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L133-L139) |
| `sources_to_targets`     | anything else            | **all sources**                                                            | [src/worker.cc#L133-L139](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L133-L139) |


Reminder of the JSON→proto enum shift (fully documented in §5.11 Step 0): the JSON wire's `date_time.type = 1` is converted to proto `depart_at = 2` at [src/worker.cc#L830-L832](https://github.com/valhalla/valhalla/blob/master/src/worker.cc#L830-L832). Our production request therefore enters the `depart_at` row above - origin (or all sources) receives the user's timestamp.

#### 5.12.1 `/route` - `thor_worker_t::get_path_algorithm()`

Source function: [src/thor/route_action.cc#L377-L449](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L377-L449). Branches evaluated in order; first match wins.


| #   | Costing / input condition                                                                                                                                                                                                                                    | Controlling logic (master source)                                                          | Algorithm picked                  | Affected by #5616?                                                                                         | Evidence                                                                                                                                                                                                             |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------ | --------------------------------- | ---------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | `costing == "multimodal"` or `"transit"`                                                                                                                                                                                                                     | `if (routetype == "multimodal"                                                             |                                   | routetype == "transit") return &multi_modal_transit;`                                                      | `multi_modal_transit`                                                                                                                                                                                                |
| 2   | `costing == "auto_pedestrian"`                                                                                                                                                                                                                               | `if (routetype == "auto_pedestrian") return &multimodal_astar;`                            | `multimodal_astar`                | No                                                                                                         | [route_action.cc#L398](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L398)                                                                                                             |
| 3   | `costing == "bikeshare"`                                                                                                                                                                                                                                     | `if (routetype == "bikeshare") return &multimodal_astar;`                                  | `multimodal_astar`                | No                                                                                                         | [route_action.cc#L403](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L403)                                                                                                             |
| 4   | `origin.date_time()` non-empty AND `date_time_type != invariant` AND `!prioritize_bidirectional` (gated at [L409-L410](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L409-L410)) AND beeline `< max_timedep_distance` (500 km) | distance gate `if (ll1.Distance(ll2) < max_timedep_distance)` → `return &timedep_forward;` | `timedep_forward`                 | **No** - unidirectional forward expansion, time advances correctly via `TimeInfo::forward()`               | [route_action.cc#L413](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L413) (gate) + [#L414](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L414) (return) |
| 5   | Same origin-side gate as row 4 but beeline `≥ max_timedep_distance`                                                                                                                                                                                          | `else { add_warning(request, 402); }` - no return, falls through to row 8                  | (warning 402, then `bidir_astar`) | **Yes** (via fall-through to row 8)                                                                        | [route_action.cc#L416](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L416)                                                                                                             |
| 6   | `destination.date_time()` non-empty AND `date_time_type != invariant` (gated at [L422](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L422)) AND beeline `< max_timedep_distance`                                               | distance gate `if (ll1.Distance(ll2) < max_timedep_distance)` → `return &timedep_reverse;` | `timedep_reverse`                 | **No** - unidirectional reverse expansion, time tracked via `TimeInfo::reverse()`                          | [route_action.cc#L425](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L425) (gate) + [#L426](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L426) (return) |
| 7   | Trivial connection: any origin-edge and destination-edge share a graph id or are directly connected                                                                                                                                                          | `if (same_graph_id                                                                         |                                   | are_connected) return &timedep_forward;`                                                                   | `timedep_forward`                                                                                                                                                                                                    |
| 8   | Default (no `date_time`, or `invariant`, or `prioritize_bidirectional=true`, or the rows above fell through)                                                                                                                                                 | `return &bidir_astar;`                                                                     | `bidir_astar`                     | **Yes** - #5616 "Bidirectional A* will use live traffic on the reverse expansion, regardless of date time" | [route_action.cc#L448](https://github.com/valhalla/valhalla/blob/master/src/thor/route_action.cc#L448)                                                                                                             |


**Production path (`date_time.type=1 (JSON) → depart_at, value=now`):**

- Origin gets `date_time` set (row "not-sources_to_targets / depart_at" of §5.12.0).
- `options.date_time_type() == depart_at` (not `invariant`), `prioritize_bidirectional=false` by default.
- Beeline `< 500 km` → **row 4 → `timedep_forward`, not affected by #5616.**
- Beeline `≥ 500 km` → **row 5 → warning 402 → falls through to row 8 → `bidir_astar`, affected by #5616.**

#### 5.12.2 `/sources_to_targets` - `thor_worker_t::get_matrix_algorithm()`

Source function: [src/thor/matrix_action.cc#L32-L86](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L32-L86). `has_time` is the return value of `check_matrix_time()` at [valhalla/thor/matrixalgorithm.h#L226-L253](https://github.com/valhalla/valhalla/blob/master/valhalla/thor/matrixalgorithm.h#L226-L253) (true iff at least one source or target has `date_time` set). `source_to_target_algorithm` is a server-config flag (default `select_optimal`, evidence at [src/thor/worker.cc#L78](https://github.com/valhalla/valhalla/blob/master/src/thor/worker.cc#L78) and [src/thor/worker.cc#L96-L100](https://github.com/valhalla/valhalla/blob/master/src/thor/worker.cc#L96-L100)).

Branches evaluated in order.


| #   | Costing / input condition                                                                                     | Controlling logic (master source)                                                                                                                                                  | Algorithm picked            | Affected by #5616?                                                                                                                                          | Evidence                                                                                                 |
| --- | ------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| 1   | `costing == "bikeshare"`                                                                                      | `if (costing == "bikeshare") return &time_distance_bss_matrix_;`                                                                                                                   | `time_distance_bss_matrix`_ | No - unidirectional                                                                                                                                         | [matrix_action.cc#L35](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L35) |
| 2   | `has_time == true` AND `!prioritize_bidirectional` AND server config `!= COST_MATRIX`                         | gate [L68-L69](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L68-L69) → `return &time_distance_matrix_;`                                            | `time_distance_matrix`_     | No - unidirectional                                                                                                                                         | [matrix_action.cc#L70](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L70) |
| 3   | `has_time == true` AND `prioritize_bidirectional` AND server config `!= TIME_DISTANCE_MATRIX`                 | gate [L71-L72](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L71-L72) → `return &costmatrix_;`                                                      | `costmatrix_`               | **Same class of bug as #5616** - reverse expansion uses `TimeInfo::invalid()` (see note below). Issue doesn't name it explicitly; code inspection confirms. | [matrix_action.cc#L73](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L73) |
| 4   | `config_algo == CostMatrix` (no `has_time`, or `SELECT_OPTIMAL`+auto+large, or explicit `COST_MATRIX` config) | gate [L74](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L74) → `return &costmatrix_;`; adds warning 301 if `has_time && !prioritize_bidirectional` | `costmatrix_`               | If `has_time=true`: same class of bug as #5616 on reverse side. If `has_time=false`: no live-traffic+time conflict by definition.                           | [matrix_action.cc#L78](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L78) |
| 5   | Default fallthrough - config-forced `TIME_DISTANCE_MATRIX`                                                    | `else { … return &time_distance_matrix_; }`; adds warning 300 if `has_time && prioritize_bidirectional`                                                                            | `time_distance_matrix_`     | No - unidirectional                                                                                                                                         | [matrix_action.cc#L84](https://github.com/valhalla/valhalla/blob/master/src/thor/matrix_action.cc#L84) |


**Why `costmatrix_` sits in the #5616 family** - evidence in code, not in the issue body:

- `CostMatrix::SourceToTarget()` runs two concurrent frontiers. The reverse-frontier call is made without a `TimeInfo` argument: `Expand<MatrixExpansionType::reverse>(i, n, graphreader, request.options());` at [src/thor/costmatrix.cc#L216](https://github.com/valhalla/valhalla/blob/master/src/thor/costmatrix.cc#L216).
- The declaration defaults the missing parameter to `TimeInfo::invalid()`: [valhalla/thor/costmatrix.h#L200-L205](https://github.com/valhalla/valhalla/blob/master/valhalla/thor/costmatrix.h#L200-L205) (`const baldr::TimeInfo& time_info = baldr::TimeInfo::invalid()`).
- The forward-frontier call passes the real per-source `TimeInfo`: [src/thor/costmatrix.cc#L251-L252](https://github.com/valhalla/valhalla/blob/master/src/thor/costmatrix.cc#L251-L252).
- Inside `Expand`, `time_info.reverse(...)` early-returns unchanged when `!valid`: [valhalla/baldr/time_info.h#L213-L214](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/time_info.h#L213-L214). Result: reverse-frontier `seconds_from_now == 0` on every edge, so the live-traffic fade multiplier stays at `1.0` (full live blend) regardless of how far in the future the caller asked for - exactly the pattern #5616 describes for `bidir_astar`.

**Production path (`date_time.type=1 (JSON) → depart_at, value=now`, default server config `source_to_target_algorithm=select_optimal`, `prioritize_bidirectional=false`):**

- Sources get `date_time` set (row "sources_to_targets / anything-but-arrive_by" of §5.12.0).
- `check_matrix_time()` returns `true` (at least one source has `date_time`) → `has_time = true`.
- Row 2 of §5.12.2 matches → `**time_distance_matrix`_, not affected by #5616.**
- If `prioritize_bidirectional=true` is set on the request, row 3 fires → `**costmatrix_`, same-class exposure.**
- A matrix request without any `date_time` triggers row 4 → `costmatrix_` with `has_time=false`; the reverse-side-invalid-TimeInfo behaviour then becomes moot (live fade uses `seconds_from_now=0`, which is equivalent to the default time-less path - no user-observable drift from the bug class).

#### 5.12.3 Summary - production exposure matrix


| Call shape                                                                            | Selected algorithm                          | #5616 exposure                                               |
| ------------------------------------------------------------------------------------- | ------------------------------------------- | ------------------------------------------------------------ |
| `/route`, `date_time.type=1`, beeline < 500 km                                        | `timedep_forward`                           | **Safe**                                                     |
| `/route`, `date_time.type=1`, beeline ≥ 500 km                                        | `bidir_astar` (via warning 402)             | **Exposed**                                                  |
| `/sources_to_targets`, `date_time.type=1`, `prioritize_bidirectional=false` (default) | `time_distance_matrix_`                     | **Safe**                                                     |
| `/sources_to_targets`, `date_time.type=1`, `prioritize_bidirectional=true`            | `costmatrix_`                               | **Same-class exposure** (reverse uses `TimeInfo::invalid()`) |
| `/sources_to_targets`, no `date_time`                                                 | `costmatrix_` (default SELECT_OPTIMAL+auto) | **Not user-observable** (`has_time=false`)                   |


---

## 6. Putting It All Together - One Request Lifecycle

Example: `POST /route?costing=auto&date_time.type=2&value=2026-04-22T18:00`:

1. **Tyr** (the service frontend) receives HTTP, validates JSON, forwards.
2. **Loki** (input handling) snaps origin and destination to the nearest edges via the spatial bin index. In the reference debug log: origin snapped to 2 candidate edges, destination to 1.
3. **Thor** (the pathfinder) runs the decision tree from §4.0:
  - `date_time_type = 2` (depart_at) → unidirectional A (see §4.5 for why bidirectional is off)
  - Without `date_time_type` → bidirectional A
4. For each popped edge:
  - `DynamicCost::EdgeCost()` asks `GraphTile::GetSpeed()` for this edge's speed right now.
  - `GetSpeed` blends live and historical via the fade formula from §5.6.
  - Converts speed to seconds to cost, adds transition cost at nodes (turn penalty etc.).
5. Hierarchy pruning (`StopExpanding`) gates which tiles are opened.
6. Paths meet, budget exceeded, and `FormPath()` recovers the predecessor chain.
7. `RecoverShortcut()` expands L0/L1 shortcuts back into real OSM edges.
8. **Odin** generates maneuvers and narration from the final sequence.
9. The service returns a GeoJSON shape plus summary.

**Module-name mnemonic** (useful when reading Valhalla source):


| Module  | Norse god   | What it does                                                   |
| ------- | ----------- | -------------------------------------------------------------- |
| Loki    | trickster   | Input handling - correlate lat/lon to graph edges              |
| Thor    | strength    | Pathfinder - A, bidirectional A, Dijkstra, matrix, isochrone   |
| Sif     | Thor's wife | Costing - dynamic cost models (auto, bike, pedestrian, truck…) |
| Odin    | wisdom      | Narrator - turn-by-turn maneuvers                              |
| Meili   | matchmaker  | Map-matching - snap GPS traces to roads                        |
| Mjolnir | hammer      | Tile builder - offline graph preprocessing from OSM            |
| Skadi   | hunt        | Elevation - terrain queries                                    |
| Tyr     | war         | Service frontend - HTTP layer                                  |


---

## 7. How Valhalla compares to the other two engines

Context for anyone asking "why not OSRM or GraphHopper?"

### 7.1 Head-to-head


| Property                             | Valhalla                 | OSRM                           | GraphHopper                |
| ------------------------------------ | ------------------------ | ------------------------------ | -------------------------- |
| Language                             | C++                      | C++                            | Java                       |
| Graph model                          | Tiled, 3-level hierarchy | Contraction Hierarchies (flat) | CH + hybrid speed/flexible |
| Planet RAM                           | 8–12 GB                  | 35–64 GB                       | 16–32 GB                   |
| Runs on mobile?                      | Yes                      | No                             | No                         |
| Route latency (single pair)          | 20–50 ms                 | 5–15 ms                        | 30–80 ms                   |
| Matrix throughput                    | ~15 req/s                | ~200 req/s                     | ~50 req/s                  |
| Dynamic costing (per-request params) | Yes                      | No                             | Partial                    |
| Live traffic overlay                 | Yes, native              | No (requires rebuild)          | No (paid tier only)        |
| Licence                              | MIT (open)               | BSD-2 (open)                   | Apache-2 (open)            |


### 7.2 The matrix throughput gap (~13× slower than OSRM)

OSRM uses **Contraction Hierarchies** - an offline preprocessing step that pre-computes shortcuts at every node level and gives roughly O(log n) queries for a fixed costing. That pre-computation is what makes matrix queries so cheap, but it also **freezes the costing model** - you cannot pass per-request parameters (no "avoid highways," no "max_height=2.1 m").

Valhalla uses **Dynamic Costing** - costs are evaluated at search time using the request's parameters. Flexible, but you pay for it on every matrix cell.

**Operational trade-off:** for dispatch workloads that run large N×M matrices (driver ↔ pickup distance grids), Valhalla's matrix endpoint is the bottleneck. Mitigations include heavier batching, aggressive caching, or running a parallel OSRM instance specifically for symmetric-matrix workloads.

### 7.3 Why Mapbox and Tesla picked Valhalla

- **Mapbox (2018):** migrated from OSRM to Valhalla. Cited low RAM footprint (fits in a small instance) and dynamic costing (support multiple profiles without rebuilding).
- **Tesla FSD v14:** on-device routing uses Valhalla tiles because the three-level hierarchy fits in car memory and `mmap` keeps it cheap.
- **Uber:** built their own routing (Gurafu) but adopted Valhalla's tiled model as inspiration.

The decisive factors for selection were the same: mobile and edge-friendly footprint, runtime flexibility, and live-traffic overlay without rebuilding.

---

## 8. Open engineering topics

Items that are out of scope for this reference but should be tracked:

1. **Issue #5616 mitigation path.** Options range from API-layer gating (force unidirectional when `depart_at > now + 1h`) to an upstream fork. See §5.11 for evidence and recommended sequencing.
2. `**last_update` freshness monitoring per tile.** A stale traffic tile silently degrades route quality. `last_update` is a `uint64_t` epoch in every `TrafficTileHeader` - inexpensive to scrape into a metrics pipeline.
3. **Live-traffic coverage dashboard.** Edges without a live signal are rendered dim grey in the tile browser; a coverage-by-road-class-by-region view would make blind spots visible to Operations.
4. `**threshold_delta` tuning.** The default `420` cost units for bidirectional has not been tuned for local road characteristics. Higher = slower but more optimal; lower = faster but can miss better routes across shortcut boundaries.
5. **Matrix throughput.** OSRM's CH-based approach is approximately 13× faster on matrix queries. A parallel read-only OSRM instance dedicated to dispatch matrices may be worth evaluating.
6. **Hierarchy-limit relaxation telemetry.** `RelaxHierarchyLimits()` in [valhalla/sif/hierarchylimits.h](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h) multiplies limits on retry. Logging this event identifies geographies where default limits are too tight.
7. **ETA-drift segmentation.** ETA-vs-actual drift bucketed by route duration (short/medium/long) is the most direct way to surface #5616's impact in production.

---

## 9. Configuration Surface - Routing, Live Traffic, ETA

Three knob groups, each affecting request behaviour at a different layer. All defaults below are sourced from [scripts/valhalla_build_config](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config) (the canonical defaults generator) and the costing source files. Line numbers reference the local cached copy of `valhalla_build_config` retrieved from master; the upstream file is the same structure.

### 9.1 Routing & hierarchy


| Knob                                                                | Default                        | What it controls                                                                                                                                                                 | Evidence                                                                                                                    |
| ------------------------------------------------------------------- | ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `thor.source_to_target_algorithm`                                   | `"select_optimal"`             | Matrix algorithm selector. Values: `select_optimal` / `costmatrix` / `timedistancematrix`                                                                                        | [scripts/valhalla_build_config#L214](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `thor.bidirectional_astar.hierarchy_limits.max_up_transitions`      | `{1: 400, 2: 100}`             | Bidir A - max allowed up-transitions per level. Caps hierarchy climbing.                                                                                                         | [scripts/valhalla_build_config#L241-L244](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config) |
| `thor.bidirectional_astar.hierarchy_limits.expand_within_distance`  | `{0: 1e8, 1: 20000, 2: 5000}`  | Bidir A - distance from origin/destination within which each level is expanded. Below 5 km the search stays on L2; 5–20 km allows L1; above 20 km allows L0.                     | [scripts/valhalla_build_config#L245](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `thor.unidirectional_astar.hierarchy_limits.expand_within_distance` | `{0: 1e8, 1: 100000, 2: 5000}` | Unidirectional A (time-dependent forward/reverse) - L1 up to 100 km vs Bidir's 20 km. Unidirectional keeps arterial detail longer because there is no opposing frontier to meet. | [scripts/valhalla_build_config#L254](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `thor.costmatrix.hierarchy_limits.expand_within_distance`           | `{0: 1e8, 1: 20000, 2: 5000}`  | Matrix (CostMatrix) - same shape as Bidir A.                                                                                                                                     | [scripts/valhalla_build_config#L233](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `service_limits.max_timedep_distance`                               | `500000` (m)                   | Beeline cutoff beyond which a time-dependent `/route` falls back to Bidir A and emits warning 402. Directly gates #5616 exposure.                                                | [scripts/valhalla_build_config#L405](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `service_limits.max_distance_disable_hierarchy_culling`             | `0`                            | Max beeline distance at which hierarchy culling can be disabled. `0` means hierarchy pruning is always on in production.                                                         | [scripts/valhalla_build_config#L411](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `service_limits.hierarchy_limits.allow_modification`                | `false`                        | Whether client requests can override hierarchy limits. Closed by default.                                                                                                        | [scripts/valhalla_build_config#L413](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `mjolnir.reclassify_links`                                          | `true`                         | Reclassifies ramps to the lowest connecting road class during tile build. Affects which ramps survive hierarchy pruning.                                                         | [scripts/valhalla_build_config#L161](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `mjolnir.shortcuts`                                                 | `true`                         | Builds shortcut edges on L0 and L1 during tile build. Turning this off disables the hierarchical algorithm entirely.                                                             | [scripts/valhalla_build_config#L149](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |


**Thor ↔ Loki connection.** The `expand_within_distance` levels are beeline distances measured from the current expanding location, not route distance. See §10.3 for how this interacts with candidate selection.

### 9.2 Live traffic pipeline

Live traffic enters the routing core through the `traffic.tar` extract, is read by a second `mmap`, and is blended into edge speed by `GraphTile::GetSpeed`.


| Knob / constant                                 | Value                                            | Role                                                             | Evidence                                                                                                                    |
| ----------------------------------------------- | ------------------------------------------------ | ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `mjolnir.traffic_extract`                       | `/data/valhalla/traffic.tar`                     | Path to the live traffic extract. Memory-mapped read-only.       | [scripts/valhalla_build_config#L136](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)      |
| `mjolnir.incident_dir` / `mjolnir.incident_log` | Optional                                         | Incident tile directory and change log. Null in our deployment.  | [scripts/valhalla_build_config#L137-L138](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config) |
| `kDefaultFlowMask`                              | `kFreeFlow                                       | kConstrainedFlow                                                 | kPredictedFlow                                                                                                              |
| `LIVE_SPEED_FADE`                               | `1/3600`                                         | Coefficient of the 1-hour linear fade on live-traffic weight.    | [valhalla/baldr/graphtile.h#L811](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L811)       |
| `live_traffic_multiplier`                       | `1 − min(seconds_from_now × LIVE_SPEED_FADE, 1)` | Weight applied to live speed. Zero at `seconds_from_now ≥ 3600`. | [valhalla/baldr/graphtile.h#L816](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L816)       |


**Speed-source fallback chain** in [valhalla/baldr/graphtile.h#L796-L903](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L796-L903) (invoked by every costing call):

1. **Live (kCurrentFlowMask)** - applied if the tile has a live speed for this edge, the speed is valid and non-zero, and `live_traffic_multiplier > 0`. Faded per above. [L820-L852](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L820-L852)
2. **Predicted (kPredictedFlowMask)** - applied if a time was passed in and the edge has a predicted-speed profile. Blended with any partial live coverage. [L857-L864](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L857-L864)
3. **Constrained flow (kConstrainedFlowMask)** - applied during daytime, defined as `25200 < seconds < 68400` (07:00–19:00 local), or if no time was supplied. [L870-L876](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L870-L876)
4. **Free flow (kFreeFlowMask)** - applied outside 07:00–19:00 or if no time was supplied. [L886-L891](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L886-L891)
5. **Static fallback** - `de->speed()` (or `de->truck_speed()` for truck mode). [L900-L902](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L900-L902)

**#5616-class exposure recap.** Any algorithm that calls `GetSpeed` with `seconds_from_now = 0` on a frontier that is *not* actually the current moment will apply full live weight (multiplier = 1.0). Confirmed sites on master:

- Bidir A reverse expansion: evidence already documented in §5.10–§5.11.
- CostMatrix reverse expansion: `Expand<reverse>` invoked without `time_info` → defaults to `TimeInfo::invalid()` → `seconds_from_now` is never set above 0 on the reverse tree. Same class of bug, same code path category. [src/thor/costmatrix.cc#L216](https://github.com/valhalla/valhalla/blob/master/src/thor/costmatrix.cc#L216), [valhalla/thor/costmatrix.h#L200-L205](https://github.com/valhalla/valhalla/blob/master/valhalla/thor/costmatrix.h#L200-L205).

### 9.3 ETA / speed pipeline

Edge speed is determined at **two distinct times**: tile build (`de->speed()`, the static default) and request time (`GetSpeed` fallback chain above). The static default is where country-aware calibration happens; the request-time chain is where live/predicted/constrained overlays come in.


| Knob                                              | Default                             | Role                                                                                                                                                                                    | Evidence                                                                                                                                                                                                                                                                                                                                                         |
| ------------------------------------------------- | ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mjolnir.default_speeds_config`                   | Optional (unset)                    | Path to a JSON config that tells the graph enhancer how to set `de->speed()` per country × urban/rural × road class × form-of-way. If unset, the enhancer uses built-in heuristics.     | [scripts/valhalla_build_config#L162](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                                                                                           |
| `mjolnir.data_processing.apply_country_overrides` | `true`                              | Applies country-specific access and turn-lane overrides during tile build. Uses the admin DB.                                                                                           | [scripts/valhalla_build_config#L166](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), applied at [src/mjolnir/graphenhancer.cc#L972](https://github.com/valhalla/valhalla/blob/master/src/mjolnir/graphenhancer.cc#L972) and [#L1165](https://github.com/valhalla/valhalla/blob/master/src/mjolnir/graphenhancer.cc#L1165) |
| `mjolnir.data_processing.use_urban_tag`           | `false`                             | When false, the enhancer computes density via `GetDensity()`; when true, it trusts the OSM urban tag. Urban vs rural drives the default-speeds lookup.                                  | [scripts/valhalla_build_config#L171](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                                                                                           |
| Auto `top_speed`                                  | `kMaxAssumedSpeed = 140 kph`        | Upper bound applied per-request at [src/sif/autocost.cc#L500-L506](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L500-L506). Any edge speed is capped by this. | [valhalla/baldr/graphconstants.h#L97](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphconstants.h#L97), range [src/sif/autocost.cc#L78-L79](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L78-L79)                                                                                                               |
| Auto `fixed_speed`                                | `kDisableFixedSpeed = 0` (disabled) | When > 0, overrides all edge-speed logic with a single speed for the whole request. Useful for planning, breaks live traffic.                                                           | [valhalla/baldr/graphconstants.h#L116](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphconstants.h#L116)                                                                                                                                                                                                                                  |
| `service_limits.auto.max_distance`                | `5,000,000` (m)                     | Hard cap on the beeline distance between all request locations.                                                                                                                         | [scripts/valhalla_build_config#L312](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                                                                                           |


**What is country-aware vs global.** The admin DB is country-aware: `drive_on_right` is per-node (set by country polygon), turn-lane parsing uses country code, and the default-speeds JSON can key on country. Costing-level constants (all the `kDefault`* and `kTC`* values in §11.2) are **global compile-time constants** - they do not vary by country.

---

## 10. Location Radius - how Loki picks candidate edges

The `radius` supplied on each input location drives Loki's candidate search. The interaction between radius, `search_cutoff`, and `minimum_reachability` decides what actually gets correlated to the graph.

### 10.1 Defaults and hard caps


| Parameter                                        | Default | Hard cap | Meaning                                                                                                                                                           | Evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ------------------------------------------------ | ------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `loki.service_defaults.radius`                   | `0`     | -        | Radius (m) applied when the client does not supply one. `0` means "point search" - only the single closest edge is considered within-radius.                      | [scripts/valhalla_build_config#L199](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), applied at [src/loki/loki_worker.cc#L50-L51](https://github.com/valhalla/valhalla/blob/master/src/loki/loki_worker.cc)                                                                                                                                                                                                                      |
| `service_limits.max_radius`                      | `200`   | hard     | Hard cap on any client-supplied radius. Requests above this are clamped.                                                                                          | [scripts/valhalla_build_config#L404](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), clamp at [src/loki/loki_worker.cc#L52](https://github.com/valhalla/valhalla/blob/master/src/loki/loki_worker.cc)                                                                                                                                                                                                                            |
| `loki.service_defaults.search_cutoff`            | `35000` | -        | Max distance (m) from the input point to a candidate edge. Beyond this Loki gives up on the point.                                                                | [scripts/valhalla_build_config#L201](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), applied at [src/loki/loki_worker.cc#L64-L66](https://github.com/valhalla/valhalla/blob/master/src/loki/loki_worker.cc), enforced at [src/loki/search.cc#L237-L239](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc#L237-L239) and [#L290](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc#L290) |
| `loki.service_defaults.node_snap_tolerance`      | `5`     | -        | If the projected snap point is within this distance (m) of a graph node, snap to the node instead of along the edge.                                              | [scripts/valhalla_build_config#L202](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), applied at [src/loki/loki_worker.cc#L58-L59](https://github.com/valhalla/valhalla/blob/master/src/loki/loki_worker.cc)                                                                                                                                                                                                                      |
| `loki.service_defaults.street_side_tolerance`    | `5`     | -        | If the input is within this distance (m) of the edge centreline, side-of-street is left undetermined rather than set to left/right.                               | [scripts/valhalla_build_config#L203](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), applied at [src/loki/loki_worker.cc#L79-L80](https://github.com/valhalla/valhalla/blob/master/src/loki/loki_worker.cc)                                                                                                                                                                                                                      |
| `loki.service_defaults.street_side_max_distance` | `1000`  | -        | Beyond this distance from the centreline, side-of-street is also left undetermined.                                                                               | [scripts/valhalla_build_config#L204](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                                                                                                                                                                                                |
| `loki.service_defaults.heading_tolerance`        | `60`    | -        | Degrees of tolerance when the client supplies a heading. Candidates whose bearing is outside ± this are de-prioritised.                                           | [scripts/valhalla_build_config#L205](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                                                                                                                                                                                                |
| `loki.service_defaults.minimum_reachability`     | `50`    | -        | Number of nodes a candidate edge must reach (inbound OR outbound) before it is classified as "reachable". Candidates below this go into the "unreachable" bucket. | [scripts/valhalla_build_config#L200](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config), applied at [src/loki/search.cc#L640-L641](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc#L640-L641)                                                                                                                                                                                                                    |
| `service_limits.max_reachability`                | `100`   | hard     | Hard cap on any client-supplied reachability.                                                                                                                     | [scripts/valhalla_build_config#L403](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                                                                                                                                                                                                |


### 10.2 In-radius vs reachable - the bucketing logic

`bin_handler_t` in [src/loki/search.cc](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc) bins candidate edges into two lists per input location: **reachable** and **unreachable**. The decision algorithm is:

```
for each candidate edge within search_cutoff:
    compute directed_reach (inbound, outbound) via Reach expansion    # L489
    reachable  = (outbound >= min_outbound) && (inbound >= min_inbound)  # L640-L641
    in_radius  = sq_distance < sq_radius                               # L671
    better     = sq_distance < back-of-bucket distance                 # L672
    closer_external_reachable = reachable && sq_distance < best_external  # L679-L680
    keep if in_radius || better                                        # L683
```

**Key consequences:**

- With `radius = 0` (server default), `in_radius` is only true for an edge whose projection sits exactly on the input point. Every other candidate must be "better" (closer than the current back-of-bucket) to be kept. This is acceptable when GPS error is small (EU/US open-sky, ~3–8 m) but marginal in urban canyons with multi-metre drift.
- With a non-zero radius, *all* edges inside the radius are kept regardless of reach. The service then prefers reachable over unreachable, but unreachable is used only if there is no reachable candidate. [src/loki/search.cc#L232-L239](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc#L232-L239).
- `minimum_reachability = 50` is the gate that determines which bucket. An edge attached to a tiny disconnected island of nodes (< 50) is marked unreachable even if it is physically the closest. This is how alley false-starts are prevented - and also how genuine alley destinations get dropped.
- Opposing-edge swap: if an edge is not reachable but its opposing is, Loki swaps to the opposing side automatically. [src/loki/search.cc#L643-L653](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc#L643-L653).

### 10.3 Hierarchy and shortcut behaviour at snap time

**Loki always searches on Level 2 (local) tiles.** Candidate bins are filtered to L2 edges only. L0 (highway) and L1 (arterial) tiles are **not** candidates - they are reachable only by transition-up from L2 during the Thor path search.

**Thor's hierarchy-climb is distance-gated.** At the origin and destination, the search is restricted to L2 for the first `expand_within_distance[2]` metres of beeline radius (5,000 m default in every algorithm). From 5 km to 20 km (Bidir / CostMatrix) or 100 km (unidirectional A), L1 is also open. Above that, L0 opens and shortcut edges become eligible. See [§9.1](#91-routing--hierarchy).

**Consequences for short routes:** a < 5 km route never touches shortcuts or highways in the graph search. Every edge is a local-tile edge, so the entire route uses local detail. Short-route ETA is therefore unaffected by `thor.*.hierarchy_limits` - those knobs are only load-bearing above 5 km.

**Consequences for long routes:** the first ~5 km from origin and ~5 km before destination are forced through local roads, regardless of how close to a highway on-ramp the input point is. Combined with `node_snap_tolerance = 5`, inputs very near a ramp will snap to the ramp itself rather than to the main road. In practice this is why ETAs for routes starting at the base of an elevated highway (common in HCM / HN) depend heavily on which ramp Loki picks - the first ramp on the list becomes the start of the L0 climb.

### 10.4 Real-service impact - snap quality and shortcut scope

**Snap quality issues that trace to radius configuration:**

1. **Wrong-side-of-road** - with `street_side_tolerance = 5`, side-of-street is left undetermined for any point within 5 m of the centreline. Urban GPS error in HCM / HN commonly exceeds 5 m (tall building canyons, overhead wires), so the majority of side-of-street decisions are "unknown", which defaults to routing over the centreline. Result: pickup/drop-off on the wrong side is a coin flip.
2. **Snap-to-service-road** - in residential districts the service roads and driveways are L2 edges with `service_penalty = 75 s` applied only at *transition*, not at *snap*. If Loki picks a service-road edge because it is nominally closer, the cost of the service snap is not penalised in the candidate phase (it is only in Thor's expansion). This produces routes that begin with "turn onto service road" maneuvers that a human would never drive.
3. **Snap-to-ferry / tunnel** - `use_ferry = 0.5` penalises ferry *use*, not ferry *snap*. Inputs near a ferry dock can snap directly onto a ferry edge as the nominal closest candidate.
4. **Hẻm false-negatives** - an alley attached to < 50 graph nodes is reachability-filtered at `minimum_reachability = 50`. The correct snap exists in the tile but is rejected; the client sees "no route" or a route starting 80 m away on the nearest through-street.

**Shortcut-scope observations:**

- **Snap-induced cliffs.** When an input point lies close to the 5 km threshold, whether L1 is allowed during origin expansion depends on the beeline distance to the *eventual* destination. Two requests with endpoints only metres apart can produce materially different routes if one tips across the 5 km boundary - the farther one is allowed L1 at origin, the closer one is not.
- **Ramp-snap asymmetry.** With `node_snap_tolerance = 5`, an input within 5 m of a ramp node snaps to the ramp. Inside the 5 km L2-only zone, the ramp is still treated as a local edge: the search must traverse it at L2 speed for the remaining pre-climb distance before transitioning up. Small positional differences on either side of a ramp node therefore produce noticeably different ETAs.
- **Alley-to-arterial transitions.** Because shortcuts exist only on L0/L1, routes that start in an alley climb to L1 at 5 km and L0 at 20 km regardless of whether a shortcut could have been used earlier. The shortcut layer is purely a mid-route acceleration - it does not influence origin-area routing.

---

## 11. Default Configuration Fit - EU / US vs Vietnam (HCM, HN)

Valhalla's shipped defaults are calibrated against OSM data and driving behaviour that most closely resembles Western Europe and the United States. Several of these defaults diverge from observed driving conditions in Ho Chi Minh City and Hanoi. This section identifies those defaults, proposes override *ranges* with rationale, and lists the analytics signals required to pick a point value inside each range.

### 11.1 Which defaults carry an EU / US calibration bias


| Default                                      | Value                                                                                                                                                                                                                                                                                       | Why this is EU / US biased                                                                                                                                                                                                                                                                         | Code Impact (If Changed)                                                                                                                                                                                                                           | Vietnamese Translation (Professional)                                                                                                                                                                                                                                                                                                                           |
| -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `kDefaultUseHighways`                        | `0.5` [src/sif/autocost.cc#L33](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L33)                                                                                                                                                                                 | In Vietnam "highway" in the OSM sense includes expressways (Cao tốc, tolled) and Highway-1 class (trunk, free). `use_highways = 0.5` does not distinguish the two.                                                                                                                                 | **If 0:** Adds massive cost multipliers to highway edges, effectively banning them unless no other route exists.<br>**If 1:** Removes all penalties, making the router aggressively prefer highways even if it means a detour.     | Trong hệ thống OSM tại Việt Nam, thẻ "highway" bao gồm cả đường cao tốc (có thu phí) và quốc lộ (đường trục chính, miễn phí). Mức mặc định `0.5` không phân biệt được hai loại đường này, dẫn đến việc định tuyến thiếu chính xác.                                                                                                              |
| `kDefaultUseTolls`                           | `0.5` [src/sif/autocost.cc#L34](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L34)                                                                                                                                                                                 | Toll roads in Vietnam are a smaller share of the network than in Western Europe; the neutral value is higher than most drivers' actual preference for paid routes.                                                                                                                                 | **If 0:** Applies maximum penalty to toll roads, avoiding them entirely.<br>**If 1:** Treats toll roads exactly like free roads, routing through them if they are even slightly faster.                                            | Tỷ lệ đường thu phí tại Việt Nam thấp hơn nhiều so với Tây Âu. Mức mặc định trung lập (0.5) đang cao hơn so với thói quen thực tế của tài xế Việt Nam, vốn thường ưu tiên các tuyến đường miễn phí.                                                                                                                                             |
| `kDefaultServicePenalty` (Auto)              | `75 s` [src/sif/autocost.cc#L30](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L30) (overrides the base `15 s` at [src/sif/dynamiccost.cc#L94](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L94))                                     | Service roads in the US / EU are parking aisles and delivery lanes - correctly rarely used. In Vietnam, service-tagged roads include large categories of parking-lot access and residential shortcuts that are routinely driven.                                                                   | N/A (Penalty in seconds)                                                                                                                                                                                                           | Tại Mỹ/Châu Âu, đường dịch vụ (service road) thường là lối đi trong bãi đỗ xe hoặc làn giao hàng nên hiếm khi được sử dụng. Ngược lại, ở Việt Nam, loại đường này bao gồm nhiều lối tắt trong khu dân cư và ngõ ngách mà xe cộ lưu thông thường xuyên.                                                                                        |
| `kDefaultAlleyPenalty`                       | `5 s` [src/sif/dynamiccost.cc#L84](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L84) with `kDefaultAlleyFactor = 1.0` [src/sif/autocost.cc#L56](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L56) - "Do not avoid alleys by default" | Alleys (ngõ / hẻm) in HN / HCM are density-constrained, slow, and frequently one-way-in-practice; the default does not discourage them.                                                                                                                                                            | N/A (Penalty in seconds)                                                                                                                                                                                                           | Ngõ/hẻm tại Hà Nội và TP.HCM thường rất hẹp, di chuyển chậm và thực tế hay hoạt động như đường một chiều. Mức phạt mặc định quá thấp không đủ để hạn chế thuật toán định tuyến vào các khu vực này.                                                                                                                                             |
| `kDefaultUseLivingStreets`                   | `0.1` [src/sif/dynamiccost.cc#L100](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L100)                                                                                                                                                                         | Living-street class in Vietnam overlaps with residential lanes that are legitimately used as through-routes; the strong-avoid bias is appropriate in EU but over-restrictive in VN urban cores.                                                                                                    | **If 0:** Strictly avoids living streets by applying a high penalty.<br>**If 1:** Treats living streets as normal roads, routing through them freely without any avoidance bias.                                                   | Loại đường sinh hoạt (living street) ở Việt Nam thường trùng với các đường nội bộ khu dân cư nhưng vẫn được dùng làm đường lưu thông chính. Việc hạn chế mạnh loại đường này phù hợp với Châu Âu nhưng lại quá khắt khe đối với các khu vực lõi đô thị tại Việt Nam.                                                                          |
| `kRightSideTurnCosts` array                  | Hard-coded: Straight 0.5, Favorable (right) 1.0, Crossing 2.0, Unfavorable (left) 2.5, Reverse (U-turn) 9.5 [src/sif/autocost.cc#L62-L64](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L62-L64)                                                                   | Fixed 2.5× cost for left turns. This is calibrated for arterial traffic with protected signals - it under-penalises left turns at un-signalised HCM / HN intersections where wait times can be 30–60 s. The arrays are `constexpr`; they are **not** runtime-configurable without a source change. | N/A (Hard-coded array)                                                                                                                                                                                                             | Hệ thống áp dụng mức phạt cố định gấp 2,5 lần cho việc rẽ trái, vốn được tối ưu cho các giao lộ có đèn tín hiệu. Điều này phạt quá nhẹ đối với các ngã rẽ trái không có đèn tín hiệu tại TP.HCM/Hà Nội. Các thông số này được cấu hình cứng (`constexpr`) và không thể thay đổi nếu không can thiệp vào mã nguồn.                             |
| `kTCRamp / kTCRoundabout`                    | Additive `+1.5` / `+0.5` [src/sif/autocost.cc#L49-L50](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L49-L50)                                                                                                                                                      | Roundabout cost +0.5 s is low for VN roundabouts which routinely back up under mixed-traffic conditions.                                                                                                                                                                                           | N/A (Penalty in seconds)                                                                                                                                                                                                           | Mức phạt thêm 0,5 giây cho vòng xuyến là quá thấp so với thực tế giao thông hỗn hợp tại Việt Nam, nơi các vòng xuyến thường xuyên xảy ra ùn tắc.                                                                                                                                                                                                |
| `meili.auto.turn_penalty_factor`             | `200` [scripts/valhalla_build_config#L293](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                | Map-matching turn penalty calibrated on car-dominant traces. VN trace density (dominant motorbikes, frequent side-street weaves) produces more apparent turns per km; a factor of 200 over-penalises legitimate turn sequences and can produce matched routes that "straighten" real paths.        | N/A (Map-matching factor)                                                                                                                                                                                                          | Thuật toán khớp bản đồ (map-matching) được tối ưu cho dữ liệu GPS của ô tô. Tại Việt Nam, với đặc thù xe máy đông đúc và thường xuyên luồn lách, hệ số 200 sẽ phạt quá nặng các ngã rẽ hợp lệ, dẫn đến việc thuật toán tự động "nắn thẳng" sai lệch so với quỹ đạo di chuyển thực tế.                                                           |
| `loki.service_defaults.radius`               | `0` [scripts/valhalla_build_config#L199](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                  | Point search assumes sub-5-m GPS accuracy. HCM / HN urban canyons regularly produce 10–20 m drift.                                                                                                                                                                                                 | N/A (Search radius in meters)                                                                                                                                                                                                      | Mặc định thuật toán tìm kiếm điểm bắt (snap) giả định sai số GPS dưới 5m. Tuy nhiên, hiệu ứng hẻm núi đô thị (tòa nhà cao tầng che khuất) tại TP.HCM/Hà Nội thường gây ra sai số định vị từ 10-20m. (Lưu ý: Radius ở đây là bán kính tìm kiếm quanh tọa độ GPS để bắt vào đoạn đường gần nhất, tương tự location radius). |
| `loki.service_defaults.search_cutoff`        | `35000` (m) [scripts/valhalla_build_config#L201](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                          | 35 km is an EU-rural "we might be far from a road" cutoff. In HCM / HN any valid snap is within a few hundred metres; the large cutoff is wasted search budget and a latency hazard on degraded-GPS inputs.                                                                                        | N/A (Distance cutoff in meters)                                                                                                                                                                                                    | Mức giới hạn 35km chỉ phù hợp với các vùng nông thôn hẻo lánh ở Châu Âu. Tại TP.HCM/Hà Nội, mọi điểm bắt hợp lệ đều nằm trong bán kính vài trăm mét. Việc để giới hạn quá lớn gây lãng phí tài nguyên tính toán và tăng độ trễ khi tín hiệu GPS kém.                                                                                          |
| `loki.service_defaults.node_snap_tolerance`  | `5` (m) [scripts/valhalla_build_config#L202](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                              | Small-block Vietnamese urban grids place node spacing close to 5 m in dense districts; snap-to-node flips fire too eagerly, attaching the request to an intersection rather than the intended edge.                                                                                                | N/A (Tolerance in meters)                                                                                                                                                                                                          | Lưới đường đô thị tại Việt Nam rất dày đặc, khoảng cách giữa các nút giao có thể chỉ khoảng 5m. Mức dung sai này khiến hệ thống quá dễ dàng bắt tọa độ vào các nút giao (ngã tư) thay vì đoạn đường thực tế mà người dùng đang đi.                                                                                                            |
| `loki.service_defaults.minimum_reachability` | `50` [scripts/valhalla_build_config#L200](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                                 | A hẻm cluster of < 50 nodes is filtered out. Many legitimate destination hẻm in HCM / HN sit below this threshold.                                                                                                                                                                                 | N/A (Node count threshold)                                                                                                                                                                                                         | Hệ thống sẽ loại bỏ các cụm đường có dưới 50 nút giao. Tuy nhiên, rất nhiều con hẻm cụt hợp lệ tại TP.HCM/Hà Nội có quy mô nhỏ hơn ngưỡng này, dẫn đến việc không thể định tuyến đến đích.                                                                                                                                                      |
| `thor.*.expand_within_distance[2]`           | `5000` (m) [scripts/valhalla_build_config#L233,L245,L254](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                                                                                                                                                 | The 5 km L2-only zone is reasonable for dispersed EU / US suburbs; in HCM the urban core extends > 10 km and L1 entry at 5 km can force premature arterial climb on intra-city trips.                                                                                                              | N/A (Distance threshold in meters)                                                                                                                                                                                                 | Vùng tìm kiếm 5km chỉ dùng đường cấp 2 (L2) phù hợp với các vùng ngoại ô phân tán ở Âu/Mỹ. Tại TP.HCM, khu vực lõi đô thị rộng hơn 10km, việc chuyển sang đường cấp 1 (L1) từ mốc 5km sẽ ép thuật toán định tuyến ra các đường huyết mạch quá sớm đối với các chuyến đi nội thành.                                                              |


### 11.2 Proposed Vietnam overrides

Confidence: **High** = code-verified mechanism and direction clear; **Medium** = mechanism verified, magnitude uncertain; **Low** = directionally plausible but depends on data we do not yet have.


| Knob                                                    | Default          | Proposed VN range                                                                                                    | Rationale                                                                                                                                                                                                                                                                                                       | Vietnamese Translation                                                                                                                                                                                                                                                                                          | Evidence                                                                                                                                             | Conf.                                            |
| ------------------------------------------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `service_penalty` (auto)                                | `75 s`           | `90 – 150 s`                                                                                                         | Service roads include large parking-aisle categories misused as shortcuts. Raising penalty pushes Thor onto residential through-roads. Not lower than baseline: true service-road destinations must still be reachable.                                                                                         | Đường dịch vụ bao gồm các danh mục lối đi bãi đậu xe lớn bị sử dụng sai làm lối tắt. Tăng hình phạt đẩy Thor sang các đường qua khu dân cư. Không thấp hơn mức cơ sở: các điểm đến đường dịch vụ thực sự vẫn phải có thể tiếp cận được.                                                                                         | [src/sif/autocost.cc#L30](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L30)                                                | High                                             |
| `alley_penalty`                                         | `5 s`            | `60 – 120 s`                                                                                                         | Ngõ / hẻm are legal but slow; routing through them should be a last resort, not a neutral option. Upper bound keeps destination alleys reachable.                                                                                                                                                               | Ngõ / hẻm là hợp pháp nhưng chậm; việc định tuyến qua chúng nên là giải pháp cuối cùng, không phải là một lựa chọn trung lập. Giới hạn trên giữ cho các hẻm đích có thể tiếp cận được.                                                                                                                                          | [src/sif/dynamiccost.cc#L84](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L84)                                          | High                                             |
| `use_highways`                                          | `0.5`            | `0.4 – 0.5` for metered consumer routes; split: `0.6 – 0.7` for cao-tốc-preferring product tier                      | Lower value = more aversion to motorway/trunk. Neutral on trunk (Highway-1) but avoid cao-tốc where the product does not want tolls implicit. A dedicated expressway-preferring tier can raise this; free tier keeps neutral.                                                                                   | Giá trị thấp hơn = tránh đường cao tốc/đường trục chính nhiều hơn. Trung lập với đường trục chính (Quốc lộ 1) nhưng tránh cao tốc ở nơi sản phẩm không muốn ngầm định phí cầu đường. Một cấp độ ưu tiên đường cao tốc chuyên dụng có thể tăng giá trị này; cấp độ miễn phí giữ ở mức trung lập.                     | [src/sif/autocost.cc#L33](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L33)                                                | Medium                                           |
| `use_tolls`                                             | `0.5`            | `0.2 – 0.3` (consumer free tier)                                                                                     | Most consumer requests would prefer toll-free; value 0.2 produces weighting `4 − 8 × 0.2 = 2.4` (see [src/sif/autocost.cc#L405](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L405)), discouraging but not excluding.                                                                  | Hầu hết các yêu cầu của người tiêu dùng thích miễn phí cầu đường; giá trị 0.2 tạo ra trọng số `4 − 8 × 0.2 = 2.4`, không khuyến khích nhưng không loại trừ.                                                                                                                                     | [src/sif/autocost.cc#L34,L404-L406](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L34)                                      | Medium                                           |
| `use_ferry`                                             | `0.5`            | `0.3 – 0.4`                                                                                                          | Mekong Delta small ferries should not be default; major river crossings must stay routable.                                                                                                                                                                                                                     | Các phà nhỏ ở Đồng bằng sông Cửu Long không nên là mặc định; các điểm qua sông lớn phải duy trì khả năng định tuyến.                                                                                                                                                                                            | [src/sif/dynamiccost.cc#L97](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L97)                                          | Medium                                           |
| `use_living_streets`                                    | `0.1`            | `0.2 – 0.3`                                                                                                          | Living-street class has legitimate through-traffic use in VN urban cores; raising softens the avoidance.                                                                                                                                                                                                        | Loại đường sinh hoạt có mục đích sử dụng giao thông đi qua hợp lệ ở các lõi đô thị VN; việc tăng giá trị làm giảm mức độ tránh né.                                                                                                                                                              | [src/sif/dynamiccost.cc#L100](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L100)                                        | Medium                                           |
| `maneuver_penalty`                                      | `5 s`            | `8 – 12 s`                                                                                                           | Every maneuver in HN / HCM carries more latency than EU arterials - signal wait plus mixed-traffic friction. Raising shifts routes toward fewer-turn geometries even at modest distance cost.                                                                                                                   | Mỗi thao tác ở Hà Nội / TP.HCM mang lại độ trễ cao hơn so với đường huyết mạch EU - chờ tín hiệu cộng với ma sát giao thông hỗn hợp. Việc tăng giá trị chuyển các tuyến đường sang hình học ít ngã rẽ hơn ngay cả với chi phí khoảng cách khiêm tốn.                                          | [src/sif/dynamiccost.cc#L83](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L83)                                          | Medium                                           |
| `gate_penalty` / `gate_cost`                            | `300 s` / `30 s` | Hold at defaults                                                                                                     | Private-access gate behaviour is consistent with VN; no override needed.                                                                                                                                                                                                                                        | Hành vi cổng truy cập riêng tư nhất quán với VN; không cần ghi đè.                                                                                                                                                                                                                              | [src/sif/dynamiccost.cc#L85-L86](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L85-L86)                                  | High                                             |
| `private_access_penalty`                                | `450 s`          | Hold at default                                                                                                      | Baseline is already aggressive.                                                                                                                                                                                                                                                                                 | Mức cơ sở đã đủ mạnh.                                                                                                                                                                                                                                                                           | [src/sif/dynamiccost.cc#L87](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L87)                                          | High                                             |
| `country_crossing_cost` / `_penalty`                    | `600 s` / `0 s`  | Not applicable                                                                                                       | VN routes do not cross admin country boundaries internally. Leave at defaults for Cambodia / Laos edge cases.                                                                                                                                                                                                   | Các tuyến đường VN không vượt qua ranh giới quốc gia hành chính trong nước. Giữ nguyên mặc định cho các trường hợp ngoại lệ ở Campuchia / Lào.                                                                                                                                                  | [src/sif/dynamiccost.cc#L92-L93](https://github.com/valhalla/valhalla/blob/master/src/sif/dynamiccost.cc#L92-L93)                                  | High                                             |
| Left-turn cost (`kTCUnfavorable = 2.5` for right-drive) | Hard-coded       | Not runtime-configurable - requires source fork to expose as option, or compensate indirectly via `maneuver_penalty` | The array is `constexpr` at [src/sif/autocost.cc#L62-L64](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L62-L64). Direct tuning means forking. Indirect compensation: raising `maneuver_penalty` adds a flat penalty to every turn (not just left), so it over-applies to right turns. | Mảng này là `constexpr`. Việc tinh chỉnh trực tiếp có nghĩa là phân nhánh. Bù đắp gián tiếp: tăng `maneuver_penalty` thêm một hình phạt cố định cho mỗi ngã rẽ (không chỉ rẽ trái), do đó nó áp dụng quá mức cho rẽ phải.                                                                       | [src/sif/autocost.cc#L588-L589](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L588) applies `kRightSideTurnCosts[turntype]` | Medium (direction) / Low (magnitude)             |
| `meili.auto.turn_penalty_factor`                        | `200`            | `80 – 140`                                                                                                           | Dense VN traces cross more real turns per km than EU; lower factor reduces false "straighten the trace" map-matches.                                                                                                                                                                                            | Dấu vết VN dày đặc đi qua nhiều ngã rẽ thực tế trên mỗi km hơn EU; hệ số thấp hơn làm giảm các khớp bản đồ sai lệch kiểu "làm thẳng dấu vết".                                                                                                                                                   | [scripts/valhalla_build_config#L293](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | Medium                                           |
| `loki.service_defaults.radius`                          | `0`              | `10 – 25 m` (urban request default)                                                                                  | GPS error floor in HN / HCM canyons exceeds 5 m; supplying a non-zero radius lets multiple candidates compete. Too-high (> 30 m) hurts latency and allows nonsense candidates.                                                                                                                                  | Mức lỗi GPS tối thiểu ở các hẻm núi Hà Nội / TP.HCM vượt quá 5m; việc cung cấp bán kính khác 0 cho phép nhiều ứng viên cạnh tranh. Quá cao (> 30m) làm tăng độ trễ và cho phép các ứng viên vô lý.                                                                                            | [scripts/valhalla_build_config#L199](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | High (direction) / Medium (magnitude)            |
| `loki.service_defaults.search_cutoff`                   | `35000 m`        | `3000 – 5000 m`                                                                                                      | In VN any legal snap is within hundreds of metres; capping lower saves p99 on degraded GPS.                                                                                                                                                                                                                     | Ở VN, bất kỳ điểm bắt hợp lệ nào cũng nằm trong phạm vi hàng trăm mét; giới hạn thấp hơn giúp tiết kiệm p99 trên GPS bị suy giảm.                                                                                                                                                               | [scripts/valhalla_build_config#L201](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | High (direction) / Medium (magnitude)            |
| `loki.service_defaults.node_snap_tolerance`             | `5 m`            | `2 – 3 m`                                                                                                            | Prevents spurious node-snap flips in dense urban grids.                                                                                                                                                                                                                                                         | Ngăn chặn các lỗi bắt vào nút giả trong lưới đô thị đông đúc.                                                                                                                                                                                                                                   | [scripts/valhalla_build_config#L202](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | Medium                                           |
| `loki.service_defaults.minimum_reachability`            | `50`             | `15 – 25`                                                                                                            | Allows small hẻm clusters to be valid destinations. Risk: more stranded-island candidates reach the router; mitigated by the opposing-edge swap at [src/loki/search.cc#L643-L653](https://github.com/valhalla/valhalla/blob/master/src/loki/search.cc#L643-L653).                                             | Cho phép các cụm hẻm nhỏ trở thành điểm đến hợp lệ. Rủi ro: nhiều ứng viên đảo bị mắc kẹt tiếp cận bộ định tuyến hơn; được giảm thiểu bằng việc hoán đổi cạnh đối diện.                                                                                                                         | [scripts/valhalla_build_config#L200](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | Medium                                           |
| `thor.bidirectional_astar.expand_within_distance[1]`    | `20000 m`        | `30000 – 40000 m` for intra-city VN                                                                                  | The 20 km L1 horizon is tight for HCM / HN cross-city routes. Raising lets the search keep arterial detail through the core. Trade-off: more labels, slightly slower Bidir.                                                                                                                                     | Đường chân trời L1 20 km là chật hẹp đối với các tuyến đường xuyên thành phố TP.HCM / Hà Nội. Việc tăng lên cho phép tìm kiếm giữ lại chi tiết đường huyết mạch qua lõi đô thị. Đánh đổi: nhiều nhãn hơn, Bidir chậm hơn một chút.                                                                | [scripts/valhalla_build_config#L245](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | Low (mechanism Medium, magnitude Low)            |
| `mjolnir.default_speeds_config`                         | unset            | Author a VN-urban profile (country × urban × road-class) - separate deliverable                                      | Biggest lever for VN ETA accuracy. Requires Vietnam-specific measured speeds by road class & urban density.                                                                                                                                                                                                     | Đòn bẩy lớn nhất cho độ chính xác ETA tại VN. Yêu cầu tốc độ đo lường cụ thể của Việt Nam theo loại đường & mật độ đô thị.                                                                                                                                                                      | [scripts/valhalla_build_config#L162](https://github.com/valhalla/valhalla/blob/master/scripts/valhalla_build_config)                               | High (direction) / Low (values - data-dependent) |


**Override granularity.** `service_defaults.`* and `service_limits.`* are server-wide. Costing knobs (`maneuver_penalty`, `use_highways`, etc.) can be supplied per-request by the client - so operator-tier profiles are feasible without server config changes, provided the client encodes the knobs.

### 11.3 Data we still need before tuning is defensible

None of the ranges above can be collapsed to a point value without the following signals. Each is listed with its purpose, the source system that would produce it, and the tuning decision it unblocks.


| Signal                                                                                                                         | Purpose                                                                                    | Source                                                                                                                                               | Unblocks                                                                 |
| ------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| GPS-error distribution by district (HCM 1/3/5/7/Bình Thạnh, HN Hoàn Kiếm/Cầu Giấy/Ba Đình/…)                                   | Pick `radius` and `node_snap_tolerance` point values                                       | rider/driver traces, grouped by urban density class                                                                                                  | `loki.default_radius`, `node_snap_tolerance`                             |
| Intersection-delay histogram by turn type (straight / right / left / U-turn) at top-100 HCM/HN intersections, peak vs off-peak | Validate or invalidate the 2.5× left-turn cost and inform `maneuver_penalty`               | Trace inference: time between last traversal on approach edge and first traversal on departure edge, filtered to through-traffic                     | `maneuver_penalty` magnitude; justification for forking turn-cost arrays |
| Alley vs through-road speed delta (hẻm / ngõ average speed vs adjacent residential mean)                                       | Validate `alley_penalty` magnitude                                                         | Trace speed samples on edges tagged `highway=service` with alley attribution                                                                         | `alley_penalty` point value                                              |
| Ferry-edge actual usage (which VN ferry edges see real driver traffic vs which are OSM-tagged but rarely used)                 | Separate "major river crossings" from "Mekong small craft"                                 | Trace snaps on edges tagged `route=ferry`                                                                                                            | `use_ferry` point value and potential per-edge exclusion list            |
| Live-traffic coverage map by road class within HCM / HN                                                                        | Decides whether `kDefaultFlowMask` including live is delivering benefit or noise per class | Aggregated over `traffic.tar` - share of L0/L1/L2 edges with a non-zero `live_speed` sample in a 7-day window                                        | Whether to gate live traffic by class                                    |
| Map-match turn-density histogram on representative HCM / HN traces                                                             | Pick `meili.auto.turn_penalty_factor`                                                      | Replay corpus of real traces through the map-matcher at varying factors; measure match quality (e.g. edit distance to manually-matched ground truth) | `turn_penalty_factor` point value                                        |
| Cross-city route length distribution                                                                                           | Decides whether raising `expand_within_distance[1]` from 20 km is load-bearing             | Daily request log - beeline distance distribution for intra-HCM and intra-HN requests                                                                | `thor.bidirectional_astar.expand_within_distance[1]` override decision   |
| Hẻm-destination success rate (% of address-level pickups that snap to the intended hẻm vs the nearest through-street)          | Validates `minimum_reachability` lowering                                                  | A/B: same request corpus, reachability 50 vs 25, compare snap addresses to input addresses                                                           | `minimum_reachability` point value                                       |
| Node density per km² by VN district                                                                                            | Calibrates the urban vs rural speed profile in `default_speeds_config`                     | One-time derivation from the local tile extract                                                                                                      | `default_speeds_config` VN urban / rural breakpoints                     |


**Minimum viable dataset to begin tuning:** the first three rows above (GPS error, intersection delay, alley speed) are sufficient to propose point values for `loki.default_radius`, `maneuver_penalty`, and `alley_penalty`. Everything downstream can be scheduled after those three land.

---

## Appendix A - Citation index

All links point to `master` on `github.com/valhalla/valhalla`. For long-term stability, pin to a commit SHA.

### Data model

- [src/baldr/tilehierarchy.cc#L14-L30](https://github.com/valhalla/valhalla/blob/master/src/baldr/tilehierarchy.cc#L14-L30) - three tile levels, sizes 4°/1°/0.25°
- [valhalla/baldr/graphid.h](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphid.h) - 64-bit GraphId definition
- [valhalla/baldr/graphreader.h#L713](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphreader.h#L713) - `RecoverShortcut`

### Hierarchy and limits

- [valhalla/sif/hierarchylimits.h#L44-L47](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h#L44-L47) - `StopExpanding` formula
- [valhalla/sif/hierarchylimits.h#L21-L29](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h#L21-L29) - defaults `{0, 400, 100}` and `expand_within_dist`
- [valhalla/sif/hierarchylimits.h](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/hierarchylimits.h) - `RelaxHierarchyLimits` (retry path)

### A heuristic and costing

- [valhalla/thor/astarheuristic.h#L60-L64](https://github.com/valhalla/valhalla/blob/master/valhalla/thor/astarheuristic.h#L60-L64) - `Get(ll) = dist × costfactor`, "MUST UNDERESTIMATE"
- [valhalla/sif/dynamiccost.h](https://github.com/valhalla/valhalla/blob/master/valhalla/sif/dynamiccost.h) - `AStarCostFactor()` contract
- [src/sif/autocost.cc#L298-L300](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc#L298-L300) - `AStarCostFactor` for auto
- [src/sif/autocost.cc#L938-L1008](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc) - `EdgeCost` formula
- [src/sif/autocost.cc#L1010-L1099](https://github.com/valhalla/valhalla/blob/master/src/sif/autocost.cc) - `TransitionCost` formula

### Algorithms

- [src/thor/unidirectional_astar.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/unidirectional_astar.cc)
- [src/thor/bidirectional_astar.cc#L146](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L146) - initial `cost_threshold = +∞`
- [src/thor/bidirectional_astar.cc#L768-L778](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L768-L778) - forward termination (`route_lower_bound > cost_threshold_`)
- [src/thor/bidirectional_astar.cc#L623-L624](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L623-L624) - meeting detection (`opp_edgeid` already `kPermanent` in reverse)
- [src/thor/bidirectional_astar.cc#L894-L901](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L894-L901) - `threshold_delta` continue-past-first-meet (`SetForwardConnection`)
- [src/thor/bidirectional_astar.cc#L546-L561](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L546-L561) - forward/reverse `TimeInfo` construction (context for #5616)
- [src/thor/costmatrix.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/costmatrix.cc) - matrix
- [src/thor/isochrone.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/isochrone.cc) - isochrone
- [src/thor/multimodal.cc](https://github.com/valhalla/valhalla/blob/master/src/thor/multimodal.cc) - multimodal

### Traffic

- [valhalla/baldr/traffictile.h#L45-L57](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L45-L57) - `TrafficSpeed` bit-packed struct
- [valhalla/baldr/traffictile.h#L133-L140](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L133-L140) - `TrafficTileHeader` layout
- [valhalla/baldr/traffictile.h#L146-L150](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L146-L150) - `static_assert`s (sizes enforced)
- [valhalla/baldr/traffictile.h#L221-L222](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/traffictile.h#L221-L222) - `volatile` pointers (live updates)
- [valhalla/baldr/graphtile.h#L811-L816](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L811-L816) - `LIVE_SPEED_FADE = 1/3600` plus multiplier
- [valhalla/baldr/graphtile.h#L860-L863](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L860-L863) - final blend formula
- [valhalla/baldr/graphtile.h#L788-L793](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L788-L793) - docstring confirming bidirectional sets `seconds_from_now = 0`
- [valhalla/baldr/graphtile.h#L857-L876](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphtile.h#L857-L876) - predicted → constrained → freeflow priority
- [proto/options.proto#L415-L421](https://github.com/valhalla/valhalla/blob/master/proto/options.proto#L415-L421) - `DateTimeType` enum

### External references

- [Issue #5616 - bidirectional live-traffic fade bug](https://github.com/valhalla/valhalla/issues/5616)
- [PR #2268 - original live-traffic support](https://github.com/valhalla/valhalla/pull/2268)
- [PR #3398 - ETA calculation correction](https://github.com/valhalla/valhalla/pull/3398)
- [CHANGELOG](https://github.com/valhalla/valhalla/blob/master/CHANGELOG.md)

---

## Appendix B - Diagram-to-code cross-reference

- **Image #4** (bidirectional tree) maps onto §4.2. The `mu` annotations in the diagram equal `cost_threshold_` in code. The "C found in both directions" box equals the `edgestatus_reverse_.Get(fwd_pred.opp_edgeid()) == kPermanent` branch at [bidirectional_astar.cc#L623](https://github.com/valhalla/valhalla/blob/master/src/thor/bidirectional_astar.cc#L623).
- **Image #5** (unidirectional tree) maps onto §4.1. The "Came From" map equals the `EdgeLabel::predecessor()` chain reconstructed by `FormPath()`.

---

## Appendix C - Raw debug trace snippets

From a debug trace (9.84 km Hanoi route, 2026-04-14):

```
step 1: INIT BidirectionalAStar
  origin: (21.081849, 105.771836) correlated to 2 candidate edges
  dest:   (20.999081, 105.738643) correlated to 1 candidate edges
  straight-line: 9.838 km
  AStarCostFactor: 0.025714 = kSpeedFactor[top_speed] * min_linear_cost_factor
  threshold_delta: 420.0 [config: thor.bidirectional_astar.threshold_delta]
  bucket_size: 1
  max_reserved_labels: 1000000

  hierarchy L0: max_up_transitions=0   expand_within_dist=100000 km
  hierarchy L1: max_up_transitions=400 expand_within_dist=20 km    ← bidir variant
  hierarchy L2: max_up_transitions=100 expand_within_dist=5 km

step 4: MAIN LOOP - alternating fwd/rev expansion
  [rev] TILE LOAD #1 tile=2/639062/0 L2 nodes=39201  edges=87306
  [fwd] TILE LOAD #2 tile=2/640503/0 L2 nodes=137361 edges=321546
  [fwd] TILE LOAD #3 tile=1/40245/0  L1 nodes=73718  edges=164357
  [rev] TILE LOAD #4 tile=1/39885/0  L1 nodes=26942  edges=60868
  [fwd] TILE LOAD #5 tile=0/2501/0   L0 nodes=108728 edges=241144
  [rev] TILE LOAD #6 tile=2/640502/0 L2 nodes=55732  edges=127077
  [rev] TILE LOAD #7 tile=2/639063/0 L2 nodes=91027  edges=209440

  [fwd] HIERARCHY PRUNE L2: up_transitions=194 > max=100
                            AND dist=5008 m > expand_within=5000 m

step 5: FIRST CONNECTION - forward met reverse
  meeting edge = 0/2501/73729       ← L0 highway tile
  connection_cost = 1786.20
  cost_threshold  = 1786.20 + 420.0 = 2206.20

step 6: TERMINATE - rev sortcost 2214.04 > threshold 2206.20
step 8: DONE
  best path: 254 edges, 16.629 km
  total time: 1119.6 s (18.66 min)
  search efficiency: 31,812 labels : 254 path edges = 125:1
  tiles loaded: 7 (L0=1, L1=2, L2=4)
```

And the HCMC → Hanoi long-route summary:

```
best path: 3301 edges, 1,653.51 km
total time: 65,708 s (1,095 min = 18.25 hr)
search efficiency: 197,027 labels : 3301 path edges = 60:1   ← BETTER than short
tiles loaded: 18 (L0=9, L1=5, L2=4)
```

---

*Corrections and additions should cite the specific source-file line on `github.com/valhalla/valhalla` that supports the change.*