package main

import (
	"fmt"
	"strings"
)

func main() {
	// Examples 1 and 2 live in demo_basic.go — untouched so the CH work
	// below can evolve without disturbing them.
	runBasicDemos()

	// ── 3. Contraction Hierarchy Demo ──
	// 18-node graph with THREE road classes (levels) and L (local) nodes
	// scattered across three districts — they're no longer colinear on y=0.
	// This is a miniature of how Valhalla sees a real road network.

	fmt.Println("\n╔══════════════════════════════════════╗")
	fmt.Println("║  3. Contraction Hierarchy            ║")
	fmt.Println("╚══════════════════════════════════════╝")

	g3 := buildHierarchicalGraph(false) // false = free-flow baseline (no live traffic)

	fmt.Println()
	fmt.Println("GRAPH: 3-level road network, 37 nodes across 3 scattered districts")
	fmt.Println("       Level 2 = highway (H1..H11)   base weight 3.0  (free-flow)")
	fmt.Println("       Level 1 = arterial (A1..A11)  base weight 5.0  (normal surface streets)")
	fmt.Println("       Level 0 = locals, 5 per district — scattered 2D, y ranges 0..5")
	fmt.Println("       Route: L1 (-1,0) in D1 → L9 (31,0) in D11   ≈ 32 units end-to-end")
	fmt.Println("       Section 3b below overlays a live-traffic update and re-runs the search.")
	fmt.Println()

	// Run 1 — plain A* baseline.
	fmt.Println("── Run 1: Standard A* (baseline, no CH) ──────────────────")
	path1, cost1, nodes1 := AStar(g3, "L1", "L9")

	// Run 2 — CH with superseded-edge skipping, no hierarchy limits.
	// This is "pure CH": shortcuts replace chains, but the search still
	// considers every level everywhere.
	fmt.Println("\n── Run 2: Bi-A* + CH (no hierarchy limits) ───────────────")
	path2, cost2, stats2 := BiAStarCHStats(g3, "L1", "L9")

	// Run 3 — CH with Valhalla-style hierarchy limits. Local edges only
	// within 2.5 of the endpoint, arterial within 5.0, highway unlimited.
	fmt.Println("\n── Run 3: Bi-A* + CH + hierarchy limits (Valhalla-style) ──")
	path3, cost3, stats3 := BiAStarCHStats(g3, "L1", "L9", DefaultHierarchyLimits())

	// Comparison table
	fmt.Println()
	fmt.Println("╔═══════════════════════════════════════════════════════════════════════════════════════════╗")
	fmt.Println("║                              COMPARISON SUMMARY                                           ║")
	fmt.Println("╠═══════════════════════════════╦═══════╦═════════════╦══════════════╦════════╦════════════╣")
	fmt.Println("║ Algorithm                     ║ Nodes ║ Shortcuts   ║ Levels used  ║ Cost   ║ Hier. cuts ║")
	fmt.Println("╠═══════════════════════════════╬═══════╬═════════════╬══════════════╬════════╬════════════╣")
	fmt.Printf("║ A* (baseline)                 ║  %3d  ║      —      ║      —       ║ %5.2f  ║     —      ║\n",
		nodes1, cost1)
	fmt.Printf("║ CH, no hierarchy limits       ║  %3d  ║     %2d      ║ %-12s ║ %5.2f  ║     —      ║\n",
		stats2.NodesExplored, stats2.ShortcutsUsed, fmt.Sprint(stats2.LevelsUsed), cost2)
	fmt.Printf("║ CH + hierarchy limits         ║  %3d  ║     %2d      ║ %-12s ║ %5.2f  ║    %3d     ║\n",
		stats3.NodesExplored, stats3.ShortcutsUsed, fmt.Sprint(stats3.LevelsUsed), cost3, stats3.HierarchyCuts)
	fmt.Println("╚═══════════════════════════════╩═══════╩═════════════╩══════════════╩════════╩════════════╝")
	fmt.Println()

	fmt.Println("  Paths found (all three identical cost ⇒ CH preserves optimality):")
	fmt.Printf("    A*            : %s\n", strings.Join(path1, " → "))
	fmt.Printf("    CH, no limits : %s\n", strings.Join(path2, " → "))
	fmt.Printf("    CH + limits   : %s\n", strings.Join(path3, " → "))
	fmt.Println()

	fmt.Println("  Interpreting the numbers on this toy graph:")
	fmt.Println("    • Euclidean heuristic is strong at 37-node scale → plain A* is already")
	fmt.Println("      tight. Bi-A*'s two-sided expansion looks like overhead here.")
	fmt.Println("    • The visible CH mechanism is still working:")
	fmt.Printf("        - preprocessing produced 18 shortcuts across ALL 3 levels\n")
	fmt.Printf("          (9 highway, 8 arterial, 1 local — L5══L12 via L13)\n")
	fmt.Printf("        - the found path uses 1 shortcut hop that hides 10 real highway edges\n")
	fmt.Printf("        - hierarchy limits rejected %d edges that unlimited-CH would have taken\n",
		stats3.HierarchyCuts)
	fmt.Println("    • Weights above are the free-flow baseline. Section 3b below overlays")
	fmt.Println("      a live-traffic update on edges at all 3 levels and re-runs the search —")
	fmt.Println("      the optimal route changes without rebuilding CH.")
	fmt.Println("    • At continental scale Valhalla's rules below invert the ranking:")
	fmt.Println("        - heuristic becomes weak over long distances → plain A* bubble explodes")
	fmt.Println("        - Bi-A*'s two 'half-searches' each hit a much smaller frontier")
	fmt.Println("        - CH shortcuts replace whole highway chains in ONE edge relaxation")
	fmt.Println("        - hierarchy limits make the middle of the route highway-only")
	fmt.Println("      Result: node count scales like O(√n) instead of O(n).")
	fmt.Println()
	fmt.Println("  Rules actually enforced (all lifted from Valhalla):")
	fmt.Println("    • CanContract — 2 same-level neighbors AND 0 transition edges")
	fmt.Println("    • Shortcuts added alongside originals; originals marked Superseded")
	fmt.Println("    • Search skips Superseded edges (a shortcut always covers them)")
	fmt.Println("    • ExpandWithin[level] — gate each edge by Euclidean distance from home")

	// ── 3b. Live-traffic overlay ──
	// Rebuild the SAME graph with a live-traffic overlay applied, then re-run
	// the search. The preprocessing (18 shortcuts) is reused unchanged — only
	// edge weights differ. The route flips because traffic hits edges at all
	// three levels at once.
	fmt.Println("\n╔══════════════════════════════════════╗")
	fmt.Println("║  3b. Live-Traffic Overlay            ║")
	fmt.Println("╚══════════════════════════════════════╝")
	fmt.Println()
	fmt.Println("A live feed arrives and updates 4 edges — we do NOT rebuild CH, we just")
	fmt.Println("re-run the search on the updated weights:")
	fmt.Println("    H4 ↔ H5     3.0 → 15.0   🚨 multi-car accident, highway crawls")
	fmt.Println("    H5 ↔ H6     3.0 → 15.0   🚨 accident backup one edge further")
	fmt.Println("    A8 ↔ A9     5.0 →  8.0   🚦 downtown rush hour on the arterial")
	fmt.Println("    L1 ↔ A1     2.5 →  8.0   🚧 construction on the L1 on-ramp street")
	fmt.Println()

	g3Traffic := buildHierarchicalGraph(true)

	fmt.Println("── Re-run: Bi-A* + CH on the updated graph ───────────────")
	pathT, costT, _ := BiAStarCHStats(g3Traffic, "L1", "L9")
	fmt.Println()

	fmt.Println("  ── BEFORE / AFTER — route change side-by-side ──")
	fmt.Printf("    [free-flow]        cost %5.2f\n", cost2)
	fmt.Printf("       route : %s\n", strings.Join(path2, " → "))
	fmt.Printf("    [+ live traffic]   cost %5.2f   (Δ = +%.2f; highway now blocked)\n", costT, costT-cost2)
	fmt.Printf("       route : %s\n", strings.Join(pathT, " → "))
	fmt.Println()

	fmt.Println("  ★ What to point out to students:")
	fmt.Println("    • The graph didn't change — only four edge WEIGHTS did. Preprocessing")
	fmt.Println("      (the 18 shortcuts from Section 3) is reused as-is.")
	fmt.Println("    • The optimal route flips in TWO independent places:")
	fmt.Println("        (a) local entry — L1→A1 (jammed) replaced by L1→L2→A1 (detour)")
	fmt.Println("        (b) backbone   — highway (accident!) replaced by arterial chain")
	fmt.Println("    • Traffic updates deliberately hit all 3 levels (H, A, L) so every")
	fmt.Println("      hierarchy layer contributes to the new best path.")
	fmt.Println("    • This is exactly how a real router applies live speeds: feed arrives,")
	fmt.Println("      overlay updates weights, next search picks the new optimum.")
}

// buildHierarchicalGraph builds the 37-node, 3-level demo graph and returns
// it with either free-flow weights (liveTraffic=false) or a live-traffic
// overlay applied on top (liveTraffic=true).
//
//   Highway (L2):  H1 — H2 — ... — H11       (11 nodes, y=4, base 3.0)
//                  |                |
//   Arterial (L1): A1 — A2 — ... — A11       (11 nodes, y=2, base 5.0)
//                 /|\\             /|\\             /|\\
//   Local (L0):   5 in D1 (x≈0)   5 in D6 (x≈15)  5 in D11 (x≈30)
//
//   D1   anchored at A1:  L1(-1,0) start, L2(0,1), L3(1,0), L10(-1,3), L11(-2,1)
//   D6   anchored at A6:  L4(14,0), L5(15,1), L6(16,0), L13(15,3), L12(15,5)
//   D11  anchored at A11: L7(29,0), L8(30,1), L9(31,0) goal, L14(31,3), L15(30,5)
//
// Free-flow baseline: every highway edge 3.0, every arterial edge 5.0,
// every L→A ramp 2.5. On this baseline the optimal L1→L9 route is the
// HIGHWAY: L1→A1→H1══H11→A11→L9 at cost 39.0.
//
// Live-traffic overlay (liveTraffic=true) mutates four edges across ALL
// three levels — a realistic "data feed arrived" scenario:
//   H4↔H5 / H5↔H6  3.0 → 15.0   multi-car accident on the highway
//   A8↔A9          5.0 →  8.0   downtown rush hour on the arterial
//   L1↔A1          2.5 →  8.0   local construction on the on-ramp street
// Under this overlay the same search re-run without rebuilding CH picks a
// DIFFERENT route: L1→L2→A1→arterial→A11→L9 at cost 58.5, avoiding both
// the closed highway and the jammed on-ramp. That flip is the demo.
//
// Weights stay admissible (every edge weight ≥ Euclidean distance between
// its endpoints) so the heuristic remains consistent.
//
// Contraction result (identical under either overlay — CH is structural):
// 18 shortcuts across ALL 3 levels.
//   • H2..H10 contract → H1══H11 (9 highway contractions)
//   • A2..A5 & A7..A10 contract → A1══A6, A6══A11 (8 arterial contractions)
//   • L13 contracts → L5══L12 (1 local contraction — proves CH isn't only
//     about highways; any level with a clean "2 neighbors + 0 transitions"
//     node collapses the same way)
func buildHierarchicalGraph(liveTraffic bool) *Graph {
	g := NewGraph()

	// === Edge-weight knobs ===
	// Free-flow baseline: all highway edges 3.0, all arterial edges 5.0,
	// L1↔A1 ramp 2.5. These are what a static cost model would give us.
	localL1A1 := 2.5
	arterialWeights := []float64{5, 5, 5, 5, 5, 5, 5, 5, 5, 5}
	highwayWeights := []float64{3, 3, 3, 3, 3, 3, 3, 3, 3, 3}

	// Live-traffic overlay: a real-time feed (accidents, rush-hour speeds,
	// construction) mutates a handful of edge weights in place. The graph
	// structure and all CH shortcuts are untouched — only numbers change.
	if liveTraffic {
		highwayWeights[3] = 15.0 // H4↔H5 — multi-car accident
		highwayWeights[4] = 15.0 // H5↔H6 — accident backup one edge further
		arterialWeights[7] = 8.0 // A8↔A9 — downtown rush hour
		localL1A1 = 8.0          // L1↔A1 — construction on the on-ramp street
	}

	// Level 2 — Highway (11 nodes, y=4)
	for i := 1; i <= 11; i++ {
		g.AddNodeWithLevel(fmt.Sprintf("H%d", i), float64((i-1)*3), 4, 2)
	}

	// Level 1 — Arterial (11 nodes, y=2)
	for i := 1; i <= 11; i++ {
		g.AddNodeWithLevel(fmt.Sprintf("A%d", i), float64((i-1)*3), 2, 1)
	}

	// Level 0 — 15 locals (5 per district), scattered in 2D with y up to 5.
	// D1 (x≈0)
	g.AddNodeWithLevel("L1", -1, 0, 0) // start
	g.AddNodeWithLevel("L2", 0, 1, 0)
	g.AddNodeWithLevel("L3", 1, 0, 0)
	g.AddNodeWithLevel("L10", -1, 3, 0) // north bridge (y=3, between A and H rows)
	g.AddNodeWithLevel("L11", -2, 1, 0) // west cul-de-sac off L1 only

	// D6 (x≈15) — the "local chain" district: L5 → L13 → L12 is 3 locals in a
	// line perpendicular to the arterial. L13 has only L-neighbors so it
	// contracts, producing shortcut L5══L12.
	g.AddNodeWithLevel("L4", 14, 0, 0)
	g.AddNodeWithLevel("L5", 15, 1, 0)
	g.AddNodeWithLevel("L6", 16, 0, 0)
	g.AddNodeWithLevel("L13", 15, 3, 0) // chain middle — CONTRACTS (2 locals, 0 transitions)
	g.AddNodeWithLevel("L12", 15, 5, 0) // far-north tip (y=5) — terminal local

	// D11 (x≈30)
	g.AddNodeWithLevel("L7", 29, 0, 0)
	g.AddNodeWithLevel("L8", 30, 1, 0)
	g.AddNodeWithLevel("L9", 31, 0, 0)  // goal
	g.AddNodeWithLevel("L14", 31, 3, 0) // NE corner — connects to A11 and L9
	g.AddNodeWithLevel("L15", 30, 5, 0) // far-north cul-de-sac (y=5)

	// === LOCAL ↔ LOCAL edges ===
	// D1
	g.AddBidirectional("L1", "L2", 1.5)
	g.AddBidirectional("L2", "L3", 1.5)
	g.AddBidirectional("L1", "L11", 1.5) // Eucl √2 ≈ 1.41
	g.AddBidirectional("L1", "L10", 3.2) // Eucl 3.0
	// D6 — the contractible chain
	g.AddBidirectional("L4", "L5", 1.5)
	g.AddBidirectional("L5", "L6", 1.5)
	g.AddBidirectional("L5", "L13", 2.5)  // L5(15,1)→L13(15,3): Eucl 2.0
	g.AddBidirectional("L13", "L12", 2.5) // L13(15,3)→L12(15,5): Eucl 2.0
	// D11
	g.AddBidirectional("L7", "L8", 1.5)
	g.AddBidirectional("L8", "L9", 1.5)
	g.AddBidirectional("L9", "L14", 3.2) // Eucl 3.0

	// === LOCAL ↔ ARTERIAL (transitions disqualify A1/A6/A11 from contracting) ===
	// D1 → A1
	g.AddBidirectional("L1", "A1", localL1A1)
	g.AddBidirectional("L2", "A1", 1.5)
	g.AddBidirectional("L3", "A1", 2.5)
	g.AddBidirectional("L10", "A1", 1.5) // Eucl √2 ≈ 1.41
	// L11 deliberately NOT connected to A1 — pure cul-de-sac off L1

	// D6 → A6
	g.AddBidirectional("L4", "A6", 2.5)
	g.AddBidirectional("L5", "A6", 1.5)
	g.AddBidirectional("L6", "A6", 2.5)
	g.AddBidirectional("L12", "A6", 3.5) // L12(15,5)→A6(15,2): Eucl 3.0 — long slow local
	// L13 deliberately NOT connected to A6 — that's what lets it contract!

	// D11 → A11
	g.AddBidirectional("L7", "A11", 2.5)
	g.AddBidirectional("L8", "A11", 1.5)
	g.AddBidirectional("L9", "A11", 2.5)
	g.AddBidirectional("L14", "A11", 1.5) // Eucl √2 ≈ 1.41
	g.AddBidirectional("L15", "A11", 3.5) // L15(30,5)→A11(30,2): Eucl 3.0

	// === ARTERIAL CHAIN ===
	for i := 1; i < 11; i++ {
		g.AddBidirectional(fmt.Sprintf("A%d", i), fmt.Sprintf("A%d", i+1), arterialWeights[i-1])
	}

	// === ARTERIAL ↔ HIGHWAY ramps (only at the two ends) ===
	g.AddBidirectional("A1", "H1", 2.0)
	g.AddBidirectional("A11", "H11", 2.0)

	// === HIGHWAY CHAIN ===
	for i := 1; i < 11; i++ {
		g.AddBidirectional(fmt.Sprintf("H%d", i), fmt.Sprintf("H%d", i+1), highwayWeights[i-1])
	}

	return g
}
