package main

import (
	"container/heap"
	"fmt"
	"math"
	"sort"
	"strings"
)

// ---------------------------------------------------------------------------
// Valhalla-style Contraction Hierarchy
// ---------------------------------------------------------------------------
//
// Key differences from a naïve CH:
//   • Levels are pre-assigned by road class (0=local, 1=arterial, 2=highway).
//   • Nodes are NEVER deleted — shortcuts are added alongside originals.
//   • Contraction is per-level: a node contracts only if its 2 "same-level"
//     neighbors can be joined by a shortcut on that level.
//   • Original edges are marked Superseded so the search skips them when it
//     can use the shortcut instead.
//   • At query time we allow superseded edges near start/goal (first-/last-mile).
//
// See Valhalla: src/mjolnir/shortcutbuilder.cc  (CanContract, AddShortcutEdges).

// ---------------------------------------------------------------------------
// Phase 1 — build shortcuts (mirrors Valhalla's ShortcutBuilder)
// ---------------------------------------------------------------------------

type contractStep struct {
	level        int
	contracted   string
	left, right  string
	wLeft, wRight float64
	leftSC, rightSC bool // were the incoming edges already shortcuts?
	shortcut     string // e.g. "L4==A2"
	scWeight     float64
	via          []string
	nodesActive  int // number of non-contracted nodes remaining
}

// BuildShortcuts adds shortcut edges to g and marks superseded originals.
// Returns the ordered list of contraction steps (for printing).
// Nodes listed in protected are never contracted.
func BuildShortcuts(g *Graph, protected ...string) []contractStep {
	prot := make(map[string]bool)
	for _, id := range protected {
		prot[id] = true
	}

	contracted := make(map[string]bool)
	var steps []contractStep

	// Process levels top-down — highway (2) first, then arterial (1), then local (0).
	// This matches Valhalla (shortcuts at higher levels are created first).
	levels := collectLevels(g)
	sort.Sort(sort.Reverse(sort.IntSlice(levels)))

	for _, level := range levels {
		// Fixed-point pass over this level.
		for {
			progressed := false
			// Sort nodes for deterministic output.
			ids := make([]string, 0, len(g.Nodes))
			for id, n := range g.Nodes {
				if n.Level == level && !contracted[id] && !prot[id] {
					ids = append(ids, id)
				}
			}
			sort.Strings(ids)

			for _, id := range ids {
				if contracted[id] {
					continue
				}
				step, ok := tryContract(g, id, level, contracted)
				if !ok {
					continue
				}
				step.nodesActive = len(g.Nodes) - len(contracted)
				steps = append(steps, step)
				contracted[id] = true
				progressed = true
			}
			if !progressed {
				break
			}
		}
	}

	return steps
}

// tryContract inspects a candidate node and, if it qualifies, adds the
// shortcut edges and marks the original edges as superseded.
func tryContract(g *Graph, id string, level int, contracted map[string]bool) (contractStep, bool) {
	// Active same-level neighbors = neighbors reached by an edge at this level
	// that has not already been superseded by a shortcut on this pass.
	n1, n2, e1, e2 := sameLevelPair(g, id, level, contracted)
	if n1 == "" {
		return contractStep{}, false
	}

	shortcutW := e1.Weight + e2.Weight

	// Via list accumulates real nodes hidden inside the shortcut, same as
	// Valhalla walks ConnectEdges across contracted chains.
	via := append([]string{}, e1.Via...)
	via = append(via, id)
	via = append(via, e2.Via...)
	revVia := make([]string, len(via))
	for i, v := range via {
		revVia[len(via)-1-i] = v
	}

	// Mark the two incoming originals as superseded (both directions).
	markSuperseded(g, n1, id)
	markSuperseded(g, id, n1)
	markSuperseded(g, id, n2)
	markSuperseded(g, n2, id)

	// Supersedes keys — useful for debugging.
	supersedes := []string{n1 + "-" + id, id + "-" + n2}
	supersedesRev := []string{n2 + "-" + id, id + "-" + n1}

	// Add the shortcut edge in both directions.
	g.Edges[n1] = append(g.Edges[n1], Edge{
		From: n1, To: n2, Weight: shortcutW, Level: level,
		IsShortcut: true, Supersedes: supersedes, Via: via,
	})
	g.Edges[n2] = append(g.Edges[n2], Edge{
		From: n2, To: n1, Weight: shortcutW, Level: level,
		IsShortcut: true, Supersedes: supersedesRev, Via: revVia,
	})

	return contractStep{
		level:       level,
		contracted:  id,
		left:        n1, right: n2,
		wLeft: e1.Weight, wRight: e2.Weight,
		leftSC:  e1.IsShortcut,
		rightSC: e2.IsShortcut,
		shortcut: n1 + "==" + n2, scWeight: shortcutW,
		via: via,
	}, true
}

// sameLevelPair returns the two active same-level neighbors of id (or "" if
// it has more/fewer than 2). An edge counts as same-level only when BOTH
// endpoints sit on `level` — edges that leave the level are "transition
// edges" in Valhalla terms and do not participate in this level's shortcuts.
// Superseded edges and edges to already-contracted nodes are skipped, because
// the CH search would skip them too.
//
// We also refuse to contract a node that has ANY transition edge still in
// play (same rule as Valhalla's CanContract: "Do not create a shortcut
// across a node that has any upward transitions").
func sameLevelPair(g *Graph, id string, level int, contracted map[string]bool) (string, string, *Edge, *Edge) {
	type nb struct {
		neighbor string
		edge     *Edge
	}
	var active []nb
	seen := make(map[string]bool)

	for i := range g.Edges[id] {
		e := &g.Edges[id][i]
		if contracted[e.To] {
			continue
		}
		// Transition edge (leaves this level) → disqualifies this node.
		if g.Nodes[e.To].Level != level {
			return "", "", nil, nil
		}
		if e.Superseded {
			continue
		}
		if seen[e.To] {
			continue
		}
		seen[e.To] = true
		active = append(active, nb{e.To, e})
	}
	if len(active) != 2 {
		return "", "", nil, nil
	}
	return active[0].neighbor, active[1].neighbor, active[0].edge, active[1].edge
}

func markSuperseded(g *Graph, from, to string) {
	for i := range g.Edges[from] {
		e := &g.Edges[from][i]
		if e.To == to && !e.IsShortcut && !e.Superseded {
			e.Superseded = true
			return
		}
	}
}

func collectLevels(g *Graph) []int {
	seen := make(map[int]bool)
	for _, n := range g.Nodes {
		seen[n.Level] = true
	}
	out := make([]int, 0, len(seen))
	for l := range seen {
		out = append(out, l)
	}
	sort.Ints(out)
	return out
}

// ---------------------------------------------------------------------------
// Printing — focused on the step-by-step table
// ---------------------------------------------------------------------------

func printContractionTable(g *Graph, steps []contractStep, totalNodes int) {
	fmt.Println()
	fmt.Println("  ── Contraction Preprocessing (Valhalla-style, per-level) ──")
	fmt.Println()
	fmt.Println("  Strategy: contract nodes with exactly 2 same-level neighbors.")
	fmt.Println("            Nodes stay in graph; originals are marked Superseded.")
	fmt.Println("            Higher levels contracted first (highway → arterial → local).")
	fmt.Println()

	// Header
	fmt.Println("  ┌──────┬───────┬─────────────┬──────────────────────────────────┬─────────────────┬───────────────────────┬───────┐")
	fmt.Println("  │ Step │ Level │ Contracted  │ Chain (left --w-- [N] --w-- right) │ Shortcut (w)    │ Via (hidden)          │ Left  │")
	fmt.Println("  ├──────┼───────┼─────────────┼──────────────────────────────────┼─────────────────┼───────────────────────┼───────┤")

	for i, s := range steps {
		lw := edgeGlyph(s.leftSC)
		rw := edgeGlyph(s.rightSC)
		chain := fmt.Sprintf("%s %s%.1f%s [%s] %s%.1f%s %s",
			s.left, lw, s.wLeft, lw, s.contracted, rw, s.wRight, rw, s.right)
		sc := fmt.Sprintf("%s (%.1f)", s.shortcut, s.scWeight)
		via := "—"
		if len(s.via) > 0 {
			via = strings.Join(s.via, "→")
		}
		fmt.Printf("  │ %4d │   L%d  │ %-11s │ %-32s │ %-15s │ %-21s │ %5d │\n",
			i+1, s.level, s.contracted, truncate(chain, 32), truncate(sc, 15), truncate(via, 21), s.nodesActive)
	}
	fmt.Println("  └──────┴───────┴─────────────┴──────────────────────────────────┴─────────────────┴───────────────────────┴───────┘")
	fmt.Println()
	fmt.Printf("  Legend: --w-- real edge    ══w══ shortcut edge (nested)\n")
	fmt.Printf("  Result: %d nodes contracted, %d shortcuts added.  Graph kept at %d nodes.\n",
		len(steps), len(steps), totalNodes)
	fmt.Printf("  Search-visible nodes (not contracted): %d\n", totalNodes-len(steps))
	fmt.Println()
}

func edgeGlyph(isShortcut bool) string {
	if isShortcut {
		return "═"
	}
	return "─"
}

func truncate(s string, n int) string {
	if len(s) <= n {
		return s
	}
	return s[:n-1] + "…"
}

// ---------------------------------------------------------------------------
// Phase 2 — Bidirectional A* on the augmented graph
// ---------------------------------------------------------------------------
//
// The graph still contains every original node. Two things change vs plain
// bi-A*:
//   • Search skips Superseded edges (a shortcut covers the same sub-path).
//   • Search is optionally gated by HierarchyLimits — a Valhalla-style
//     per-level distance bound. Forward search refuses to expand an edge at
//     level L if the current node is farther than ExpandWithin[L] from the
//     start (and symmetrically, backward uses distance-from-goal). Highway
//     level is unlimited. This is what lets real Valhalla scale to continent
//     sized graphs: in the middle of a route only highway edges are ever
//     considered, so explored-node count grows like O(√n) instead of O(n).

// HierarchyLimits mirrors Valhalla's src/thor hierarchy_limits_.
// ExpandWithin is keyed by node/edge level and gives the maximum Euclidean
// distance from the search's "home" endpoint at which edges of that level may
// still be relaxed. Use math.Inf(1) for "always allowed".
type HierarchyLimits struct {
	ExpandWithin map[int]float64
}

// DefaultHierarchyLimits returns the recommended 3-level limits for the demo
// graph: local edges only within 2.5 units of the endpoint, arterial within
// 5, highway unlimited.
func DefaultHierarchyLimits() HierarchyLimits {
	return HierarchyLimits{
		ExpandWithin: map[int]float64{
			0: 2.5,
			1: 5.0,
			2: math.Inf(1),
		},
	}
}

func unlimitedHierarchy(g *Graph) HierarchyLimits {
	levels := map[int]float64{}
	for _, n := range g.Nodes {
		levels[n.Level] = math.Inf(1)
	}
	return HierarchyLimits{ExpandWithin: levels}
}

// CHStats carries the extra counters a Valhalla-style search wants to
// surface: how many edges were rejected by the hierarchy limits, which levels
// were actually touched, and how many shortcut edges appeared in the answer.
type CHStats struct {
	NodesExplored int
	ShortcutsUsed int
	HierarchyCuts int  // edges skipped because of ExpandWithin
	LevelsUsed    []int
}

// BiAStarCH (variadic limits) keeps backward compatibility: callers who
// pass no HierarchyLimits get the unlimited policy (pure superseded-edge
// skipping, no distance gate).
func BiAStarCH(original *Graph, startID, goalID string, limitsOpt ...HierarchyLimits) ([]string, float64, int, int) {
	path, cost, stats := BiAStarCHStats(original, startID, goalID, limitsOpt...)
	return path, cost, stats.NodesExplored, stats.ShortcutsUsed
}

// BiAStarCHStats is the full-result variant used by the 3-way comparison
// table. Returns the reconstructed path, its cost, and search stats.
func BiAStarCHStats(original *Graph, startID, goalID string, limitsOpt ...HierarchyLimits) ([]string, float64, CHStats) {
	g := cloneGraph(original)

	steps := BuildShortcuts(g, startID, goalID)
	printContractionTable(g, steps, len(g.Nodes))

	var lim HierarchyLimits
	limitsActive := len(limitsOpt) > 0
	if limitsActive {
		lim = limitsOpt[0]
	} else {
		lim = unlimitedHierarchy(g)
	}

	if limitsActive {
		fmt.Println("  ── Bi-A* on augmented graph (Valhalla hierarchy limits ON) ──")
		levels := sortedLevelKeys(lim.ExpandWithin)
		parts := []string{}
		for _, l := range levels {
			d := lim.ExpandWithin[l]
			if math.IsInf(d, 1) {
				parts = append(parts, fmt.Sprintf("L%d=∞", l))
			} else {
				parts = append(parts, fmt.Sprintf("L%d=%.1f", l, d))
			}
		}
		fmt.Printf("  Limits (ExpandWithin): %s\n", strings.Join(parts, "  "))
	} else {
		fmt.Println("  ── Bi-A* on augmented graph (no hierarchy limits) ──")
	}
	fmt.Println()

	start := g.Nodes[startID]
	goal := g.Nodes[goalID]
	reverse := buildReverseEdgesCH(g)

	fwdOpen := &priorityQueue{}
	heap.Init(fwdOpen)
	bwdOpen := &priorityQueue{}
	heap.Init(bwdOpen)

	fwdG, bwdG := infMap(g), infMap(g)
	fwdF, bwdF := infMap(g), infMap(g)
	fwdCameFrom := make(map[string]string)
	bwdCameFrom := make(map[string]string)
	fwdInOpen, bwdInOpen := map[string]bool{}, map[string]bool{}
	fwdClosed, bwdClosed := map[string]bool{}, map[string]bool{}

	fwdG[startID] = 0
	fwdF[startID] = heuristic(start, goal)
	heap.Push(fwdOpen, &pqItem{nodeID: startID, fScore: fwdF[startID]})
	fwdInOpen[startID] = true

	bwdG[goalID] = 0
	bwdF[goalID] = heuristic(goal, start)
	heap.Push(bwdOpen, &pqItem{nodeID: goalID, fScore: bwdF[goalID]})
	bwdInOpen[goalID] = true

	mu := math.Inf(1)
	meet := ""
	step := 0
	shortcutsUsed := 0
	hierarchyCuts := 0
	levelsTouched := map[int]bool{}
	fwdEdgeSC := map[string]bool{}
	bwdEdgeSC := map[string]bool{}

	checkMeet := func(id string) {
		if !math.IsInf(fwdG[id], 1) && !math.IsInf(bwdG[id], 1) {
			if c := fwdG[id] + bwdG[id]; c < mu {
				mu = c
				meet = id
			}
		}
	}

	expand := func(
		openSet *priorityQueue,
		gScore, fScore map[string]float64,
		cameFrom map[string]string,
		inOpen, closed map[string]bool,
		edges map[string][]Edge,
		hTarget *Node,
		home *Node, // distance anchor for hierarchy limits
		scMap map[string]bool,
	) {
		it := heap.Pop(openSet).(*pqItem)
		cur := it.nodeID
		delete(inOpen, cur)
		closed[cur] = true
		checkMeet(cur)

		curNode := g.Nodes[cur]
		dHome := heuristic(curNode, home)

		for _, e := range edges[cur] {
			// A shortcut always covers superseded originals at the same cost,
			// so skipping them is lossless and saves work.
			if e.Superseded {
				continue
			}
			// Valhalla hierarchy gate: too far from home to still consider
			// this level. Highway level has math.Inf(1), so it's always in.
			if maxD, ok := lim.ExpandWithin[e.Level]; ok && dHome > maxD {
				hierarchyCuts++
				continue
			}
			if closed[e.To] {
				continue
			}
			ng := gScore[cur] + e.Weight
			if ng >= gScore[e.To] {
				continue
			}
			cameFrom[e.To] = cur
			gScore[e.To] = ng
			fScore[e.To] = ng + heuristic(g.Nodes[e.To], hTarget)
			levelsTouched[e.Level] = true
			if e.IsShortcut {
				scMap[cur+"-"+e.To] = true
			}
			if !inOpen[e.To] {
				heap.Push(openSet, &pqItem{nodeID: e.To, fScore: fScore[e.To]})
				inOpen[e.To] = true
			}
			checkMeet(e.To)
		}
	}

	for fwdOpen.Len() > 0 && bwdOpen.Len() > 0 {
		fwdMin := (*fwdOpen)[0].fScore
		bwdMin := (*bwdOpen)[0].fScore
		if !math.IsInf(mu, 1) && fwdMin >= mu && bwdMin >= mu {
			break
		}
		step++
		if fwdMin <= bwdMin {
			expand(fwdOpen, fwdG, fwdF, fwdCameFrom, fwdInOpen, fwdClosed, g.Edges, goal, start, fwdEdgeSC)
		} else {
			expand(bwdOpen, bwdG, bwdF, bwdCameFrom, bwdInOpen, bwdClosed, reverse, start, goal, bwdEdgeSC)
		}
	}

	if math.IsInf(mu, 1) {
		fmt.Println("  ✗ No path found")
		return nil, -1, CHStats{NodesExplored: step, HierarchyCuts: hierarchyCuts, LevelsUsed: sortedLevelSet(levelsTouched)}
	}

	// Reconstruct contracted path.
	fwdPath := []string{meet}
	for c := meet; ; {
		p, ok := fwdCameFrom[c]
		if !ok {
			break
		}
		fwdPath = append([]string{p}, fwdPath...)
		c = p
	}
	bwdPath := []string{}
	for c := meet; ; {
		p, ok := bwdCameFrom[c]
		if !ok {
			break
		}
		bwdPath = append(bwdPath, p)
		c = p
	}
	contractedPath := append(fwdPath, bwdPath...)

	for i := 0; i < len(contractedPath)-1; i++ {
		from, to := contractedPath[i], contractedPath[i+1]
		if fwdEdgeSC[from+"-"+to] || bwdEdgeSC[to+"-"+from] {
			shortcutsUsed++
			continue
		}
		for _, e := range g.Edges[from] {
			if e.To == to && e.IsShortcut {
				shortcutsUsed++
				break
			}
		}
	}

	levelsUsed := sortedLevelSet(levelsTouched)
	fmt.Printf("  Meeting node : %s\n", meet)
	fmt.Printf("  Contracted   : %s\n", strings.Join(contractedPath, " → "))
	if limitsActive {
		fmt.Printf("  Cost=%.2f  nodes explored=%d  shortcuts used=%d  hierarchy-cuts=%d  levels touched=%v\n",
			mu, step, shortcutsUsed, hierarchyCuts, levelsUsed)
	} else {
		fmt.Printf("  Cost=%.2f  nodes explored=%d  shortcuts used=%d  levels touched=%v\n",
			mu, step, shortcutsUsed, levelsUsed)
	}

	fullPath := RecoverPath(contractedPath, g)
	fmt.Printf("  Full route   : %s\n", strings.Join(fullPath, " → "))
	fmt.Println()

	return fullPath, mu, CHStats{
		NodesExplored: step,
		ShortcutsUsed: shortcutsUsed,
		HierarchyCuts: hierarchyCuts,
		LevelsUsed:    levelsUsed,
	}
}

// sortedLevelKeys returns the keys of a map[int]float64 in ascending order.
func sortedLevelKeys(m map[int]float64) []int {
	out := make([]int, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	sort.Ints(out)
	return out
}

// sortedLevelSet returns the sorted keys of a bool set.
func sortedLevelSet(m map[int]bool) []int {
	out := make([]int, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	sort.Ints(out)
	return out
}

// RecoverPath expands shortcuts using the Via list stored on each edge.
func RecoverPath(contractedPath []string, g *Graph) []string {
	out := []string{contractedPath[0]}
	for i := 0; i < len(contractedPath)-1; i++ {
		from, to := contractedPath[i], contractedPath[i+1]
		var sc *Edge
		for j := range g.Edges[from] {
			if g.Edges[from][j].To == to && g.Edges[from][j].IsShortcut {
				sc = &g.Edges[from][j]
				break
			}
		}
		if sc != nil && len(sc.Via) > 0 {
			out = append(out, sc.Via...)
		}
		out = append(out, to)
	}
	return out
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

func cloneGraph(g *Graph) *Graph {
	c := NewGraph()
	for id, n := range g.Nodes {
		c.Nodes[id] = &Node{ID: n.ID, X: n.X, Y: n.Y, Level: n.Level}
	}
	for from, edges := range g.Edges {
		dup := make([]Edge, len(edges))
		copy(dup, edges)
		c.Edges[from] = dup
	}
	return c
}

func buildReverseEdgesCH(g *Graph) map[string][]Edge {
	rev := make(map[string][]Edge)
	for _, edges := range g.Edges {
		for _, e := range edges {
			rev[e.To] = append(rev[e.To], Edge{
				From: e.To, To: e.From, Weight: e.Weight, Level: e.Level,
				IsShortcut: e.IsShortcut, Superseded: e.Superseded,
				Supersedes: e.Supersedes, Via: reverseStrings(e.Via),
			})
		}
	}
	return rev
}

func reverseStrings(s []string) []string {
	out := make([]string, len(s))
	for i, v := range s {
		out[len(s)-1-i] = v
	}
	return out
}

func infMap(g *Graph) map[string]float64 {
	m := make(map[string]float64, len(g.Nodes))
	for id := range g.Nodes {
		m[id] = math.Inf(1)
	}
	return m
}
