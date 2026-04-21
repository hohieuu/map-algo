package main

import (
	"container/heap"
	"fmt"
	"math"
	"strings"
)

// This file depends on types from graph.go: Graph, Node, Edge, priorityQueue, pqItem, heuristic, cellW, cellH

// ---------------------------------------------------------------------------
// Bidirectional A* with step-by-step debug output
// ---------------------------------------------------------------------------
//
// Two searches run in alternation:
//   FWD  →  expands from start using g.Edges  (forward edges)
//   BWD  ←  expands from goal  using reverseEdges (reversed edges)
//
// For a one-way edge  A→B  (AddEdge):
//   FWD sees A→B,  BWD sees B→A   (backward can traverse it in reverse)
//
// For a two-way edge A↔B (AddBidirectional):
//   FWD sees A→B and B→A,  BWD sees B→A and A→B
//
// Termination: when the top of BOTH open sets have f ≥ mu (best known path
// cost through any meeting node), we have the optimal path.

// buildReverseEdges creates a reverse adjacency list.
// If the forward graph has edge A→B(w), the reverse has B→A(w).
func buildReverseEdges(g *Graph) map[string][]Edge {
	rev := make(map[string][]Edge)
	for _, edges := range g.Edges {
		for _, e := range edges {
			rev[e.To] = append(rev[e.To], Edge{From: e.To, To: e.From, Weight: e.Weight})
		}
	}
	return rev
}

// BiAStar runs bidirectional A* search with full debug output.
func BiAStar(g *Graph, startID, goalID string) ([]string, float64, int) {
	start := g.Nodes[startID]
	goal := g.Nodes[goalID]
	reverseEdges := buildReverseEdges(g)

	// --- Forward search state (FWD →) ---
	fwdOpen := &priorityQueue{}
	heap.Init(fwdOpen)
	fwdG := make(map[string]float64)
	fwdF := make(map[string]float64)
	fwdCameFrom := make(map[string]string)
	fwdInOpen := make(map[string]bool)
	fwdClosed := make(map[string]bool)

	// --- Backward search state (← BWD) ---
	bwdOpen := &priorityQueue{}
	heap.Init(bwdOpen)
	bwdG := make(map[string]float64)
	bwdF := make(map[string]float64)
	bwdCameFrom := make(map[string]string)
	bwdInOpen := make(map[string]bool)
	bwdClosed := make(map[string]bool)

	// Init scores to infinity
	for id := range g.Nodes {
		fwdG[id] = math.Inf(1)
		fwdF[id] = math.Inf(1)
		bwdG[id] = math.Inf(1)
		bwdF[id] = math.Inf(1)
	}

	// Init forward
	hF := heuristic(start, goal)
	fwdG[startID] = 0
	fwdF[startID] = hF
	heap.Push(fwdOpen, &pqItem{nodeID: startID, fScore: hF})
	fwdInOpen[startID] = true

	// Init backward
	hB := heuristic(goal, start)
	bwdG[goalID] = 0
	bwdF[goalID] = hB
	heap.Push(bwdOpen, &pqItem{nodeID: goalID, fScore: hB})
	bwdInOpen[goalID] = true

	// mu = best known total path cost through any meeting node
	mu := math.Inf(1)
	meetNode := ""

	step := 0

	fmt.Println("========================================")
	fmt.Printf("Bidirectional A*:  %s  →  %s\n", startID, goalID)
	fmt.Println("========================================")
	fmt.Printf("\n[init FWD] start=%s  h(%s,%s)=%.2f  f=%.2f\n", startID, startID, goalID, hF, hF)
	fmt.Printf("[init BWD] goal =%s  h(%s,%s)=%.2f  f=%.2f\n\n", goalID, goalID, startID, hB, hB)
	fmt.Println("Legend:  [>X]=current  [oX]=open  [xX]=closed  [#X]=path")
	fmt.Println("         FWD uses →   BWD uses ←")
	fmt.Println()

	// tracePath returns "A→B(3) + B→D(4) = 7" style breakdown
	tracePath := func(nodeID string, cameFrom map[string]string, gScore map[string]float64, edges map[string][]Edge) string {
		// Build path from start/goal to nodeID
		path := []string{nodeID}
		cur := nodeID
		for {
			prev, ok := cameFrom[cur]
			if !ok {
				break
			}
			path = append([]string{prev}, path...)
			cur = prev
		}
		if len(path) <= 1 {
			return fmt.Sprintf("%s (origin, cost=0)", nodeID)
		}
		// Build "A→B(3) + B→D(4)" string
		var parts []string
		for i := 0; i < len(path)-1; i++ {
			from, to := path[i], path[i+1]
			w := 0.0
			for _, e := range edges[from] {
				if e.To == to {
					w = e.Weight
					break
				}
			}
			parts = append(parts, fmt.Sprintf("%s→%s(%.0f)", from, to, w))
		}
		return fmt.Sprintf("%s = %.2f", strings.Join(parts, " + "), gScore[nodeID])
	}

	// Helper: check meeting and update mu
	checkMeet := func(nodeID string, direction string) {
		if !math.IsInf(fwdG[nodeID], 1) && !math.IsInf(bwdG[nodeID], 1) {
			candidate := fwdG[nodeID] + bwdG[nodeID]
			if candidate < mu {
				old := mu
				mu = candidate
				meetNode = nodeID
				if math.IsInf(old, 1) {
					fmt.Printf("  ★ Meet : %s found in both directions!  (NEW BEST)\n", nodeID)
				} else {
					fmt.Printf("  ★ Meet : %s improves mu!  (was %.2f)\n", nodeID, old)
				}
				fwdTrace := tracePath(nodeID, fwdCameFrom, fwdG, g.Edges)
				bwdTrace := tracePath(nodeID, bwdCameFrom, bwdG, reverseEdges)
				fmt.Printf("           fwdG[%s] = %s\n", nodeID, fwdTrace)
				fmt.Printf("           bwdG[%s] = %s\n", nodeID, bwdTrace)
				fmt.Printf("           mu = fwdG[%s](%.2f) + bwdG[%s](%.2f) = %.2f\n",
					nodeID, fwdG[nodeID], nodeID, bwdG[nodeID], mu)
			}
		}
	}

	// Helper: expand one side
	expandSide := func(
		label string,
		openSet *priorityQueue,
		gScore, fScoreMap map[string]float64,
		cameFrom map[string]string,
		inOpen, closed map[string]bool,
		edges map[string][]Edge,
		hTarget *Node,
	) {
		item := heap.Pop(openSet).(*pqItem)
		current := item.nodeID
		delete(inOpen, current)

		fmt.Printf("  Pick : %-4s   f=%.2f  ← lowest f in %s open", current, fScoreMap[current], label)
		if openSet.Len() > 0 {
			fmt.Printf("  (others: ")
			for i, it := range *openSet {
				if i > 0 {
					fmt.Printf(", ")
				}
				fmt.Printf("%s f=%.2f", it.nodeID, it.fScore)
			}
			fmt.Printf(")")
		}
		fmt.Println()
		fmt.Printf("  Why  : g=%.2f + h=%.2f = f=%.2f\n",
			gScore[current],
			fScoreMap[current]-gScore[current],
			fScoreMap[current])

		closed[current] = true
		fmt.Printf("  Close: %-4s   (added to %s closed set)\n", current, label)

		checkMeet(current, label)

		neighbors := edges[current]
		if len(neighbors) == 0 {
			fmt.Printf("  (no %s edges from %s)\n", label, current)
		} else {
			fmt.Printf("  Neighbors of %s (%s):\n", current, label)
			for _, edge := range neighbors {
				neighbor := edge.To
				if closed[neighbor] {
					fmt.Printf("    → %-4s  weight=%.2f  [SKIP – already in %s closed]\n", neighbor, edge.Weight, label)
					continue
				}

				tentativeG := gScore[current] + edge.Weight
				h := heuristic(g.Nodes[neighbor], hTarget)
				tentativeF := tentativeG + h

				fmt.Printf("    → %-4s  weight=%.2f  |  tentative g=%.2f+%.2f=%.2f  h=%.2f  f=%.2f",
					neighbor, edge.Weight,
					gScore[current], edge.Weight, tentativeG,
					h, tentativeF)

				if tentativeG < gScore[neighbor] {
					oldG := gScore[neighbor]
					cameFrom[neighbor] = current
					gScore[neighbor] = tentativeG
					fScoreMap[neighbor] = tentativeF

					if math.IsInf(oldG, 1) {
						fmt.Printf("  ✓ NEW  cameFrom[%s]=%s\n", neighbor, current)
					} else {
						fmt.Printf("  ✓ IMPROVED (was g=%.2f)  cameFrom[%s]=%s\n", oldG, neighbor, current)
					}

					if !inOpen[neighbor] {
						heap.Push(openSet, &pqItem{nodeID: neighbor, fScore: tentativeF})
						inOpen[neighbor] = true
					}

					// Check if this neighbor creates a meeting
					checkMeet(neighbor, label)
				} else {
					fmt.Printf("  ✗ no improvement (current g=%.2f)\n", gScore[neighbor])
				}
			}
		}

		// Show cameFrom
		fmt.Printf("  %s cameFrom: {", label)
		first := true
		for k, v := range cameFrom {
			if !first {
				fmt.Printf(", ")
			}
			fmt.Printf("%s←%s", k, v)
			first = false
		}
		fmt.Println("}")
	}

	for fwdOpen.Len() > 0 && bwdOpen.Len() > 0 {
		// Termination check: if min-f of both sides >= mu, we're done
		fwdMinF := (*fwdOpen)[0].fScore
		bwdMinF := (*bwdOpen)[0].fScore
		if !math.IsInf(mu, 1) && fwdMinF >= mu && bwdMinF >= mu {
			fmt.Printf("\n─── TERMINATION ─────────────────────────────────────\n")
			fmt.Printf("  FWD min-f=%.2f >= mu=%.2f  AND  BWD min-f=%.2f >= mu=%.2f\n",
				fwdMinF, mu, bwdMinF, mu)
			fmt.Printf("  No better path possible. Optimal meeting at: %s\n", meetNode)
			break
		}

		step++
		fmt.Printf("─── step %d ───────────────────────────────────────────\n", step)
		fmt.Printf("  mu=%.2f  meet=%s  FWD open=%d  BWD open=%d\n", mu, meetNode, fwdOpen.Len(), bwdOpen.Len())

		// Alternate: expand the side with the smaller min-f
		if fwdMinF <= bwdMinF {
			fmt.Printf("  Direction: FWD →  (fwd min-f=%.2f <= bwd min-f=%.2f)\n", fwdMinF, bwdMinF)
			expandSide("FWD", fwdOpen, fwdG, fwdF, fwdCameFrom, fwdInOpen, fwdClosed, g.Edges, goal)
		} else {
			fmt.Printf("  Direction: ← BWD  (bwd min-f=%.2f < fwd min-f=%.2f)\n", bwdMinF, fwdMinF)
			expandSide("BWD", bwdOpen, bwdG, bwdF, bwdCameFrom, bwdInOpen, bwdClosed, reverseEdges, start)
		}

		// Render grid showing both directions
		// Merge open/closed sets for display
		merged := make(map[string]bool)
		for k := range fwdClosed {
			merged[k] = true
		}
		for k := range bwdClosed {
			merged[k] = true
		}
		mergedOpen := make(map[string]bool)
		for k := range fwdInOpen {
			mergedOpen[k] = true
		}
		for k := range bwdInOpen {
			mergedOpen[k] = true
		}
		// Use fwdG for display (pick the side that has a score)
		mergedG := make(map[string]float64)
		mergedF := make(map[string]float64)
		for id := range g.Nodes {
			if !math.IsInf(fwdG[id], 1) && !math.IsInf(bwdG[id], 1) {
				// Both sides know this node — show both
				mergedG[id] = fwdG[id]
				mergedF[id] = bwdG[id] // abuse fScore field to show bwdG
			} else if !math.IsInf(fwdG[id], 1) {
				mergedG[id] = fwdG[id]
				mergedF[id] = fwdF[id]
			} else if !math.IsInf(bwdG[id], 1) {
				mergedG[id] = bwdG[id]
				mergedF[id] = bwdF[id]
			}
		}
		// fmt.Println()
		// fmt.Print(renderBiGrid(g, fwdClosed, bwdClosed, fwdInOpen, bwdInOpen, fwdG, fwdF, bwdG, bwdF, nil))
		// fmt.Println()
	}

	if math.IsInf(mu, 1) {
		fmt.Println("  ✗ No path found!")
		fmt.Println("========================================")
		return nil, -1, step
	}

	// Reconstruct path: forward start→meet + backward meet→goal
	fmt.Println()
	fmt.Println("  ── Path reconstruction ──")
	fmt.Printf("  Meeting node: %s\n", meetNode)

	// Forward half: start → meetNode
	fmt.Printf("\n  Forward half (start → %s):\n", meetNode)
	fmt.Printf("  FWD cameFrom: %v\n", fwdCameFrom)
	fwdPath := []string{meetNode}
	cur := meetNode
	fwdStep := 0
	for {
		prev, ok := fwdCameFrom[cur]
		if !ok {
			fmt.Printf("    %s has no parent → reached start!\n", cur)
			break
		}
		fwdStep++
		fmt.Printf("    [%d] cameFrom[%s] = %s\n", fwdStep, cur, prev)
		fwdPath = append([]string{prev}, fwdPath...)
		cur = prev
	}

	// Backward half: meetNode → goal
	fmt.Printf("\n  Backward half (%s → goal):\n", meetNode)
	fmt.Printf("  BWD cameFrom: %v\n", bwdCameFrom)
	bwdPath := []string{}
	cur = meetNode
	bwdStep := 0
	for {
		prev, ok := bwdCameFrom[cur]
		if !ok {
			fmt.Printf("    %s has no parent → reached goal!\n", cur)
			break
		}
		bwdStep++
		fmt.Printf("    [%d] cameFrom[%s] = %s\n", bwdStep, cur, prev)
		bwdPath = append(bwdPath, prev)
		cur = prev
	}

	fullPath := append(fwdPath, bwdPath...)
	fmt.Printf("\n  Full path: %s\n", strings.Join(fullPath, " → "))
	fmt.Printf("  Total cost: %.2f\n", mu)

	// Final grid with path
	// fmt.Println()
	// fmt.Print(renderBiGrid(g, fwdClosed, bwdClosed, fwdInOpen, bwdInOpen, fwdG, fwdF, bwdG, bwdF, fullPath))
	// fmt.Println("========================================")

	return fullPath, mu, step
}

// renderBiGrid draws the 2D map with bidirectional search state.
// Nodes show which direction discovered them: F=forward, B=backward, M=both (meeting).
func renderBiGrid(g *Graph,
	fwdClosed, bwdClosed map[string]bool,
	fwdOpen, bwdOpen map[string]bool,
	fwdG, fwdF, bwdG, bwdF map[string]float64,
	path []string) string {

	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, n := range g.Nodes {
		if n.X < minX {
			minX = n.X
		}
		if n.X > maxX {
			maxX = n.X
		}
		if n.Y < minY {
			minY = n.Y
		}
		if n.Y > maxY {
			maxY = n.Y
		}
	}

	cols := int(maxX-minX) + 1
	rows := int(maxY-minY) + 1
	w := cols*cellW + 1
	h := rows*cellH + 1

	canvas := make([][]byte, h)
	for i := range canvas {
		canvas[i] = make([]byte, w)
		for j := range canvas[i] {
			canvas[i][j] = ' '
		}
	}

	put := func(r, c int, s string) {
		for i, ch := range s {
			if r >= 0 && r < h && c+i >= 0 && c+i < w {
				canvas[r][c+i] = byte(ch)
			}
		}
	}

	toCanvas := func(n *Node) (int, int) {
		col := int(n.X-minX)*cellW + cellW/2
		row := (rows-1-int(n.Y-minY))*cellH + cellH/2
		return row, col
	}

	pathSet := make(map[string]bool)
	pathEdgeSet := make(map[string]bool)
	for i, p := range path {
		pathSet[p] = true
		if i > 0 {
			pathEdgeSet[path[i-1]+"-"+p] = true
			pathEdgeSet[p+"-"+path[i-1]] = true
		}
	}

	// Draw edges
	drawn := make(map[string]bool)
	for from, edges := range g.Edges {
		for _, e := range edges {
			key := from + "-" + e.To
			rev := e.To + "-" + from
			if drawn[key] || drawn[rev] {
				continue
			}
			drawn[key] = true

			nFrom := g.Nodes[from]
			nTo := g.Nodes[e.To]
			r1, c1 := toCanvas(nFrom)
			r2, c2 := toCanvas(nTo)

			edgeChar := byte('-')
			isPathEdge := pathEdgeSet[key]

			if r1 == r2 {
				if isPathEdge {
					edgeChar = '='
				}
				minC, maxC := c1, c2
				if minC > maxC {
					minC, maxC = maxC, minC
				}
				for c := minC + 1; c < maxC; c++ {
					canvas[r1][c] = edgeChar
				}
				midC := (minC + maxC) / 2
				label := fmt.Sprintf("%.0f", e.Weight)
				put(r1-1, midC-len(label)/2, label)
			} else if c1 == c2 {
				vChar := byte('|')
				if isPathEdge {
					vChar = '#'
				}
				minR, maxR := r1, r2
				if minR > maxR {
					minR, maxR = maxR, minR
				}
				for r := minR + 1; r < maxR; r++ {
					canvas[r][c1] = vChar
				}
				midR := (minR + maxR) / 2
				label := fmt.Sprintf("%.0f", e.Weight)
				put(midR, c1+1, label)
			} else {
				dr := 1
				if r2 < r1 {
					dr = -1
				}
				steps := int(math.Abs(float64(r2 - r1)))
				cSteps := int(math.Abs(float64(c2 - c1)))
				for s := 1; s < steps || s < cSteps; s++ {
					rr := r1 + s*dr
					cc := c1 + int(float64(s)*float64(c2-c1)/float64(steps))
					if rr >= 0 && rr < h && cc >= 0 && cc < w {
						if isPathEdge {
							canvas[rr][cc] = '*'
						} else if dr*int(c2-c1) > 0 {
							canvas[rr][cc] = '\\'
						} else {
							canvas[rr][cc] = '/'
						}
					}
				}
				midR := (r1 + r2) / 2
				midC := (c1 + c2) / 2
				label := fmt.Sprintf("%.0f", e.Weight)
				put(midR, midC+1, label)
			}
		}
	}

	// Draw nodes with bidirectional status
	for _, n := range g.Nodes {
		r, c := toCanvas(n)

		// Determine status
		inFwd := fwdClosed[n.ID] || fwdOpen[n.ID] || !math.IsInf(fwdG[n.ID], 1)
		inBwd := bwdClosed[n.ID] || bwdOpen[n.ID] || !math.IsInf(bwdG[n.ID], 1)

		status := " "
		if pathSet[n.ID] {
			status = "#"
		} else if inFwd && inBwd {
			status = "M" // meeting — both directions know this node
		} else if fwdClosed[n.ID] {
			status = "F" // forward closed
		} else if bwdClosed[n.ID] {
			status = "B" // backward closed
		} else if fwdOpen[n.ID] {
			status = "f" // forward open
		} else if bwdOpen[n.ID] {
			status = "b" // backward open
		}

		box := fmt.Sprintf("[%s%s]", status, n.ID)
		put(r, c-len(box)/2, box)

		// Scores below: show both sides if available
		hasFwd := !math.IsInf(fwdG[n.ID], 1)
		hasBwd := !math.IsInf(bwdG[n.ID], 1)

		if hasFwd && hasBwd {
			score := fmt.Sprintf("Fg=%.0f Bg=%.0f", fwdG[n.ID], bwdG[n.ID])
			put(r+1, c-len(score)/2, score)
		} else if hasFwd {
			score := fmt.Sprintf("Fg=%.0f f=%.0f", fwdG[n.ID], fwdF[n.ID])
			put(r+1, c-len(score)/2, score)
		} else if hasBwd {
			score := fmt.Sprintf("Bg=%.0f f=%.0f", bwdG[n.ID], bwdF[n.ID])
			put(r+1, c-len(score)/2, score)
		}
	}

	var sb strings.Builder
	for _, row := range canvas {
		sb.WriteString("  " + strings.TrimRight(string(row), " ") + "\n")
	}
	return sb.String()
}
