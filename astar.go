package main

import (
	"container/heap"
	"fmt"
	"math"
	"strings"
)

// ---------------------------------------------------------------------------
// A* with step-by-step debug output
// ---------------------------------------------------------------------------

func AStar(g *Graph, startID, goalID string) ([]string, float64, int) {
	start := g.Nodes[startID]
	goal := g.Nodes[goalID]

	openSet := &priorityQueue{}
	heap.Init(openSet)

	gScore := make(map[string]float64) // cheapest cost from start to node
	fScore := make(map[string]float64) // gScore + heuristic
	cameFrom := make(map[string]string)
	inOpen := make(map[string]bool)

	for id := range g.Nodes {
		gScore[id] = math.Inf(1)
		fScore[id] = math.Inf(1)
	}

	h0 := heuristic(start, goal) // straight-line distance
	gScore[startID] = 0
	fScore[startID] = h0

	heap.Push(openSet, &pqItem{nodeID: startID, fScore: h0})
	inOpen[startID] = true

	step := 0

	fmt.Println("========================================")
	fmt.Printf("A* Search:  %s  →  %s\n", startID, goalID)
	fmt.Println("========================================")
	fmt.Printf("\n[init] start=%s  h(start,goal)=%.2f  f=0+%.2f=%.2f\n\n", startID, h0, h0, h0)
	fmt.Println("Legend:  [>X]=current  [oX]=open  [xX]=closed  [#X]=path")
	fmt.Println()

	closed := make(map[string]bool)

	for openSet.Len() > 0 {
		step++
		item := heap.Pop(openSet).(*pqItem)
		current := item.nodeID
		delete(inOpen, current)

		fmt.Printf("─── step %d ───────────────────────────────────────────\n", step)

		// Show why this node was picked: it has the lowest f in the open set
		fmt.Printf("  Pick : %-4s   f=%.2f  ← lowest f in open set", current, fScore[current])
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

		fmt.Printf("  Why  : g=%.2f (cost so far) + h=%.2f (heuristic) = f=%.2f\n",
			gScore[current],
			fScore[current]-gScore[current],
			fScore[current])

		// Goal reached
		if current == goalID {
			path := reconstructPath(cameFrom, current)
			fmt.Printf("  ★ GOAL REACHED!\n")
			fmt.Printf("  Path : %s\n", strings.Join(path, " → "))
			fmt.Printf("  Cost : %.2f\n", gScore[goalID])
			fmt.Println()
			fmt.Println("========================================")
			return path, gScore[goalID], step
		}

		closed[current] = true
		fmt.Printf("  Close: %-4s   (added to closed set)\n", current)

		edges := g.Edges[current]
		if len(edges) == 0 {
			fmt.Println("  (no outgoing edges)")
			continue
		}

		fmt.Printf("  Neighbors of %s:\n", current)

		for _, edge := range edges {
			neighbor := edge.To
			if closed[neighbor] {
				fmt.Printf("    → %-4s  weight=%.2f  [SKIP – already closed]\n", neighbor, edge.Weight)
				continue
			}

			tentativeG := gScore[current] + edge.Weight
			h := heuristic(g.Nodes[neighbor], goal)
			tentativeF := tentativeG + h

			fmt.Printf("    → %-4s  weight=%.2f  |  tentative g=%.2f+%.2f=%.2f  h=%.2f  f=%.2f",
				neighbor, edge.Weight,
				gScore[current], edge.Weight, tentativeG,
				h, tentativeF)

			if tentativeG < gScore[neighbor] {
				oldG := gScore[neighbor]
				cameFrom[neighbor] = current
				gScore[neighbor] = tentativeG
				fScore[neighbor] = tentativeF

				if math.IsInf(oldG, 1) {
					fmt.Printf("  ✓ NEW  cameFrom[%s]=%s\n", neighbor, current)
				} else {
					fmt.Printf("  ✓ IMPROVED (was g=%.2f)  cameFrom[%s]=%s (updated)\n", oldG, neighbor, current)
				}

				if !inOpen[neighbor] {
					heap.Push(openSet, &pqItem{nodeID: neighbor, fScore: tentativeF})
					inOpen[neighbor] = true
				}
			} else {
				fmt.Printf("  ✗ no improvement (current g=%.2f)\n", gScore[neighbor])
			}
		}

		// Show cameFrom map
		fmt.Printf("  cameFrom: {")
		first := true
		for k, v := range cameFrom {
			if !first {
				fmt.Printf(", ")
			}
			fmt.Printf("%s←%s", k, v)
			first = false
		}
		fmt.Println("}")

		// Show open set state
		fmt.Printf("  Open set: [")
		for i, it := range *openSet {
			if i > 0 {
				fmt.Printf(", ")
			}
			fmt.Printf("%s(f=%.2f)", it.nodeID, it.fScore)
		}
		fmt.Println("]")

	}

	fmt.Println("  ✗ No path found!")
	fmt.Println("========================================")
	return nil, -1, step
}

func reconstructPath(cameFrom map[string]string, current string) []string {
	fmt.Println()
	fmt.Println("  ── Path reconstruction (backtrack via cameFrom) ──")
	fmt.Printf("  cameFrom table: %v\n", cameFrom)
	fmt.Printf("  Start at goal: %s\n", current)

	path := []string{current}
	step := 0
	for {
		prev, ok := cameFrom[current]
		if !ok {
			fmt.Printf("  %s has no parent → reached start!\n", current)
			break
		}
		step++
		fmt.Printf("    [%d] cameFrom[%s] = %s  → go to %s\n", step, current, prev, prev)
		path = append([]string{prev}, path...)
		current = prev
	}
	return path
}
