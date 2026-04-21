package main

import (
	"fmt"
	"math"
	"strings"
)

// ---------------------------------------------------------------------------
// Graph types
// ---------------------------------------------------------------------------

type Node struct {
	ID    string
	X     float64 // map coordinate – used by heuristic
	Y     float64
	Level int // 0=local, 1=arterial, 2=highway (default 0)
}

type Edge struct {
	From       string
	To         string
	Weight     float64
	Level      int      // hierarchy level of this edge (same as endpoints)
	IsShortcut bool     // true if this edge skips contracted nodes
	Superseded bool     // true if a shortcut at this level replaces this edge in the search
	Supersedes []string // base edge keys this shortcut replaces (e.g. ["A-X","X-B"])
	Via        []string // full ordered list of intermediate nodes (for nested shortcut recovery)
}

type Graph struct {
	Nodes map[string]*Node
	Edges map[string][]Edge // adjacency list keyed by node ID
}

func NewGraph() *Graph {
	return &Graph{
		Nodes: make(map[string]*Node),
		Edges: make(map[string][]Edge),
	}
}

func (g *Graph) AddNode(id string, x, y float64) {
	g.Nodes[id] = &Node{ID: id, X: x, Y: y}
}

func (g *Graph) AddNodeWithLevel(id string, x, y float64, level int) {
	g.Nodes[id] = &Node{ID: id, X: x, Y: y, Level: level}
}

func (g *Graph) AddEdge(from, to string, weight float64) {
	level := 0
	if a, ok := g.Nodes[from]; ok {
		if b, ok := g.Nodes[to]; ok {
			if a.Level < b.Level {
				level = a.Level
			} else {
				level = b.Level
			}
		}
	}
	g.Edges[from] = append(g.Edges[from], Edge{From: from, To: to, Weight: weight, Level: level})
}

// AddBidirectional is a convenience for undirected roads.
func (g *Graph) AddBidirectional(a, b string, weight float64) {
	g.AddEdge(a, b, weight)
	g.AddEdge(b, a, weight)
}

// ---------------------------------------------------------------------------
// Priority queue (min-heap on fScore)
// ---------------------------------------------------------------------------

type pqItem struct {
	nodeID string
	fScore float64
	index  int // managed by heap.Interface
}

type priorityQueue []*pqItem

func (pq priorityQueue) Len() int           { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool { return pq[i].fScore < pq[j].fScore }
func (pq priorityQueue) Swap(i, j int)      { pq[i], pq[j] = pq[j], pq[i]; pq[i].index = i; pq[j].index = j }
func (pq *priorityQueue) Push(x interface{}) {
	item := x.(*pqItem)
	item.index = len(*pq)
	*pq = append(*pq, item)
}
func (pq *priorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*pq = old[:n-1]
	return item
}

// ---------------------------------------------------------------------------
// Heuristic: Euclidean distance (admissible & consistent)
// ---------------------------------------------------------------------------

func heuristic(a, b *Node) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(dx*dx + dy*dy)
}

// ---------------------------------------------------------------------------
// 2D grid renderer (used by standard A*)
// ---------------------------------------------------------------------------

const (
	cellW = 10 // horizontal chars per grid unit
	cellH = 4  // vertical   chars per grid unit
)

func renderGrid(g *Graph, current string, closed map[string]bool, openIDs map[string]bool,
	gScore, fScore map[string]float64, path []string) string {

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
				dc := 1
				if c2 < c1 {
					dc = -1
				}
				steps := int(math.Abs(float64(r2 - r1)))
				cSteps := int(math.Abs(float64(c2 - c1)))
				for s := 1; s < steps || s < cSteps; s++ {
					rr := r1 + s*dr
					cc := c1 + int(float64(s)*float64(c2-c1)/float64(steps))
					if rr >= 0 && rr < h && cc >= 0 && cc < w {
						if isPathEdge {
							canvas[rr][cc] = '*'
						} else if dr*dc > 0 {
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

	// Draw nodes
	for _, n := range g.Nodes {
		r, c := toCanvas(n)
		status := " "
		if n.ID == current {
			status = ">"
		} else if pathSet[n.ID] {
			status = "#"
		} else if closed[n.ID] {
			status = "x"
		} else if openIDs[n.ID] {
			status = "o"
		}

		box := fmt.Sprintf("[%s%s]", status, n.ID)
		put(r, c-len(box)/2, box)

		if g, ok := gScore[n.ID]; ok && !math.IsInf(g, 1) {
			score := fmt.Sprintf("g=%.0f f=%.0f", g, fScore[n.ID])
			put(r+1, c-len(score)/2, score)
		}
	}

	var sb strings.Builder
	for _, row := range canvas {
		sb.WriteString("  " + strings.TrimRight(string(row), " ") + "\n")
	}
	return sb.String()
}
