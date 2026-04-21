package main

import "fmt"

// runBasicDemos runs the teaching examples for plain A* and Bidirectional A*
// on a tiny 6-node graph. These are isolated from the CH demo so that the
// larger preprocessing logic in main.go can evolve without touching them.
func runBasicDemos() {
	runStandardAStarDemo()
	runBidirectionalAStarDemo()
}

func runStandardAStarDemo() {
	g := NewGraph()

	g.AddNode("A", 0, 1)
	g.AddNode("B", 1, 3)
	g.AddNode("C", 1, 0)
	g.AddNode("D", 3, 3)
	g.AddNode("E", 3, 0)
	g.AddNode("F", 5, 2)

	g.AddBidirectional("A", "B", 3)
	g.AddBidirectional("A", "C", 2)
	g.AddBidirectional("B", "C", 1)
	g.AddBidirectional("B", "D", 4)
	g.AddBidirectional("C", "E", 5)
	g.AddBidirectional("D", "E", 4)
	g.AddBidirectional("D", "F", 3)
	g.AddBidirectional("E", "F", 2)

	fmt.Println("INITIAL MAP:  A↔B(3)  A↔C(2)  B↔C(1)  B↔D(4)  C↔E(5)  D↔E(4)  D↔F(3)  E↔F(2)")

	fmt.Println("\n╔══════════════════════════════════════╗")
	fmt.Println("║      1. Standard A*                  ║")
	fmt.Println("╚══════════════════════════════════════╝")
	_, _, _ = AStar(g, "A", "F")
}

func runBidirectionalAStarDemo() {
	g2 := NewGraph()

	g2.AddNode("A", 0, 1)
	g2.AddNode("B", 1, 3)
	g2.AddNode("C", 1, 0)
	g2.AddNode("D", 3, 3)
	g2.AddNode("E", 3, 0)
	g2.AddNode("F", 5, 2)

	g2.AddBidirectional("A", "B", 3) // A↔B  two-way road
	g2.AddEdge("A", "C", 2)          // A→C  one-way
	g2.AddBidirectional("B", "C", 1) // B↔C  two-way road
	g2.AddEdge("B", "D", 4)          // B→D  one-way
	g2.AddBidirectional("C", "E", 5) // C↔E  two-way road
	g2.AddBidirectional("D", "E", 4) // D↔E  two-way road
	g2.AddBidirectional("D", "F", 3) // D↔F  two-way road
	g2.AddBidirectional("E", "F", 2) // E↔F  two-way road

	fmt.Println("\n╔══════════════════════════════════════╗")
	fmt.Println("║      2. Bidirectional A*             ║")
	fmt.Println("╚══════════════════════════════════════╝")
	fmt.Println()
	fmt.Println("GRAPH (→=one-way ↔=two-way):  A↔B(3)  A→C(2)  B↔C(1)  B→D(4)  C↔E(5)  D↔E(4)  D↔F(3)  E↔F(2)")

	_, _, _ = BiAStar(g2, "A", "F")
}
