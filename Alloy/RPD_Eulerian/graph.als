module graph

/* Each node as a set of outgoing edges, representing a directed graph without multiple edged. */
abstract sig Node {
	adj : set Node
}

fact considerations {
	adj = ~adj /* The graph is undirected. */
	no iden & adj /* The graph contains no loops.  */
	Node->Node in *(adj + ~adj)  /* The graph is connected. */
}

run {} for exactly 5 Node
