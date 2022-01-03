module eulerian_path
/* open graph */

/* Each node as a set of outgoing edges, representing a directed graph without multiple edged. */
sig ENode  {
	adj : set ENode, 
	var visited : set ENode
}
one sig Init extends ENode {}
var one sig Euler in ENode{}

/* Graph assumptions */
fact graph_assumptions {
	adj = ~adj /* The graph is undirected. */
	no iden & adj /* The graph contains no loops.  */
	ENode ->ENode  in *(adj + ~adj)  /* The graph is connected. */
}

fact assumptions {
	/* Initial configuration */
	Euler = Init
	no visited
	//some adj
	eventually (adj in visited and Euler in Init)
	always (move or stutter)
}

/* move */
pred move {
	some adj_node : Euler.adj {
		adj_node not in Euler.visited
		visited' = visited + Euler->adj_node + adj_node->Euler
		Euler' = adj_node
	}
}

/* stutter */
pred stutter {
	visited'=visited
	Euler'=Euler
}

/* 
fun eur_finish : ENode {
	{ node : ENode | Euler' in node and once Euler in node }
}
*/

run example {
	// eventually some eur_finish
	/* Eurelian Path */
	
} for exactly 5 ENode

assert safety {
	always visited in visited'
} 
check safety 

/* forall nodes eventually Euler = n -> liveness */
/* se always tudo visited entao para todo o nodo once euler = n -> safety */ 
assert eulerian {
	all n : node | 
}




