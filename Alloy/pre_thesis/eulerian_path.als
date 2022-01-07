module eulerian_path
/* open graph */

/* Each node as a set of outgoing edges, representing a directed graph without multiple edged. */
sig Node  {
	adj : set Node, 
	var visited : set Node
}
/* {
	always adj' = adj
	once no visited
	// this not in adj
} */

// check { all n : Node | n.adj not in n.(Node<:iden) implies n not in n.adj }

one sig Init extends Node {}
var one sig Euler in Node {}

/* Graph assumptions */
fact eulerian_assumptions {
	Euler = Init
	no visited
	assumptions

	adj = ~adj /* The graph is undirected. */
	no iden & adj /* The graph contains no loops.  */
	Node ->Node in *(adj + ~adj)  /* The graph is connected. */
}

pred assumptions {
	/* Initial configuration */
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


fact traces {
	init
	eventually (adj in visited and Euler in Init)
	always (move or stutter)
}
/* 
fun eur_finish : ENode {
	{ node : ENode | Euler' in node and once Euler in node }
}
*/

run example {
	// eventually some eur_finish
	/* Eurelian Path */
	
} for exactly 5 Node

assert safety_visited {
	always visited in visited'
} 
check safety_visited 

/* 
assert safety_eulerian {

} check safety_eulerian
*/ 
assert safety_euler_visited {
	always (all n :  Node | n in Node.visited implies once Euler = n)
} check safety_euler_visited

assert liveness_euler {
	all n :  Node | eventually Euler = n
} check liveness_euler
/* forall nodes eventually Euler = n -> liveness */
/* se always tudo visited entao para todo o nodo once euler = n -> safety */ 
/* assert eulerian {
	all n : node | 
} */




