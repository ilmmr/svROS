module eulerian_path
open graph

sig Eulerian_Node extends Node {
        adj : set Eulerian_Node // as arestas que saem do Node
        var visited : set Eurelian_Node // as arestas já visitadas
}
one sig Init extends Eulerian_Node {} // o no onde começa e tem que acabar

var one sig Euler in Eulerian_Node {} // o no onde se encontra o Euler em cada momento
