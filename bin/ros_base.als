/* === SIGNATURES === */
abstract sig Node {
	subscribes, advertises : set Topic
}
abstract sig Not_Numeric {}
sig Message = Not_Numeric + Int {}
abstract sig Topic {}
/* === SIGNATURES === */

/* === INITIAL CONFIG === */
fact system_behaviour {
	always (nop[T1] or system[T1])	
	always (nop[T2] or system[T2])
}
pred publish[t : Trace, topic: Topic, m : Message] {
	not (t.inbox[topic].lastIdx = max[seq/Int]) 
	t.inbox'[topic] = add[t.inbox[topic], m]
}
fun isconnected [] : Node -> Topic -> Node {
	{ n : Node , t : n.advertises, n1 : Node | t in n1.subscribes }
}
/* === INITIAL CONFIG === */