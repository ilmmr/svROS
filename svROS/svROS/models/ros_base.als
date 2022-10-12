open numeric
/* === SIGNATURES === */
abstract sig Node {
	subscribes, advertises : set Channel
}
abstract sig Not_Numeric {}
sig Message = Numeric + Not_Numeric
abstract sig Channel {}
/* === SIGNATURES === */

/* === INITIAL CONFIG === */
fact initial_assumptions {
	no inbox
	always (nop[T1] or system[T2])	
	always (nop[T1] or system[T2])
}
pred publish[t : Trace, channel: Channel, m : Message] {
	t.inbox'[channel] = add[t.inbox[channel], m]
}
fun isconnected [] : Node -> Channel -> Node {
	{ n : Node , t : n.advertises, n1 : Node | t in n1.subscribes }
}
/* === INITIAL CONFIG === */