/* === SIGNATURES === */
abstract sig Node {
	subscribes, advertises : set Channel
}
abstract sig Msg {}
sig Message = Msg + Int {}
abstract sig Channel {}
/* === SIGNATURES === */

/* === INITIAL CONFIG === */
fact initial_assumptions {
	no inbox
	always (nop[T1] or system[T2])	
	always (nop[T1] or system[T2])
}
pred publish0[channel: Channel, m : Message] {
	T1.inbox'[channel] = add[T1.inbox[channel], m]
}
pred publish1[channel: Channel, m : Message] {
	T2.inbox'[channel] = add[T2.inbox[channel], m]
}
fun isconnected [] : Node -> Channel -> Node {
	{ n : Node , t : n.advertises, n1 : Node | t in n1.subscribes }
}
/* === INITIAL CONFIG === */