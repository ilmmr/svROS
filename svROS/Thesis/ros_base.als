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
	m not in (channel.(T2.inbox)).elems and m in (channel.(T1.inbox))'.elems
}
pred publish1[channel: Channel, m : Message] {
	m not in (channel.(T2.inbox)).elems and m in (channel.(T2.inbox))'.elems
}
fun active [] : set Node {
	advertises.(Execution.inbox).Message + subscribes.(Execution.inbox).Message
}
/* === INITIAL CONFIG === */