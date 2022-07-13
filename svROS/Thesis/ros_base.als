/* === SIGNATURES === */
abstract sig Node {
	subscribes, advertises : set Channel
}
abstract sig Value {}
abstract sig Message {
	value : one (Value + Int)
}
abstract sig Channel {}
/* === SIGNATURES === */

/* === INITIAL CONFIG === */
fact initial_assumptions {
	no inbox
	always (nop[T1] or system[T2])	
	always (nop[T1] or system[T2])
}
pred publish0[channel: Channel, m : Message] {
	channel->m not in T1.inbox and channel->m in T1.inbox'
}
pred publish1[channel: Channel, m : Message] {
	channel->m not in T2.inbox and channel->m in T2.inbox'
}
fun active [] : set Node {
	advertises.(Execution.inbox).Message + subscribes.(Execution.inbox).Message
}
/* === INITIAL CONFIG === */