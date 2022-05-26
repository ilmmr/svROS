module ros_base

/* === SIGNATURES === */
abstract sig Node {
	subscribes, advertises : set Topic,
	var inbox, outbox : set Message
}
abstract sig Value {}
abstract sig Message {
	value : one Value
}
abstract sig Topic {
	var box0 : lone Message,
	var box1 : lone Message
}
/* === SIGNATURES === */

/* === ASSUMPTIONS n BEHAVIOURS === */
fact ros_assumptions {
	no inbox + outbox
	no box0  + box1
	topic_behaviour[box0] and topic_behaviour[box1]
	always { all m : Node.outbox | eventually m not in Node.outbox }
}

pred topic_behaviour [box : Topic -> Message] {
	all m : Message, t: Topic | always { m in t.box implies before once (m in (advertises.t).outbox) }
	all m : Message, n: Node  | always { m in n.inbox implies before once (m in (n.subscribes).box) } 
	all m : Message, t: Topic | always { m in t.box implies after (m in (subscribes.t).inbox) }
}

pred publish0[t : Topic, m : Message] {
	m not in t.box0 and m in t.box0'
}
pred publish1[t : Topic, m : Message] {
	m not in t.box1 and m in t.box1'
}
/* === ASSUMPTIONS n BEHAVIOURS === */

