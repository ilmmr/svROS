module ros_base

/* === SIGNATURES === */
abstract sig Node {
	subscribes, advertises : set Topic,
	var inbox, outbox : set Message
}
abstract sig Value {}
abstract sig Message {
	topic : one Topic,
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
	all n : Node | always { n.inbox.topic in n.subscribes and n.outbox.topic in n.advertises }
	always { all m : Node.outbox | after m not in Node.outbox }
}

pred topic_behaviour [box : Topic -> Message] {
	always { box.~box in iden }
	all m : Message | always {  
		m in Node.inbox  implies { (m in (m.topic).box) and (one n : advertises.(m.topic) | m in n.outbox) }
		m in Node.outbox implies { (m in (m.topic).box) and (all n : subscribes.(m.topic) | m in n.inbox ) }
		m in Topic.box	  implies { (one n : advertises.(m.topic) | m in n.outbox) and (all n : subscribes.(m.topic) | m in n.inbox ) }
	}
	all m : Message, t: Topic | always { m in t.box implies after m not in t.box }
}

pred publish0[t : Topic, m : Message] {
	m not in t.box0 and m in t.box0'
}
pred publish1[t : Topic, m : Message] {
	m not in t.box1 and m in t.box1'
}
/* === ASSUMPTIONS n BEHAVIOURS === */

