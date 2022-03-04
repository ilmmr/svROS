/* open util/natural */
/* module for ros-based topic */
module ros_base

/* --- Abstract Notations of the ROS elements --- */
abstract sig Node {
	subscribes, advertises : set Topic,
	var inbox, outbox : set Message
}
abstract sig Var, Value {}
abstract sig Message {
	topic : one Topic,
	type: one Interface,
	content: Var -> lone Value
}
/* content auxiliar: Message->Value */
let message_value = {m: Message, v: Value | some c: Var | m->c->v in content}
/* content auxiliar_2: Message->Var */
let message_var = {m: Message, v: Var | lone c: Value | m->v->c in content}
/* topic to interface */
let topic_interface = {t: Topic, i: Interface | some m: Message {m.topic = t and m.type = i}}
abstract sig Interface {
	fields: set Var
}
abstract sig Topic {}
/* --- Abstract Notations of the ROS elements --- */

/* --- Some basic assumptions --- */
fact ros_assumptions {
	/* Each node can not have corresponding messages at the beggining */
	no inbox + outbox
	/* A topic must have been advertised or subscribed */
	Topic = Node.advertises + Node.subscribes
	/* Inbox and Outbox messages must consider its Topic message type */
		-- Later we can check as a dynamic property --
	always (inbox.topic in subscribes and outbox.topic in advertises)
	/* Message fields must be preserved */
	content.Value in type.fields
	/* Messages have some content */
	(Message->Message & iden) in message_value.~message_value
	/* 
		Each node has its corresponding topic
		~advertises.advertises in iden
		advertises.~advertises in iden
	*/
}
/* --- Some basic assumptions --- */

/* --- Functionality assumptions --- */
fact ros_functionality {
	always (some m: Node.outbox | (all n: subscribes.(m.topic) | eventually (m in n.inbox)))
	always (some m: Node.outbox | (all n: subscribes.(m.topic) | eventually (m in n.inbox)))
	always (all m: Node.outbox      |  eventually m not in Node.outbox)
}
/* --- Functionality assumptions --- */
