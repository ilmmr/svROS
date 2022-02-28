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
let c_aux = {m:Message, v:Value | some c:Var | m->c->v in content}
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
	inbox.topic in subscribes and outbox.topic in advertises
	/* Message fields must be preserved */
	content.Value in type.fields
	/* Each node has its corresponding topic */
	~advertises.advertises in iden
	advertises.~advertises in iden
	/* Messages have some content */
	(Message->Message & iden) in c_aux.~c_aux
}
/* --- Some basic assumptions --- */

/* --- Functionality assumptions --- */
fact ros_functionality {
	always (some m: Node.outbox | (all n: subscribes.(m.topic) | eventually (m in n.inbox)))
	always (some m: Node.outbox | (all n: subscribes.(m.topic) | eventually (m in n.inbox)))
	always (all m: Node.outbox      |  eventually m not in Node.outbox)
}
/* --- Functionality assumptions --- */
