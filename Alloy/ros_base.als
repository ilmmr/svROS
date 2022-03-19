/* open util/natural */
/* module for ros-based topic */
module ros_base
-- open util/ordering[Id]

/* Noção de ordem em Id */
-- sig Id {}

/* --- Abstract Notations of the ROS elements --- */
abstract sig Node {
	subscribes, advertises : set Topic,
	var inbox0, outbox0 : set Message,
	var inbox1, outbox1 : set Message
}
abstract sig Field, Value {}
abstract sig numeric, string, bool extends Value {}
one sig true, false extends bool {}
abstract sig Message {
	// id: one Id, -- Later for ordering?
	topic : one Topic,
	-- interface: one Interface,
	content: Field -> lone Value
}
-- sig Box in Message {}
/* content auxiliar: Message->Value */
let message_value = {m: Message, v: Value | some f: Field | m->f->v in content}
/* content auxiliar_2: Message->Var */
let message_var = {m: Message, f: Field | lone v: Value | m->f->v in content}
/* topic to interface */
-- let topic_interface = {t: Topic, i: Interface | some m: Message {m.topic = t and m.type = i}}
/* 
	-- Para já vou ignorar a Interface #i
	abstract sig Interface {
		fields: set Field
	} 
*/
abstract sig Topic {}
/* --- Abstract Notations of the ROS elements --- */

/* --- Some basic assumptions --- */
fact ros_assumptions {
	bool = true + false
	/* Each node can not have corresponding messages at the beggining */
	ros_functionality[inbox0, outbox0]
	/* A topic must have been advertised or subscribed */
	Topic = Node.advertises + Node.subscribes
	/* Inbox and Outbox messages must consider its Topic message type */
		-- Later we can check as a dynamic property --
	/* #i - Message fields must be preserved */
	-- content.Value in interface.fields
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
pred ros_functionality [inbox : Node -> Message, outbox : Node -> Message] {
	-- behaviour duplication
	no inbox + outbox
	always (inbox.topic in subscribes and outbox.topic in advertises)
	always (all m: Node.outbox, n: subscribes.(m.topic) | eventually (m in n.inbox)))
	always (all m: Message | m in Node.inbox implies (some n: advertises.(m.topic) | before once (m in n.outbox)))
}
/* --- Functionality assumptions --- */
