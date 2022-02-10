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

abstract sig Interface {
	fields: set Var
}

abstract sig Topic {}
/* --- Abstract Notations of the ROS elements --- */

/* --- Some basic assumptions --- */
fact ros_assumptions {
	/* A topic must have been advertised or subscribed */
	Topic = Node.advertises + Node.subscribes
	/* Inbox and Outbox messages must consider its Topic message type */
	inbox.topic in subscribes and outbox.topic in advertises
	/* Message fields must be preserved */
	content.Value in type.fields
}
/* --- Some basic assumptions --- */
