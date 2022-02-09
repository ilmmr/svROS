/* open util/natural */

/* module for ros-based topic */
module ros_abstract

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

abstract sig Topic {
	type : set Interface
}
/* --- Abstract Notations of the ROS elements --- */

/* --- Some basic assumptions --- */
fact ros_assumptions {
	/* A topic must have been advertised or subscribed */
	Topic = Node.advertises + Node.subscribes
	/* Inbox and Outbox messages must consider its Topic message type */
	inbox.topic in subscribes and outbox.topic in advertises
	/* Message interface must be the same as its topic interface */
	topic.type = Message<:type
}
/* --- Some basic assumptions --- */
