/* open util/natural */
module ros-topic

/* Abstract Notations of the ROS elements */
abstract sig Node {
	subscribes, advertises : set Topic,
	var inbox, outbox : set Message
}

abstract sig Message {
	topic : one Topic,
	interface : one Interface
}

abstract sig Interface {}

sig Topic {
	type : one Interface
}

/* Some basic assumptions */
fact ros_assumptions {
	/* A topic must have been advertised or subscribed */
	Topic = Node.advertises + Node.subscribes
	/* Inbox and Outbox messages must consider its Topic message type */
	inbox.topic in subscribes and outbox.topic in advertises
	/* Message interface must be the same as its topic interface */
	topic.type = interface
}
