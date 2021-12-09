open util/natural

/* Abstract Notations of the ROS elements */
abstract sig Node {
	subscribes, advertises : set Topic,
	var inbox, outbox : set Message
}

abstract sig Message {
	topic : one Topic
}

/* Turtle topics */
enum Topic {HighTopic, LowTopic, Velocity, Position}

/* Nodes involved in the Turtle Multiplexer example */
sig Controller extends Node {} {
	no subscribes
	advertises = LowTopic or HighTopic
}

one sig Multiplexer extends Node {} {
	subscribes = HighTopic + LowTopic
	advertises = Velocity
}

one sig Turtle extends Node {} {
	subscribes = Velocity
	advertises = Position
}

/* Message definition */
sig Twist extends Message {
	direction : one Direction
}
enum Direction {Fw,Bw}

/* Pose definition */
sig Pose extends Message {
	where : one Natural
}

/* Network Functionality */
fact {
	no inbox + outbox
	advertises.~advertises in iden
	some m : Multiplexer.outbox | enableTurtle[m]
	always (nop or low or high or send ) -- or turtle or some m : Message | send[m])
}

pred nop {
	outbox' = outbox
	inbox' = inbox
}

pred low {
	/* erase ? Maybe later set to cardinality to one */
	some (advertises.LowTopic).outbox {
		(advertises.LowTopic).outbox = none
	}
	some m : Twist | outbox' = outbox + (advertises.LowTopic)->m
	inbox' = inbox
}

pred high {
	/* erase ? Maybe later set to cardinality to one */
	some (advertises.HighTopic).outbox {
		(advertises.HighTopic).outbox = none
	}
	some m : Twist | outbox' = outbox + (advertises.HighTopic)->m
	inbox' = inbox
}

run {} for exactly 2 Controller

check {
    always (some Turtle.outbox implies once some Controller.outbox)
}
