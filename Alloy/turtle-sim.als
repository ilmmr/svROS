open ros-topic

/* Turtle topics */
one HighTopic, LowTopic, Velocity, Position extends Interface {}

/* Nodes involved in the Turtle Multiplexer example */
some sig Controller extends Node {} {
	no subscribes
	advertises in LowTopic + HighTopic
}

one sig Multiplexer extends Node {var count : one Int} {
	subscribes = HighTopic + LowTopic
	advertises = Velocity
	count = 0
}

one sig Turtle extends Node {} {
	subscribes = Velocity
	advertises = Position
}

/* Message definition */
sig Twist extends Message {
	direction : one Direction
} { interface in Position }
enum Direction {Fw,Bw}

/* Pose definition */
sig Pose extends Message {
	var where : one Int
}

/* Network Functionality */
fact functionality {
	Pose = where.0
	no inbox + outbox
	always (some Turtle.inbox implies vel2pose)
	always (nop or low or high or multiplexer or some n: Node | some n.outbox implies send[n])
}

fact turtle_network_assumptions {
	/* Each node has its corresponding topic */
	Topic = Node.advertises
	advertises.~advertises in iden

	/* Inbox and Outbox messages must consider its Topic message type */
	inbox.topic in subscribes and outbox.topic in advertises
}

pred nop {
	outbox' = outbox
	inbox' = inbox
}

/* low topic compute */
pred low {
	/* erase ? Maybe later set to cardinality to one */
	some (advertises.LowTopic).outbox {
		(advertises.LowTopic).outbox = none
	}
	some m : Twist | outbox' = outbox + (advertises.LowTopic)->m
	inbox' = inbox
	where' = where
}

/* high topic compute */
pred high {
	/* erase ? Maybe later set to cardinality to one */
	some (advertises.HighTopic).outbox {
		(advertises.HighTopic).outbox = none
	}
	some m : Twist | outbox' = outbox + (advertises.HighTopic)->m
	inbox' = inbox
	where' = where
}

/* multiplexer compute */
pred multiplexer {
	some Multiplexer.inbox
	HighTopic in (Multiplexer.inbox).topic and Multiplexer.count < 3 implies {
		LowTopic in (Multiplexer.inbox).topic implies {
			count' = count - Multiplexer->(Multiplexer.count) + Multiplexer->(Multiplexer.count + 1)
		}
		some m : Multiplexer.inbox {
			m.topic = HighTopic
			outbox' = outbox + Multiplexer->m
			inbox' = inbox - Multiplexer->m
		}
	}
	else {
		LowTopic in (Multiplexer.inbox).topic implies {
			count' = count - Multiplexer->(Multiplexer.count) + Multiplexer->0
			some m : Multiplexer.inbox {
				m.topic = LowTopic
				outbox' = outbox + Multiplexer->m
				inbox' = inbox - Multiplexer->m
			}
		}
	} 
	where' = where
}

/* Turtle enables its movement */
pred vel2pose {
	some m : Turtle.inbox {
		inbox' = inbox - Turtle->m
		some p : Pose | outbox' = outbox + Turtle->p
		where' = where
	}
}


pred send[n: Node] {
	/* the outbox must have something */
	some m : n.outbox {
		outbox' = outbox - n->m
		inbox' = inbox + subscribes.(m.topic)->m
	}
	where' = where
}

run main {} for exactly 2 Controller, 10 Message

check {
    always (some Turtle.outbox implies once some Controller.outbox)
}
