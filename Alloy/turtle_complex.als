zopen ros_base
open util/ordering[_int]


/* --- Node Declaration --- */
sig Random extends Node {} {
	no subscribes
	advertises in Topic_Random
}
sig Multiplexer extends Node  {} {
	subscribes = Topic_Random + Topic_Safety
	advertises = MainTopic + Light_Topic
}
sig Turtle extends Node {var pos : one _int} {
	subscribes = MainTopic
	advertises = TurtlePosition 
}
sig Safety extends Node {} {
	subscribes = TurtlePosition 
	advertises = Topic_Safety
}
sig LightBubble extends Node {var light: one _bool} {
	subscribes = Light_Topic
	no advertises
}
/* --- Node Declaration --- */

/* --- Topic Declaration --- */
one sig MainTopic, Topic_Random, Topic_Safety, TurtlePosition, Light_Topic extends Topic {}
/* --- Topic Declaration --- */

/* --- Interface Declaration --- */
one sig Twist extends Interface {} {
	fields = Direction + Vel
}
one sig Pose extends Interface {} {
	fields = PosX
}
one sig Light_Info extends Interface {} {no fields}
sig _int, _bool, high, slow, fw, bw extends Value {}
one sig Direction, Vel, PosX extends Var {}
-- one sig Fw, Bw extends Direction {}
-- one sig High, Slow extends Vel {}
-- one sig neg_3, neg_2, neg_1, neutro, pos_1, pos_2, pos_3 extends _int {}
one sig true, false extends _bool {}
/* --- Interface Declaration --- */

/* --- Network Functionality --- */
fact functionality {
	Turtle.pos = next[next[first]] and Turtle.pos = prev[prev[last]]
	LightBubble.light = false
	always (nop or multiplexer or (some m: Message | random[m] or safety[m]) or (some n: advertises.Topic | some n.outbox implies send[n]) or turtlePos or light)
}
pred nop {
	outbox' = outbox
	inbox' = inbox
	light' = LightBubble->false
	pos' = pos
}
/* isto está muito mal... */
fact messages_assumptions {
	/* Topic assingnments to Interface */
	Topic_Random.topic_interface = Twist and Topic_Safety.topic_interface = Twist and MainTopic.topic_interface = Twist and TurtlePosition.topic_interface = Pose and Light_Topic.topic_interface = Light_Info
	/* Interface Values specification */
	type.Twist in (message_var.Direction & message_var.Vel) and type.Pose in message_var.PosX
	/* Var to Value */
	Message.content in (Direction->fw + Direction->bw + Vel->high + Vel->slow + PosX->_int) 
}
/* --- Network Functionality --- */

/* --- Predicates --- */

pred random [m : Message] { /* --- random --- */
	-- some m : Message {
		m.topic = Topic_Random /* and m.type = Twist */
		outbox' = outbox + Random->m
		inbox' = inbox
		light' = LightBubble->false
		pos' = pos
	-- }
}
pred safety [m : Message] { /* --- safety node --- */
	-- some m : Message {
		m.topic = Topic_Safety /* and m.type = Twist */
		/* specify the message direction */
		some mn : Safety.inbox {
			PosX.(mn.content) = first implies {
				Direction.(m.content) = fw
			}
			else {
				Direction.(m.content) = bw
			}
		}
		outbox' = outbox + Safety->m
		inbox' = inbox
		light' = LightBubble->false
		pos' = pos
	-- }
}
pred multiplexer { /* --- multiplexer computation --- */
	
	some Multiplexer.inbox

	Topic_Safety in (Multiplexer.inbox).topic implies {
		some m : Multiplexer.inbox {
			m.topic = Topic_Safety
			/* the topic must change */
			some new_m : Message {
				new_m.topic = MainTopic
				new_m.content = m.content 
				outbox' = outbox + Multiplexer->new_m
			}
			
			-- Light
			Vel.(m.content) = high implies {
				some mn: Message | mn.topic = Light_Topic and inbox' = inbox - Multiplexer->m + LightBubble->mn
			}
			else {
				inbox' = inbox - Multiplexer->m
			}	
		}
	}
	else {
		Topic_Random in (Multiplexer.inbox).topic implies {
			some m : Multiplexer.inbox {
				m.topic = Topic_Random
				/* the topic must change */
				some new_m : Message {
					new_m.topic = MainTopic
					new_m.content = m.content 
					outbox' = outbox + Multiplexer->new_m
				}
				
				-- Light
				Vel.(m.content) = high implies {
					some mn: Message | mn.topic = Light_Topic and inbox' = inbox - Multiplexer->m + LightBubble->mn
				}
				else {
					inbox' = inbox - Multiplexer->m
				}
			}
		}
	}
	pos' = pos
	light' = LightBubble->false
}
pred turtlePos {
	some m : Turtle.inbox {
		Direction.(m.content) = fw and Vel.(m.content) = high {
			pos' = pos - Turtle->(Turtle.pos) + Turtle->(next[next[Turtle.pos]])
		}
		Direction.(m.content) = fw and Vel.(m.content) = slow {
			pos' = pos - Turtle->(Turtle.pos) + Turtle->(next[Turtle.pos])
		}
		Direction.(m.content) = bw and Vel.(m.content) = high {
			pos' = pos - Turtle->(Turtle.pos) + Turtle->(prev[Turtle.pos])
		}
		Direction.(m.content) = bw and Vel.(m.content) = slow {
			pos' = pos - Turtle->(Turtle.pos) + Turtle->(prev[prev[Turtle.pos]])
		}
		inbox'  = inbox - Turtle->m
	}
	light' = LightBubble->false
	pos' = pos
	/* change turtle pos */
	Turtle.pos = first or Turtle.pos = last implies {
		/* instead of passing the pos of the turtle, its either return true (first) or false (last) */
		Turtle.pos = first implies { 
			some m: Message | m.type = Pose and m.topic = TurtlePosition and m.content = PosX->first and outbox' = outbox + Turtle->m
		}
		else {
			some m: Message | m.type = Pose and m.topic = TurtlePosition and m.content = PosX->last and outbox' = outbox + Turtle->m
		}
	}
	else {
		outbox' = outbox
	}
}
pred send [n : Node] {
	/* must advertise something */
	 -- some n : advertises.Topic | some n.outbox implies {
		/* the outbox must have something */
		some m : n.outbox {
			outbox' = outbox - n->m
			inbox' = inbox + subscribes.(m.topic)->m
		}
		pos' = pos
		light' = LightBubble->false
	-- }
}
pred light {
	some m : LightBubble.inbox {
		outbox' = outbox
		inbox' = inbox - LightBubble->m
		pos' = pos
		light' = LightBubble->true
	}
}
/* --- Predicates --- */

/* --- Test cases --- */
run test_case {
	some m : Message | Vel.(m.content) = slow and Direction.(m.content) = fw and random[m];
	send[Random];
	multiplexer;
	send[Multiplexer];
	turtlePos;
	(send[Turtle] iff some Turtle.outbox) implies {
		/* ... */
	}
}
/* --- Test cases --- */
