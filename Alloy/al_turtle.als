open ros_base
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

sig _int, _bool, high, slow, fw, bw extends Value {}
one sig Direction, Vel, PosX extends Var {}
-- one sig Fw, Bw extends Direction {}
-- one sig High, Slow extends Vel {}
-- one sig neg_3, neg_2, neg_1, neutro, pos_1, pos_2, pos_3 extends _int {}
one sig true, false extends _bool {}
/* --- Interface Declaration --- */

/* --- Network Functionality --- */
fact functionality {
	Turtle.pos = first
	LightBubble.light = false
	always (nop or multiplexer or (some m: Message | random[m] or safety[m]) or (some n: (Node-LightBubble) | some n.outbox implies send[n]) or turtlePos)
}
pred nop {
	outbox' = outbox
	inbox' = inbox
	light' = LightBubble->false
	pos' = pos
}
/* --- Network Functionality --- */

/* --- Predicates --- */
pred random[m : Message] { /* --- random --- */
	m.topic = Topic_Random and m.type = Twist
	outbox' = outbox + Random->m
	inbox' = inbox
	light' = LightBubble->false
	pos' = pos
}
pred safety[m : Message] { /* --- safety node --- */
	m.topic = Topic_Safety and m.type = Twist
	outbox' = outbox + Safety->m
	inbox' = inbox
	light' = LightBubble->false
	pos' = pos
}
pred multiplexer { /* --- multiplexer computation --- */
	some Multiplexer.inbox

	Topic_Safety in (Multiplexer.inbox).topic implies {
		some m : Multiplexer.inbox {
			m.topic = Topic_Safety
			outbox' = outbox + Multiplexer->m
			inbox' = inbox - Multiplexer->m
			
			-- Light
			Vel.(m.content) = high implies {
				light' = LightBubble->true
			}
			else {
				light' = LightBubble->false
			}	
		}
	}
	else {
		Topic_Random in (Multiplexer.inbox).topic implies {
			some m : Multiplexer.inbox {
				m.topic = Topic_Random
				outbox' = outbox + Multiplexer->m
				inbox' = inbox - Multiplexer->m
				
				-- Light
				Vel.(m.content) = high implies {
					light' = LightBubble->true
				}
				else {
					light' = LightBubble->false
				}
			}
		}
	}
	pos' = pos
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
	outbox' = outbox
	light' = LightBubble->false
	processPos
}
pred processPos {
	Turtle.pos = first or Turtle.pos = last
	inbox'  = inbox
	some m: Message | m.type = Pose and m.topic = TurtlePosition and m.content = PosX->Turtle.pos and outbox' = outbox + Turtle->m
	light' = LightBubble->false
	pos' = pos
}
pred send[n : Node] {
	/* the outbox must have something */
	some m : n.outbox {
		outbox' = outbox - n->m
		inbox' = inbox + subscribes.(m.topic)->m
	}
	pos' = pos
	light' = LightBubble->false
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

