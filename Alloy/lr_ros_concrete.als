open ros_base
open util/ordering[pos]

/* --- self compose --- */
-- abstract sig NodeSC extends Node {var inbox0, inbox1, outbox0, outbox1: set Message} {always ((inbox0 + inbox1 = inbox) and (outbox0 + outbox1 = outbox))}
sig pos in numeric {}
var one sig position0 in pos {}
var one sig position1 in pos {}
/* --- self compose --- */

/* --- Node Declaration --- */
one sig Random extends Node {} {
	no subscribes
	advertises in NotSafe
}
one sig Multiplexer extends Node  {} {
	subscribes = NotSafe + Safe
	advertises = MainTopic + Alarm
}
one sig Turtle extends Node {} {
	subscribes = MainTopic
	advertises = TurtlePosition 
}
one sig Safety extends Node {} {
	subscribes = TurtlePosition 
	advertises = Safe
}
one sig Light extends Node {} {
	subscribes = Alarm
	no advertises
}
-- Behaviour relacionado com nós internos: Do tipo valor da pos, se luz está acesa ou não, deverá estar relacionado com um tópico.
/* --- Node Declaration --- */

/* --- Topic Declaration --- */
one sig MainTopic, NotSafe, Safe, TurtlePosition, Alarm extends Topic {}
/* --- Topic Declaration --- */

/* 
--- Interface Declaration --- 
one sig Twist extends Interface {} {
	fields = Direction + Vel
}
one sig Pose extends Interface {} {
	fields = PosX
}
one sig Light_Info extends Interface {} {no fields}
sig _pos, _bool, high, slow, fw, bw extends Value {}
*/
-- NOTA: Para já vou abstrair o vel e a direction como true ou false
one sig data_direction, data_vel, data_position, data_alarm extends Field {}
-- one sig Fw, Bw extends Direction {}
-- one sig High, Slow extends Vel {}
-- one sig neg_3, neg_2, neg_1, neutro, pos_1, pos_2, pos_3 extends _int {}
/* --- Interface Declaration --- */

/* --- Network Functionality --- */
fact type_coherency {
	((topic.(MainTopic + Safe + NotSafe)).content).Value in data_direction and ((topic.(MainTopic + Safe + NotSafe)).content).Value in data_vel
	((topic.TurtlePosition).content).Value in data_position
	((topic.Alarm).content).Value in data_alarm
}
fact field_types {
	Message.content in (data_direction->bool + data_vel->bool + data_position->pos + data_alarm->bool) 
}

fact functionality {
	always (nop[inbox0,outbox0,position0] or random[inbox0,outbox0,position0] or safety[inbox0,outbox0,position0] or multiplexer[inbox0,outbox0,position0] or turtle[inbox0,outbox0,position0] or send[inbox0,outbox0,position0])	
	-- Turtle.pos = next[next[first]] and Turtle.pos = prev[prev[last]] Será importante saber onde ela começa?
	always (nop[inbox1,outbox1,position1] or random[inbox1,outbox1,position1] or safety[inbox1,outbox1,position1] or multiplexer[inbox1,outbox1,position1] or turtle[inbox1,outbox1,position1] or send[inbox1,outbox1,position1])
}
/* --- Network Functionality --- */

/* --- Predicates --- */
pred nop [inbox : Node->Message, outbox : Node->Message, position : pos] {
	inbox' = inbox
	outbox' = outbox
	position' = position
}

pred random [inbox : Node -> Message, outbox : Node -> Message, position : pos] { /* --- random --- */
	some m : Message | m.topic = NotSafe and inbox' = inbox ++ Random->m
	outbox' = outbox
	position' = position
}

pred safety [inbox : Node -> Message, outbox : Node -> Message, position : pos] { /* --- safety node --- */
		/* specify the message direction */
		some m : Safety.inbox {
			data_position.(m.content) = first implies (some t: Message | (t.topic = Safe and data_direction.(t.content) = true)  implies outbox' = outbox ++ Safety->t)
			data_position.(m.content) = last  implies (some t: Message | (t.topic = Safe and data_direction.(t.content) = false) implies outbox' = outbox ++ Safety->t)
			data_position.(m.content) != first and data_position.(m.content) != last implies outbox' = outbox
			inbox' = inbox - Safety->m
		}
		position' = position
}

pred turtle [inbox : Node -> Message, outbox : Node -> Message, position : pos] {
	some m : Turtle.inbox {
		data_direction.(m.content) = true  implies {position' = max[position + position.next + (data_vel.(m.content) = true implies position.next.next else none)]}
		data_direction.(m.content) = false implies {position' = min[position + position.prev + (data_vel.(m.content) = true implies position.prev.prev else none)]}
		inbox'  = inbox - Turtle->m
	}
	/* instead of passing the pos of the turtle, its either return true (first) or false (last) */
	some mn : Message | mn.topic = TurtlePosition and mn.content = data_position->position' and outbox' = outbox ++ Turtle->mn
}

pred send [inbox : Node -> Message, outbox : Node -> Message, position : pos] {
	some n : Node {
		some m : n.outbox {
			no outbox'
			inbox' = inbox ++ subscribes.(m.topic)->m
		}
		position' = position
	}
}

pred multiplexer [inbox : Node -> Message, outbox : Node -> Message, position : pos] { /* --- multiplexer computation --- */
	
	some Multiplexer.inbox

	Safe in (Multiplexer.inbox).topic implies {
		some m : Multiplexer.inbox {
			m.topic = Safe
			/* the topic must change */
			some new_m : Message {
				new_m.topic = MainTopic
				new_m.content = m.content 
				outbox' = outbox ++ Multiplexer->new_m
			}
			some mn: Message | mn.topic = Alarm and data_alarm.(mn.content) = data_vel.(m.content) and inbox' = (inbox - Multiplexer->m) ++ Light->mn	
		}
	}
	else {
		NotSafe in (Multiplexer.inbox).topic implies {
			some m : Multiplexer.inbox {
				m.topic = NotSafe
				/* the topic must change */
				some new_m : Message {
					new_m.topic = MainTopic
					new_m.content = m.content 
					outbox' = outbox ++ Multiplexer->new_m
				}
				some mn: Message | mn.topic = Alarm and data_alarm.(mn.content) = data_vel.(m.content) and inbox' = (inbox - Multiplexer->m) ++ Light->mn
			}
		}
	}
	position' = position
}

/* --- Predicates --- */

pred RandomLow {
	always (no data_vel.(Random.inbox0.content) & true) 
	always (no data_vel.(Random.inbox1.content) & true) 
}

pred SafetyLow {
	always (no data_vel.(Safety.inbox0.content) & true) 
	always (no data_vel.(Safety.inbox1.content) & true) 
}

pred publish0 [n : Node, m : Message] {
	m not in n.inbox0 and m in n.inbox0'
}
pred publish1 [n : Node, m : Message] {
	m not in n.inbox1 and m in n.inbox1'
}

/* --- Synchronize --- */
pred LowSync {
	historically (all m : Message | (publish0[Random,m] iff publish1[Random,m]) and (publish0[Light,m] iff publish1[Light,m]))
}

check Dependence {
	/* Safety Low -> o Safety never activates the alarm --> No possible private information leaked! */ 
	SafetyLow /* and SystemSync */ implies always (before LowSync implies all m0,m1 : Message | publish0[Light,m0] and publish1[Light,m1] implies data_alarm.(m0.content) =  data_alarm.(m1.content))
} for 3 but 3 numeric, 8 Message, 1..30 steps
