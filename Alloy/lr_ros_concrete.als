open ros_base
open util/ordering[pos] -- o ordering implicitly makes the order signature exact!

/* --- self compose --- */
-- abstract sig NodeSC extends Node {var inbox0, inbox1, outbox0, outbox1: set Message} {always ((inbox0 + inbox1 = inbox) and (outbox0 + outbox1 = outbox))}
sig pos extends Value {} -- se usar o in tenho q especificar o scope do pos 
var one sig position0 in pos {}
var one sig position1 in pos {}
/* --- self compose --- */

/* --- Node Declaration --- */
one sig Random extends Node {} {
	no subscribes
	advertises = NotSafe
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
one sig Light extends Warner {} {
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
	let twist = (topic.(MainTopic + Safe + NotSafe)) {
		(twist.content) in (data_direction->bool + data_vel->bool)
		all t: twist | some data_direction.(t.content) and some data_vel.(t.content)
		-- twist must be entire relation with data_direction and data_vel
		-- let dt = (data_direction.(twist.content)) and dv = (data_vel.(twist.content)) { (iden & Message->Message) in dt.~dt and (iden & Message->Message) in dv.~dv}
	}
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
pred nop [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] {
	inbox' = inbox
	outbox' = outbox
	position' = position
}

pred random [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] { /* --- random --- */
	some m : Message | m.topic = NotSafe and outbox' = outbox ++ Random->m
	inbox' = inbox
	position' = position
}

pred safety [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] { /* --- safety node --- */
		/* specify the message direction */
		some m : Safety.inbox {
			data_position.(m.content) = first implies (some t: Message | t.topic = Safe and data_direction.(t.content) = true  and outbox' = outbox ++ Safety->t)
			data_position.(m.content) = last  implies (some t: Message | t.topic = Safe and data_direction.(t.content) = false and outbox' = outbox ++ Safety->t)
			data_position.(m.content) != first and data_position.(m.content) != last implies outbox' = outbox
			inbox' = inbox - Safety->m
		}
		position' = position
}

pred turtle [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] {
	some m : Turtle.inbox {
		data_direction.(m.content) = true  implies {position' = max[position + position.next + (data_vel.(m.content) = true implies position.next.next else none)]}
		data_direction.(m.content) = false implies {position' = min[position + position.prev + (data_vel.(m.content) = true implies position.prev.prev else none)]}
		inbox'  = inbox - Turtle->m
		/* instead of passing the pos of the turtle, its either return true (first) or false (last) */
		some mn : Message | mn.topic = TurtlePosition and data_position.(mn.content) = position' and outbox' = outbox ++ Turtle->mn
	}
}

pred send [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] {
	some n : Node {
		some m : n.outbox {
			outbox' = outbox - n->m
			inbox' = inbox ++ (subscribes.(m.topic))->m
		}
		position' = position
	}
}
pred multiplexer_high [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] {
	Safe in (Multiplexer.inbox).topic 
	some m : Multiplexer.inbox {
		m.topic = Safe
		/* the topic must change */
		some new_m : Message | new_m.topic = MainTopic and new_m.content = m.content and outbox' = outbox ++ Multiplexer->new_m
		some mn: Message | mn.topic = Alarm and data_alarm.(mn.content) = data_vel.(m.content) and inbox' = (inbox - Multiplexer->m) ++ Light->mn	
	}
}
pred multiplexer_low [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] {
	Safe not in (Multiplexer.inbox).topic
	some m : Multiplexer.inbox {
		/* the topic must change */
		some new_m : Message | new_m.topic = MainTopic and new_m.content = m.content and outbox' = outbox ++ Multiplexer->new_m
		some mn: Message | mn.topic = Alarm and data_alarm.(mn.content) = data_vel.(m.content) and inbox' = (inbox - Multiplexer->m) ++ Light->mn
	}
}
/* --- multiplexer computation --- */	
pred multiplexer [inbox : Executable -> Message, outbox : Executable -> Message, position : pos] {
	some Multiplexer.inbox
	multiplexer_high[inbox,outbox,position] or multiplexer_low[inbox,outbox,position]
	position' = position
}
/* --- Predicates --- */

pred RandomLow {
	always (no data_vel.(Random.outbox0.content) & true) 
	always (no data_vel.(Random.outbox1.content) & true) 
}

pred SafetyLow {
	always (no data_vel.(Safety.outbox0.content) & true) 
	always (no data_vel.(Safety.outbox1.content) & true) 
}

pred receive0 [t : Topic, m : Message] {
	m not in (subscribes.t).inbox0 and m in (subscribes.t).inbox0'
}
pred receive1 [t : Topic, m : Message] {
	m not in (subscribes.t).inbox1 and m in (subscribes.t).inbox1'
}
pred publish0 [n : Node, m : Message] {
	m not in n.outbox0 and m in n.outbox0'
}
pred publish1 [n : Node, m : Message] {
	m not in n.outbox1 and m in n.outbox1'
}

/* --- Synchronize --- */
pred LowSync {
	historically (all m, m1 : Message | (receive0[NotSafe,m] iff receive1[NotSafe,m]) and (receive0[Alarm,m1] iff receive1[Alarm,m1]))
}

check Dependence {
	/* Safety Low -> o Safety never activates the alarm --> No possible private information leaked! */ 
	SafetyLow /* and SystemSync */ implies always (before LowSync implies all m0,m1 : Message | receive0[Alarm,m0] and receive1[Alarm,m1] implies data_alarm.(m0.content) =  data_alarm.(m1.content))
} for 0 but 3 pos, 13 Message, 1..11 steps

run {eventually some Safety.outbox0 }  for 0 but 3 pos, 20 Message, 1..20 steps
run {random[inbox0, outbox0, position0]; send[inbox0, outbox0, position0]; multiplexer[inbox0, outbox0, position0]; send[inbox0, outbox0, position0]; turtle[inbox0,outbox0,position0]} for 0 but 3 pos, 20 Message, 1..20 steps

