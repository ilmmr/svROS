open util/ordering[Pos]

abstract sig Message {}
sig Pose extends Message {
	pos : one Pos
}
sig Twist extends Message {
	high : one Bool,
	fw : one Bool
}
sig Switch extends Message {
	val : one Bool
}

sig Pos {}
var one sig position0 in Pos {}
var one sig position1 in Pos {}

abstract sig Bool {}
one sig False,True extends Bool {}

 
abstract sig Topic {
	var inbox0 : set Message,
	var inbox1 : set Message
}
one sig Low,High,Vel,Log,Alarm extends Topic {}

pred nop[inbox : Topic->Message,position : Pos] {
	inbox' = inbox
	position' = position
}

pred random[inbox : Topic->Message,position : Pos] {
	some m : Twist | inbox' = inbox ++ Low->m
	position' = position
}

pred light[inbox : Topic->Message,position : Pos] {
	some m : Alarm.inbox | inbox' = inbox - Alarm->m
	position' = position
}

pred safety[inbox : Topic->Message,position : Pos] {
	some m : Log.inbox {
		m.pos = first implies (some v : Twist | v.fw = True and inbox' = (inbox - Log->m) ++ High->v)
		m.pos = last implies (some v : Twist | v.fw = False and inbox' = (inbox - Log->m) ++ High->v)
		m.pos != first and m.pos != last implies inbox' = inbox - Log->m
	}
	position' = position
}

pred multiplexer_high[inbox : Topic->Message,position : Pos] {
	some m : High.inbox {
		High.inbox' = High.inbox - m
		some v : Twist | v.fw = m.fw and v.high = m.high and Vel.inbox' = v
		some p : Switch | p.val = m.high and Alarm.inbox' = p
		Low.inbox' = Low.inbox
		Log.inbox' = Log.inbox
	}
}

pred multiplexer_low[inbox : Topic->Message,position : Pos] {
	no High.inbox
	some m : Low.inbox {
		Low.inbox' = Low.inbox - m
		some v : Twist | v.fw = m.fw and v.high = m.high and Vel.inbox' = v
		some p : Switch | p.val = m.high and Alarm.inbox' = p
		no High.inbox'
		Log.inbox' = Log.inbox
	}
}

/* unifying both predicates multiplexer_low and multiplexer_high */
pred multiplexer[inbox : Topic->Message,position : Pos] {
	multiplexer_high[inbox,position] or multiplexer_low[inbox,position]
	position' = position
}

pred turtle[inbox : Topic->Message,position : Pos] {
	some m : Vel.inbox {
		/* wow */
		m.fw = True implies {position' = max[position + position.next + (m.high = True implies position.next.next else none)]}
		m.fw = False implies {position' = min[position + position.prev + (m.high = True implies position.prev.prev else none)]}
		some p : Pose | p.pos = position' and inbox' = (inbox - Vel->m) ++ Log->p
	}
}

/* --- Network Functionality --- */
fact functionality {
	no inbox0
	always (nop[inbox0,position0] or random[inbox0,position0] or safety[inbox0,position0] or multiplexer[inbox0,position0] or turtle[inbox0,position0])	
	no inbox1
	always (nop[inbox1,position1] or random[inbox1,position1] or safety[inbox1,position1] or multiplexer[inbox1,position1] or turtle[inbox1,position1])	
}

pred RandomLow {
	always (no Low.inbox0 & high.True)
	always (no Low.inbox1 & high.True)
}

pred SafetyLow {
	always (no High.inbox0 & high.True)
	always (no High.inbox1 & high.True)
}

pred publish0[t : Topic, m : Message] {
	m not in t.inbox0 and m in t.inbox0'
}
pred publish1[t : Topic, m : Message] {
	m not in t.inbox1 and m in t.inbox1'
}
/* --- Network Functionality --- */

/* --- Synchronize --- */
pred LowSync {
	historically (all m : Message | (publish0[Low,m] iff publish1[Low,m]) and (publish0[Alarm,m] iff publish1[Alarm,m]))
}
/* system sync - se quiser sincronizar eventos -> stuttering reasoning */
pred SystemSync {
	always (nop[inbox0,position0] iff nop[inbox1,position1])
	always (random[inbox0,position0] iff random[inbox1,position1])
	always (safety[inbox0,position0] iff safety[inbox1,position1])
	always (multiplexer[inbox0,position0] iff multiplexer[inbox1,position1])
	always (turtle[inbox0,position0] iff turtle[inbox1,position1])
}

/* --- EVENT-BASED SYSTEMS --- */
-- "We characterize the correct synchronization of a pair of traces as having updates to low variables only at the same positions."

/* --- Synchronize --- */

check Dependence {
	/* Safety Low -> o Safety never activates the alarm --> No possible private information leaked! */ 
	SafetyLow /* and SystemSync */ and StutterSync implies always (before LowSync implies all m0,m1 : Switch | publish0[Alarm,m0] and publish1[Alarm,m1] implies m0.val = m1.val)
} for 0 but 3 Pos, 8 Message, 1..20 steps
