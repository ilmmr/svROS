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
var one sig position in Pos {}

abstract sig Bool {}
one sig False,True extends Bool {}

abstract sig Topic {
	var inbox : set Message
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

pred multiplexer[inbox : Topic->Message,position : Pos] {
	multiplexer_high[inbox,position] or multiplexer_low[inbox,position]
	position' = position
}

pred turtle[inbox : Topic->Message,position : Pos] {
	some m : Vel.inbox {
		m.fw = True implies {position' = max[position + position.next + (m.high = True implies position.next.next else none)]}
		m.fw = False implies {position' = min[position + position.prev + (m.high = True implies position.prev.prev else none)]}
		some p : Pose | p.pos = position' and inbox' = (inbox - Vel->m) ++ Log->p
	}
}

fact {
	no inbox
	always (nop[inbox,position] or random[inbox,position] or safety[inbox,position] or multiplexer[inbox,position] or turtle[inbox,position])	
}

pred RandomLow {
	always (no Low.inbox & high.True)
}

pred SafetyLow {
	always (no High.inbox & high.True)
}

run Alarm {
	RandomLow
	eventually some Alarm.inbox & val.True
} for 3 but 5 Pos, 5 Message
