open ros_base

/* --- Node Declaration --- */
sig Random extends Node {} {
	no subscribes
	advertises in Topic_Random
}
sig Multiplexer extends Node  {
	subscribes = Topic_Random + Topic_Safety
	advertises = MainTopic + Light
}
sig Turtle extends Node {} {
	subscribes = MainTopic
	advertises = TurtlePosition 
}
sig Safety extends Node {} {
	subscribes = TurtlePosition 
	advertises = Topic_Safety
}
sig LightBubble extends Node {var light: one Bool} {
	subscribes in Light
	no advertises
}

/* --- Topic Declaration --- */
enum Topic {MainTopic, Topic_Random, Topic_Safety, TurtlePosition, Light}

/* --- Interface Declaration --- */
one sig Twist extends Interface {} {
	fields in Direction
}
one sig Pose extends Interface {} {
	fields in PosX
}
one sig Light extends Interface {} {
	fields in Bool
}

enum Var {Bool, Direction, PosX}
