open util/natural

abstract sig Node {
    subscribes, advertises : set Topic,
    var inbox, outbox : set Message
}

abstract sig Message {
    topic : one Topic
}

enum Topic {Velocity, Position}

one sig Random extends Node {} {
    no subscribes
    advertises = Velocity
}

one sig Turtle extends Node {} {
    subscribes = Velocity
    advertises = Position
}

sig Twist extends Message {
    direction : one Direction
}

enum Direction {Fw,Bw}

sig Pose extends Message {
    where : one Natural
}

fact {
    no inbox+outbox
    always (nop or random) -- or turtle or some m : Message | send[m])
}

pred nop {
    outbox' = outbox
    inbox' = inbox
}

pred random {
    inbox' = inbox
}

run {}

check {
    always (some Turtle.outbox implies once some Random.outbox)
}
