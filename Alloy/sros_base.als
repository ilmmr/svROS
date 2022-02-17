/* open util/natural */

/* module for sros-based */
module sros_base

/* --- SROS basics configuration template --- */
abstract sig Enclave {
	profiles: set Profile
} { some profiles }

abstract sig Profile {
	privileges: set Privilege
}

abstract sig Privilege {
	role : one Role,
	object: one Object,
	value: one Value
}

enum Role {Publish, Subscribe}
enum Value {Alloy, Deny}
abstract sig Object {}
/* --- SROS basics configuration template --- */

/* --- Some basic assumptions --- */
fact sros_assumptions {
	/* A profile can not have different priveleges to the same object */
	all p: Profile | some p1, p2 : p.privileges | p1.role = p2.role and p1.object = p2.object implies p1 = p2
}
/* --- Some basic assumptions --- */
