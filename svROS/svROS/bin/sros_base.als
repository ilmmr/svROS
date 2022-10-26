module sros_base

/* === SIGNATURE DECLARATION ==== */
-- ENCLAVE --
abstract sig Enclave {
	profiles: set Profile
} { some profiles }
-- PROFILE --
abstract sig Profile {
	privileges: set Privilege
}
-- PRIVILEGE: Can either be a SROS privilege or a ROS call. --
abstract sig Privilege {
	object: one Object,
	role  : one Role,
	rule  : one Rule
}
-- OBJECT n RULES --
enum Role {Advertise, Subscribe}
enum Rule {Allow, Deny}
abstract sig Object {}
/* === SIGNATURE DECLARATION ==== */

/* === ASSUMPTIONS === */
-- A profile can not have different privileges to the same object.
pred rule_different_privileges [p : Profile] {
	 all p1, p2 : p.privileges | p1.role = p2.role and p1.object = p2.object implies p1 = p2
}
fun different_privileges : Profile -> Role -> Object {
	{ p: Profile, r : Role , o : Object |
		some disj p1, p2 : p.privileges {
			p1.role = p2.role and r = p1.role
			p1.object = p2.object and o = p1.object
		}
	}
}
/* === ASSUMPTIONS === */

assert valid_configuration {
	all p : Profile | rule_access_in_privileges[p]
} check valid_configuration

