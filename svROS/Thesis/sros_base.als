/* === SIGNATURE DECLARATION ==== */
abstract sig Enclave {
	profiles: set Profile
} { some profiles }
abstract sig Profile {
	privileges: set Privilege,
	access: set Privilege
}
abstract sig Privilege {
	object: one Object,
	role  : one Role,
	rule  : one Rule
}
enum Role {Advertise, Subscribe}
enum Rule {Allow, Deny}
abstract sig Object {}
/* === SIGNATURE DECLARATION ==== */

/* === ASSUMPTIONS === */
-- A profile can not have different privileges to the same object.
pred rule_different_privileges {
	all p: Profile | some p1, p2 : p.privileges | p1.role = p2.role and p1.object = p2.object implies p1 = p2
}
-- A profile must constrain its topic access to its privileges.
pred access_in_privileges {
	all p: Profile | p.access in p.privileges
}
/* === ASSUMPTIONS === */

assert valid_configuration_1 {
	rule_different_privileges and not access_in_privileges
} check valid_configuration_1
assert valid_configuration_2 {
	access_in_privileges and not rule_different_privileges
} check valid_configuration_2
assert valid_configuration {
	access_in_privileges and rule_different_privileges
} check valid_configuration

