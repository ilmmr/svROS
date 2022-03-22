/* --- Dynamic Modelling Tut --- */
open util/ordering[Book]

/* --- --- */
abstract sig Target {}
sig Name extends Target {}
sig Addr extends Target {}
sig Book { addr: Name -> Target }
/* --- --- */
pred init [b: Book] { no b.addr }
pred inv [b: Book] {
  let addr = b.addr | all n: Name {
    n not in n.^addr
    some addr.n => some n.addr
  }
}
fun lookup [b: Book, n: Name] : set Addr {
  n.^(b.addr) & Addr
}
/* --- --- */
assert namesResolve {
  all b: Book | inv[b] =>
    all n: Name | some b.addr[n] => some lookup[b, n]
}
check namesResolve for 4
/* --- --- */
pred add [b, bb: Book, n: Name, t: Target] {
  b != bb and bb in b.^next
  inv[b]
  bb.addr = b.addr + n->t 
}
pred showAdd [b, bb: Book] {
  b != bb and bb in b.^next
  inv[b]
  some n : Name, t : Target | add[b, bb, n, t]
}
run showAdd

pred del [b, bb: Book, n: Name, t: Target] {
	b != bb and bb in b.^next
	inv[b]
	bb.addr = b.addr - n->t 
}

pred inc_dec [b,b1,b2 : Book] {
	some n: Name, t : Target {
		n->t not in b.addr
		add[b,b1,n,t]
		del[b1,b2,n,t]
	}
}
assert dif {
	all disj b,b1,b2 : Book | (inc_dec[b,b1,b2] implies b.addr in b2.addr) }
check dif for 3
