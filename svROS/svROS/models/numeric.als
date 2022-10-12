module numeric

enum Number { num_neg_10, num_neg_9, num_neg_8, num_neg_7, num_neg_6, num_neg_5, num_neg_4, num_neg_3, num_neg_2, num_neg_1, num_0, num_1, num_2, num_3, num_4, num_5, num_6, num_7, num_8, num_9, num_10 }

// ADD OPERATION => Retrieves the adding of 2 numbers.
fun add [num: Number, add: Number] : Number {
	{
		m : Number | 
			m = num and add = num_0 or 
			// ADD
			m = num.next and add = num_1 or 
			m = num.next.next and add = num_2 or 
			m = num.next.next.next and add = num_3 or 
			m = num.next.next.next.next and add = num_4 or 
			m = num.next.next.next.next.next and add = num_5 or 
			m = num.next.next.next.next.next.next and add = num_6 or 
			m = num.next.next.next.next.next.next.next and add = num_7 or  
			m = num.next.next.next.next.next.next.next.next and add = num_8 or 
			m = num.next.next.next.next.next.next.next.next.next and add = num_9 or 
			m = num.next.next.next.next.next.next.next.next.next.next and add = num_10 or
			// REMOVE
			m = num.prev and add = num_neg_1 or 
			m = num.prev.prev and add = num_neg_2 or 
			m = num.prev.prev.prev and add = num_neg_3 or 
			m = num.prev.prev.prev.prev and add = num_neg_4 or 
			m = num.prev.prev.prev.prev.prev and add = num_neg_5 or 
			m = num.prev.prev.prev.prev.prev.prev and add = num_neg_6 or 
			m = num.prev.prev.prev.prev.prev.prev.prev and add = num_neg_7 or  
			m = num.prev.prev.prev.prev.prev.prev.prev.prev and add = num_neg_8 or 
			m = num.prev.prev.prev.prev.prev.prev.prev.prev.prev and add = num_neg_9 or 
			m = num.prev.prev.prev.prev.prev.prev.prev.prev.prev.prev and add = num_neg_10
	}
}

// REFELEXIVE OPERATION => Retrieves the reverse of a number.
fun reflexive [num: Number] : Number {
	{
		m : Number | 
			m = num_0 and num = num_0 or 
			// POS
			m = num_1 and num = num_neg_1 or
			m = num_2 and num = num_neg_2 or
			m = num_3 and num = num_neg_3 or
			m = num_4 and num = num_neg_4 or
			m = num_5 and num = num_neg_5 or
			m = num_6 and num = num_neg_6 or
			m = num_7 and num = num_neg_7 or
			m = num_8 and num = num_neg_8 or
			m = num_9 and num = num_neg_9 or
			m = num_10 and num = num_neg_10 or
			// NEG
			m = num_neg_1 and num = num_1 or
			m = num_neg_2 and num = num_2 or
			m = num_neg_3 and num = num_3 or
			m = num_neg_4 and num = num_4 or
			m = num_neg_5 and num = num_5 or
			m = num_neg_6 and num = num_6 or
			m = num_neg_7 and num = num_7 or
			m = num_neg_8 and num = num_8 or
			m = num_neg_9 and num = num_9 or
			m = num_neg_10 and num = num_10
	}
}

// MINUS OPERATOR => Retrieves the subtraction of 2 numbers.
fun minus [num: Number, min: Number] : Number {
	{
		m : Number |
			m = add[num, reflexive[min]]
	}
}