# A list of commands and their dependencies
CommandRequest[] commands
# ex.	[
#			{id: 1, action: "mott", ps_list: [Null, target1], topic_list: ["obj1", Null]},
#			{id: 2, action: "mott", ps_list: [Null, target1], topic_list: ["obj2", Null]},
#			{id: 3, action: "mott", ps_list: [Null, target1], topic_list: ["obj3", Null]},
#		]


int[][] dependencies
# ex. command 2 and 3 have to happen after command 1: [[1,2], [1,3]]