# A list of commands and their dependencies
CommandRequest[] commands
# ex.	[
#			{  id: 1, action: "mott", args: "{ command: "mott", object_pose_topic: "ar_7", target_pose_topic: "target_1", object_pose: {PoseStamped}, target_pose: {PoseStamped} }  }
#			{  id: 2, action: "mott", args: "{ command: "mott", object_pose_topic: "ar_2", target_pose_topic: "target_2", object_pose: {PoseStamped}, target_pose: {PoseStamped} }  }
#			{  id: 3, action: "mott", args: "{ command: "mott", object_pose_topic: "ar_3", target_pose_topic: "target_3", object_pose: {PoseStamped}, target_pose: {PoseStamped} }  }
}",
#			{id: 1, action: "mott", args: "{mottmsg}",
#			{id: 1, action: "mott", args: "{mottmsg}",
#			{id: 2, action: "mott", ps_list: [Null, target1], topic_list: ["obj2", Null]},
#			{id: 3, action: "mott", ps_list: [Null, target1], topic_list: ["obj3", Null]},
#		]


# Dependency[] dependencies
int[] parents
int[] children
# Note: parents and children must have same length
# ex. command 2 and 3 have to happen after command 1: [1, 1] and [2, 3]
# ex. command 3 happens after command 2 and command 2 happens after command 1: [1, 2] and [2, 3]