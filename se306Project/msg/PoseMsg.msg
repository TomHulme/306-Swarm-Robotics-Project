# A special position message which defines who the sender is
# String sender: The type of robot sending the message
#		Possible Values:
#			farmer
#			sheepdog
#			sheep
#			grass
#			truck
# Pose position: Position of the sender.
#		Pose2D messages are composed as:
#			float64 x
#			float64 y
#			float64 theta

std_msgs/String sender

geometry_msgs/Pose2D position
