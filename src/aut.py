#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import sys
from main_state import read_command
from client import base_coords_filename, go_to_base_client, read_file, get_command, clean_room


commands_with_descriptions = {
	'set_base': "set base coordinates for robot",
	'go_to_base': "send robot to base", 
	'clean': "room_name_1, room_name_2, ..., room_name_N: clean provided rooms", 
	'quit': "exit from program", 
	'return': "", 
	'undock': "undock from base"
}

# define state Taking_Command
class Taking_Command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=commands_with_descriptions.keys(), output_keys=['rooms'])#['set_base','go_to_base', 'clean', 'quit', 'return', 'undock'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Taking_Command')
	for command in commands_with_descriptions.keys():
		description = commands_with_descriptions[command]
		if description != "":
			print(command + ": " + description)
	outcome, rooms = read_command()
	if outcome == "clean":
		userdata.rooms = rooms
	return outcome


# define state Going_To_Base
class Going_To_Base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['return'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Going_To_Base')
        try:
                x, y = read_file()
                result = go_to_base_client(x, y)
                if result:
                    print("Robot has successfully reached the base.")
                else:
                    print("Robot hasn't reached the base.")
        except Exception as e:
                print("Error: " + str(e))
        return 'return'
        
class Cleaning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['return'], input_keys=['rooms'])

    def execute(self, userdata):
        for room in userdata.rooms:
                print(clean_room(room))
        rospy.loginfo('Executing state Cleaning')
        return 'return'

class Undock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['return'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Undock')
        try:
                x, y = read_file()
                result = go_to_base_client(x + 1, y)
                if result:
                    print("Robot has successfully undocked.")
                else:
                    print("Robot hasn't undocked.")
        except Exception as e:
                print("Error: ", str(e), "\n")
        return 'return'

class Setting_Base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['return'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setting_Base')
	get_command()
        return 'return'


# define state Start
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Start')
        return 'start'




def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Finish'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('Start', Start(),
                               transitions={'start':'SUB'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['exit'])

        # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('Taking_Command', Taking_Command(), 
                                   transitions={'set_base':'Setting_Base', 
                                                'quit':'exit',
						'clean':'Cleaning',
						'undock':'Undock',
						'return':'Taking_Command',
						'go_to_base':'Going_To_Base'})
            smach.StateMachine.add('Cleaning', Cleaning(), 
                                   transitions={'return':'Taking_Command'})

            smach.StateMachine.add('Undock', Undock(), 
                                   transitions={'return':'Taking_Command'})

            smach.StateMachine.add('Setting_Base', Setting_Base(), 
                                   transitions={'return':'Taking_Command'})

            smach.StateMachine.add('Going_To_Base', Going_To_Base(), 
                                   transitions={'return':'Taking_Command'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'exit':'Finish'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
