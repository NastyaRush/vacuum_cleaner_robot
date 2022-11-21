import sys
from unittest import result
import rospy
from my_service.srv import MoveToBase, CleanRoom
import tf


set_base = "set_base"
go_to_base = "go_to_base"
clean = "clean"
undock = "undock"
quit = "quit"



wrong_command_description = "Wrong command!"
base_coords_filename = "base_coords.txt"

return_sign = "r"
save_sign = "s"
explanation_msg = "Run command 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py', " + \
			"provide robot to its base and then enter 's' to save its base coordinates or " + \
			"'r' to return to the menu without saving\n"


def go_to_base_client(x, y):
    rospy.wait_for_service('go_to_base')
    try:
        go_to_base = rospy.ServiceProxy('go_to_base', MoveToBase)
        resp = go_to_base(x, y)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def clean_room(roomName):
    #rospy.init_node('state_machine', anonymous=True)
    rospy.wait_for_service('clean_room')
    try:
        clean_room = rospy.ServiceProxy('clean_room', CleanRoom)
        resp = clean_room(roomName)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def get_command():
    input_sign = raw_input(explanation_msg)
    if input_sign == save_sign:
        try:
            x, y = get_base_coords()
            write_to_file(x, y)
            print("ok save and return")
            return "return"#"save"
        except Exception as e:
            print("Error: ", str(e), "\n")
            print("New base coordinates were not written.\n")
            #return "return"

    elif input_sign == return_sign:
        print("ok return")
        #return "return"
    else:
        print("Incorrect input, try again.")
        #return "return"#"input_error"


def get_base_coords():
    try:
        #rospy.init_node('state_machine', anonymous=True)
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (transform, _) = listener.lookupTransform(
                    "/map", "/base_scan", rospy.Time(0))
                #print(transform)
                x, y = transform[0], transform[1]
                #rospy.signal_shutdown("Got coordinates.")
                return x, y
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            except Exception as e:
                print("Error: ", str(e), "\n")
		#return "return"
    except rospy.ROSInterruptException:
        pass


def write_to_file(x, y):
    try:
        with open(base_coords_filename, "w") as f:
		
		f.writelines(str(float(x)) + " " + str(float(y)))
    except Exception as e:
        print("Error: ", str(e), "\n")


def read_file():
    with open(base_coords_filename) as f:
        lines = f.read().split(" ")
        x, y = float(lines[0]), float(lines[1])
        return x, y


def read_command():
    arguments = input("\n").split(" ")
    if len(arguments) < 1:
        print(wrong_command_description)
        read_command()
    else:
        command = arguments[0]
        if command == set_base:
            get_command()
            read_command()
        elif command == go_to_base:
            try:
                x, y = read_file()
                result = go_to_base_client(x, y)
                if result:
                    print("Robot has successfully reached the base.")
                else:
                    print("Robot hasn't reached the base.")
            except Exception as e:
                print("Error: ", str(e), "\n")
            read_command()
        elif command == clean:
            rooms = arguments[1:]
            for room in rooms:
                print(clean_room(room))
            read_command()
        elif command == quit:
            return "quit"
        elif command == undock:
            try:
                x, y = read_file()
                result = go_to_base_client(x + 1, y)
                if result:
                    print("Robot has successfully undocked.")
                else:
                    print("Robot hasn't undocked.")
            except Exception as e:
                print("Error: ", str(e), "\n")
            read_command()
        else:
            print(wrong_command_description)
            read_command()


#read_command()
