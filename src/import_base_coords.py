import rospy
#from std_msgs.msg import String
import tf

base_coords_filename = "base_coords.txt"

return_sign = "r"
save_sign = "s"
explanation_msg = f"""Run command 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py', 
provide robot to its base and then enter '{save_sign}' to save its base coordinates or 
'{return_sign}' to return to the menu without saving\n"""


def get_command():
    input_sign = input(explanation_msg)
    if input_sign == save_sign:
        try:
            x, y = get_base_coords()
            write_to_file(x, y)
            print("ok save and return")
            return "save"
        except Exception as e:
            print(f"Error: {e}\n")
            print(f"New base coordinates were not written.\n")
            return "return"

    elif input_sign == return_sign:
        print("ok return")
        return "return"
    else:
        print("Incorrect input, try again")
        return "input_error"


def get_base_coords():
    try:
        rospy.init_node('get_base_coordinates', anonymous=True)
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (transform, _) = listener.lookupTransform(
                    "/map", "/base_scan", rospy.Time(0))
                print(transform)
                x, y = transform[0], transform[1]
                rospy.signal_shutdown("Got coordinates")
                return x, y
                # with open(base_coords_filename, "w") as f:
                #     f.writelines(f"{x} {y}")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            except Exception as e:
                print(f"Error: {e}")
    except rospy.ROSInterruptException:
        pass


def write_to_file(x, y):
    try:
        with open(base_coords_filename, "w") as f:
            #x, y = 1.2, 2.1
            f.writelines(f"{float(x)} {float(y)}")
    except Exception as e:
        print(f"Error: {e}")


# print(get_command())
