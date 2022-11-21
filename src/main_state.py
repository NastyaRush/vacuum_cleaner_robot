set_base = "set_base"
go_to_base = "go_to_base"
clean = "clean"
quit = "quit"
undock = "undock"

wrong_command_description = "Wrong command!"


def read_command():
    arguments = raw_input("\n").split(" ")
    if len(arguments) < 1:
        print(wrong_command_description)
        read_command()
    else:
	command = arguments[0]
	if len(arguments) > 1:
        	if command == clean:
        		rooms = arguments[1:]
            		return "clean", rooms
		else:
			print(wrong_command_description)
	    	return "return", None
	else:
        	if command == set_base:
            		return "set_base", None
        	elif command == go_to_base:
           		print("go_to_base")
	    		return "go_to_base", None
        	elif command == quit:
            		return "quit", None
        	elif command == undock:
            		return "undock", None
        	else:
           		print(wrong_command_description)
	    		return "return", None

