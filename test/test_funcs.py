"""prompt virtual player for a start and end location"""
start = 0
end = 0 

while start == end:

    while start == 0:

        input_string_srt = raw_input("Enter a start coordinate using an uppercase letter and an integer: ")
        # print(input_string_srt)
        # print(ord(input_string_srt[0]))
        # print(input_string_srt[1])
        

        if len(input_string_srt) != 2: #more than 2 character input 
            print("Please enter a start coordinate 2 characters long, an uppercase letter and an integer")

        elif ord(input_string_srt[0]) > 90:  #entered upper case letters or to high of numbers
            print("Please use upper case letters in your start coordinate")

        elif ord(input_string_srt[0]) > 72 or int(input_string_srt[1]) > 8: #not on board
            print("That start coordinate is not on the board")

        else: 
            start = input_string_srt
            print("Start Coordinate noted")
    

    while end == 0: 

        input_string_end = raw_input("Enter an end coordinate using an uppercase letter and an integer: ")

        if len(input_string_end) != 2: #more than 2 character input 
            print("Please enter an end coordinate 2 characters long, an uppercase letter and an integer")
        
        elif ord(input_string_end[0]) > 90:  #entered upper case letters or to high of numbers
            print("Please use upper case letters in your start coordinate")
            
        elif ord(input_string_end[0]) > 72 or int(input_string_end[1]) > 8: #not on board
            print("That end coordinate is not on the board")
           
        else: 
            end = input_string_end
            print("End Coordinate noted")

print("RoboBoi will move the piece from the start coordinate " + input_string_srt + " to the end coordinate " + input_string_end)
#return start, end = input_string_srt, input_string_end