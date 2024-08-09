import mod_search_object as search


if __name__ == "__main__":
    

    robot_position = [0,0]

    #Robot position, Radius of searching circle, samples per iteration
    alg_obj = search.Search_Object(robot_position, 15)

    #Initialize matplotlib
    alg_obj.display_init()

    #Display visualizations
    #alg_obj.display_alg()

    for runs in range(0,2):

        #Generate points 1st batch
        ordered_samples = alg_obj.generate_points()
        #print(type(ordered_samples))
        #alg_obj.display_points()
        #alg_obj.display_alg()

        

        #Loop through generated points (simulating the rover searching)
        for i in range(ordered_samples.shape[1]-1):

            #Set starting position?
            if(i == 0):
                robot_position = ordered_samples[:, 0]
                
            #Sample position
            next_position = ordered_samples[:, i+1]
            line_vector = robot_position - next_position

            
                    
            #How else to handle this?
            alg_obj.update_positions(next_position, line_vector)

            #Draw line on the display
            alg_obj.draw_line([robot_position[0], ordered_samples[0, i+1]], [robot_position[1], ordered_samples[1, i+1]])
            robot_position = ordered_samples[:, i]

            #Dont need to tell algorithm if we found object, as the rover would be controlling whats next
            

            alg_obj.redist_point(robot_position)
            #alg_obj.redist_original()
            alg_obj.update_display()
            #Travel to next position
            robot_position = next_position


    alg_obj.display_alg()

    print("Finished...")